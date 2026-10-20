"""Measured-tree continuous scanning; reuse certified survey chunks and A*."""
import math
import numpy as np
from .tree_mission import TreeInspectionController
from .survey import SurveyController
from .control import MissionController
from .continuous import ContinuousTrajectory
from .mapping import OCCUPIED


class ContinuousTreeInspectionController(TreeInspectionController):
    def __init__(self,config,grid,sensor_mode=False,asynchronous=True):
        sensor_opt_in = config.get('sensor_rolling_localisation') or config.get('sensor_fixed_localisation')
        if (sensor_mode and not sensor_opt_in) or not (config.get('simulation_continuous_roll') or config.get('lidar_fixed')):
            raise ValueError('continuous tree scan currently requires rolling GT simulation')
        super().__init__(config,grid,sensor_mode,asynchronous)
        self.scan_orbit_ready=False
        self.scan_start_aligned=False
        self.scan_sector_records=[]
        self.scan_settled_since=None
        self.scan_handoff={}
        self.scan_target_repairs=[]
        self.scan_target_diagnostics={}
        self.scan_height_diagnostics={}
        self.scan_curve_diagnostics={}
        self.scan_endpoint_rejections=[]
        self.transit_curve_diagnostics={}
        self.navigation_camera_active=False
        self.navigation_camera_completed=None
        self.scan_anchor_count=int(self.c.get('tree_scan_anchors',24))
        if self.scan_anchor_count<12 or self.scan_anchor_count%self.c['tree_view_count']:
            raise ValueError('scan anchors must be >=12 and divisible by sector count')
        self.c['continuous_survey']=True
        self.c['tree_continuous_ascent']=bool(self.c.get('rolling_canopy_ascent',False))

    def _mount_phase(self,phase,now):
        if phase=='NORMALIZE' and self.c.get('rolling_canopy_ascent'):
            # Rolling LiDAR stays rolling: no pitch-normalization phase.
            self.inspection_altitude=float(self.command[2])
            phase='SCAN'
        super()._mount_phase(phase,now)

    def _prepare_trajectory(self,now,position,range_value=None,range_stamp=None):
        if self.tree_phase=='ASCEND':
            return False  # The live ascent reference owns motion, never A* here.
        if self.tree_phase!='SCAN':
            before=self.tree_phase
            ready=super()._prepare_trajectory(now,position,range_value,range_stamp)
            if before=='APPROACH' and self.tree_phase=='SCAN' and self.c.get('rolling_canopy_ascent'):
                self._mount_phase('ASCEND',now)
            return ready
        return self._prepare_scan(now,position,range_value,range_stamp)

    def _begin_continuous_scan(self,position):
        measured=np.asarray(position).copy()
        position=measured.copy()
        if self.inspection_altitude is not None:
            # Never promote ascent overshoot to a higher, uncertified target.
            position[2]=self.inspection_altitude
        self.scan_handoff=dict(policy='retain_certified_ascent_stop_height',
            measured_start_height_m=float(measured[2]),reference_height_m=float(position[2]))
        self.scan_settled_since=None
        centre=np.asarray(self.active_tree['xy'])
        delta=np.asarray(position)[:2]-centre
        radius=float(np.linalg.norm(delta))
        angle=math.atan2(delta[1],delta[0])
        angles=angle+np.arange(1,self.scan_anchor_count+1)*2*math.pi/self.scan_anchor_count
        self.waypoints=np.column_stack((centre[0]+radius*np.cos(angles),
                                       centre[1]+radius*np.sin(angles),
                                       np.full(len(angles),position[2])))
        self.scan_nominal_waypoints=self.waypoints.copy()
        self.scan_initial_height=float(position[2])
        self.scan_target_repairs=[]
        self.scan_target_diagnostics={}
        self.waypoint_index=0
        self.view_target=self.goal=self.trajectory=None
        self.batch_end_index=None
        self.scan_orbit_ready=True
        self.scan_start_aligned=False
        self.scan_sector_records=[]
        self.nominal=[np.asarray(position).tolist(),*self.waypoints.tolist()]

    def _choose_goal(self,current):
        if self.tree_phase=='SCAN' and self.scan_orbit_ready and self.state=='SURVEY':
            if self.c.get('scan_adaptive_height'):
                from .canopy_scan import canopy_profile
                self.waypoints,self.scan_height_diagnostics=canopy_profile(self.grid,
                    self.waypoints,self.waypoint_index,self.command,self.active_tree,self.c,
                    self.scan_initial_height)
            self.goal=self.waypoints[self.waypoint_index].copy()
            endpoint_safe,_=self._scan_endpoint_safe(self.goal,current)
            if self.c.get('scan_target_repair') and (self.grid.occupied_goal(self.goal) or not endpoint_safe):
                self._repair_scan_target(current)
            return self._scan_endpoint_safe(self.goal,current)[0]
        return super()._choose_goal(current)

    def _scan_exit_endpoint(self,index=None):
        index=self.waypoint_index if index is None else index
        return (index==len(self.waypoints)-1 and
                self.c.get('transit_lateral_observation_guard',0.)>0.)

    def _scan_endpoint_grid(self,grid,index=None):
        if not self._scan_exit_endpoint(index):return grid
        from .transit_guard import transit_guard_grid
        return transit_guard_grid(grid,self.c.get('transit_observation_guard',0.),
            self.c.get('transit_lateral_observation_guard',0.))

    def _scan_endpoint_safe(self,position,previous,grid=None):
        if grid is None:
            grid=self.grid
            if self.c.get('scan_adaptive_height'):
                from .canopy_scan import canopy_guard_grid
                grid=canopy_guard_grid(grid,self.c.get('tree_ascent_observation_guard',.2))
        grid=self._scan_endpoint_grid(grid)
        if not grid.safe(position):
            return False,dict(reason='scan_exit_not_transit_guarded_free' if
                self._scan_exit_endpoint() else 'scan_endpoint_not_guarded_free')
        from .local_braking_retime import endpoint_reserve_check
        return endpoint_reserve_check(position,np.asarray(position)-previous,grid,self.c)

    def _batch_endpoint_safe(self,points,index):
        if self.tree_phase!='SCAN':
            return True
        from .local_braking_retime import route_terminal_reserve_check
        safe,detail=route_terminal_reserve_check(points,self._scan_endpoint_grid(self.grid,index),self.c)
        if not safe:
            self.scan_endpoint_rejections.append(dict(anchor=int(index),**detail))
        return safe

    def _repair_scan_target(self,current,grid=None):
        """Resample the SAME angular anchor, never skip an inspection sector.

        Only at rest in the planner snapshot. A free endpoint is not a flight
        certificate: the shared A*, curve and braking checks still own that.
        """
        index=self.waypoint_index
        nominal=self.scan_nominal_waypoints[index]
        delta=nominal[:2]-np.asarray(self.active_tree['xy'])
        radius=float(np.linalg.norm(delta))
        direction=delta/max(radius,1e-9)
        top=float(self.waypoints[index,2])
        # The final scan anchor also prepares departure under nearby leaves.
        # Use the transit altitude floor within the SAME bounded scan descent;
        # map/body/ground clearance still decides whether any candidate is safe.
        floor_limit=self.c.get('exploration_min_altitude',1.) if self._scan_exit_endpoint() else self.c['takeoff_altitude']
        floor=max(float(floor_limit),float(nominal[2])-self.c.get('scan_target_max_descent',.6))
        step=self.grid.res
        rejects=dict(occupied=0,unknown_or_outside=0,terminal_reserve=0)
        candidates=[]
        candidate_grid=self.grid if grid is None else grid
        if grid is None and self.c.get('scan_adaptive_height'):
            from .canopy_scan import canopy_guard_grid
            candidate_grid=canopy_guard_grid(self.grid,self.c.get('tree_ascent_observation_guard',.2))
        heights=np.arange(top,floor-1e-8,-step)
        if top>=floor:
            heights=np.unique(np.r_[heights,floor])[::-1]
        for offset in np.arange(0.,self.c.get('scan_target_max_outward',1.)+1e-8,step):
            for height in heights:
                p=np.r_[nominal[:2]+offset*direction,height]
                if candidate_grid.occupied_goal(p):
                    rejects['occupied']+=1
                elif not candidate_grid.safe(p):
                    rejects['unknown_or_outside']+=1
                elif not self._scan_endpoint_safe(p,current,candidate_grid)[0]:
                    rejects['terminal_reserve']+=1
                else:
                    score=float(abs(offset)+3*(top-height)+.05*np.linalg.norm(p-current))
                    candidates.append((score,p))
        detail=dict(policy='same_angle_observed_free_resampling',tree_id=self.active_tree['id'],
            anchor=int(index),nominal=nominal.tolist(),blocked_target=self.goal.tolist(),
            map_version=int(self.grid.version),rejections=rejects,candidate_count=len(candidates),
            observation_stamp=float(self.grid.last_observation_stamp),
            departure_reserve_required=self._scan_exit_endpoint(),
            original_blockage=self.grid.describe_blockage(self.grid.indices(self.goal)),
            sector_skipped=False,endpoint_is_not_route_certificate=True)
        if not candidates:
            self.scan_target_diagnostics=dict(detail,result='no_observed_safe_candidate')
            return
        candidates.sort(key=lambda item:item[0])
        # A detour can enter from a different direction than the direct chord.
        # Check that actual route's final tangent before accepting the endpoint.
        selected=None
        if self.c.get('feedback_speed_envelope'):
            from .planning import plan
            from .local_braking_retime import route_terminal_reserve_check
            rejects.update(route_unavailable=0,route_terminal_reserve=0)
            evaluated=0
            for _,candidate in candidates[:24]:
                evaluated+=1
                if candidate_grid.line_safe(self.command,candidate):
                    route=[self.command,candidate]
                else:
                    route=plan(candidate_grid,self.command,candidate,
                               self.c.get('planner_max_expansions',50000))
                if route is None:
                    rejects['route_unavailable']+=1
                elif not route_terminal_reserve_check(route,self._scan_endpoint_grid(candidate_grid),self.c)[0]:
                    rejects['route_terminal_reserve']+=1
                else:
                    selected=candidate
                    break
            detail['route_candidates_evaluated']=evaluated
            if selected is None:
                self.scan_target_diagnostics=dict(detail,result='no_route_terminal_reserve_candidate')
                return
        else:
            selected=next((p for _,p in candidates if candidate_grid.line_safe(self.command,p)),candidates[0][1])
        self.waypoints[index]=selected
        # A lower roof applies to the remaining ring. Do not repeatedly climb
        # back into the old blocked height at every following angular anchor.
        if not self.c.get('scan_adaptive_height'):
            self.waypoints[index:,2]=np.minimum(self.waypoints[index:,2],selected[2])
        self.goal=selected.copy()
        detail.update(result='proposed_requires_route_certificate',selected=selected.tolist(),
            remaining_height_m=float(selected[2]),
            direct_corridor_safe=bool(candidate_grid.line_safe(self.command,selected)))
        self.scan_target_diagnostics=detail
        self.scan_target_repairs.append(detail)


    def _accept_plan_auxiliary(self,clone,ready):
        self.transit_curve_diagnostics=clone.transit_curve_diagnostics
        if self.tree_phase=='SCAN' and self.scan_orbit_ready:
            self.scan_target_diagnostics=clone.scan_target_diagnostics
            self.scan_height_diagnostics=clone.scan_height_diagnostics
            self.scan_curve_diagnostics=clone.scan_curve_diagnostics
            self.scan_endpoint_rejections=clone.scan_endpoint_rejections
            if ready:
                self.waypoints=clone.waypoints.copy()
                self.scan_target_repairs=clone.scan_target_repairs


    def _history_goal(self,current,destination):
        grid=self.grid
        guard=self._route_observation_guard()
        if guard>0:self.grid=self._observation_grid(grid,guard)
        try:
            return super()._history_goal(current,destination)
        finally:
            self.grid=grid

    def _observation_grid(self,grid,guard):
        transit=self.state=='RETURN' or (self.state=='SURVEY' and self.tree_phase=='APPROACH')
        if transit:
            from .transit_guard import transit_guard_grid
            return transit_guard_grid(grid,guard,self.c.get('transit_lateral_observation_guard',0.))
        from .canopy_scan import canopy_guard_grid
        remaining=max(0.,guard-getattr(grid,'canopy_guard_cells',0)*grid.res)
        return canopy_guard_grid(grid,remaining) if remaining else grid

    def _route_observation_guard(self):
        if self.tree_phase=='SCAN' and self.c.get('scan_adaptive_height'):
            return self.c.get('tree_ascent_observation_guard',.2)
        if self.state=='RETURN' or (self.state=='SURVEY' and self.tree_phase=='APPROACH'):
            return self.c.get('transit_observation_guard',0.)
        return 0.

    def _plan_route(self,start,goal):
        self.scan_endpoint_rejections=[]
        grid=self.grid
        guard=self._route_observation_guard()
        if guard>0:self.grid=self._observation_grid(grid,guard)
        try:
            route=super()._plan_route(start,goal)
            if (route is not None and self.state=='SURVEY' and self.tree_phase=='SCAN'
                    and self.c.get('feedback_speed_envelope') and self.c.get('scan_target_repair')
                    and self.batch_end_index==self.waypoint_index):
                from .local_braking_retime import route_terminal_reserve_check
                if not route_terminal_reserve_check(route,self._scan_endpoint_grid(self.grid),self.c)[0]:
                    original=self.goal.copy()
                    self._repair_scan_target(start,grid=self.grid)
                    if np.linalg.norm(self.goal-original)>1e-8:
                        route=super()._plan_route(start,self.goal)
            return route
        finally:
            self.grid=grid


    def _camera_heading(self,position):
        delta=np.asarray(self.active_tree['xy'])-np.asarray(position)[:2]
        return math.atan2(delta[1],delta[0])-self.c.get('camera_mount_yaw',0.)

    def _record_pass(self,index,now,position):
        step=self.scan_anchor_count//self.c['tree_view_count']
        if (index+1)%step:return
        yaw_error=math.atan2(math.sin(self._camera_heading(position)-self.body_yaw_measured),
                             math.cos(self._camera_heading(position)-self.body_yaw_measured))
        measured=self.mount_feedback.get('measured',{})
        targets=self.mount_feedback.get('targets',{})
        useful=(abs(yaw_error)<.2 and self.mount_feedback.get('feedback_fresh',False)
                and abs(measured.get('camera_pitch',99)-targets.get('camera_pitch',0))<.1)
        record=dict(tree_id=self.active_tree['id'],view=int((index+1)//step-1),
                    anchor=index,position=np.asarray(position).tolist(),scan_stamp=float(self.grid.last_observation_stamp),
                    camera_policy='continuous_front_tree' if self.c.get('lidar_fixed') else 'continuous_left_tree',inspection_valid=bool(useful),
                    body_yaw_error_rad=float(yaw_error),commanded_speed_mps=float(np.linalg.norm(self.command_v)))
        self.scan_sector_records.append(record)
        if useful:self.scan_records.append(record)
        self.inspection_view=len([r for r in self.scan_sector_records if r['inspection_valid']])

    def _on_motion(self,now,position):
        if self.tree_phase=='SCAN' and self.state=='SURVEY' and self.scan_orbit_ready:
            self.goal_yaw=self._camera_heading(position)
            before=self.waypoint_index
            SurveyController._on_motion(self,now,position)
            if self.waypoint_index>before:self._record_pass(before,now,position)
        else:
            super()._on_motion(now,position)

    def _prepare_scan(self,now,position,range_value=None,range_stamp=None):
        if not self.scan_orbit_ready:
            self._begin_continuous_scan(position)
        # A pitch-only front camera cannot simultaneously face the tree and
        # inspect below a tangential route. Observe at rest only when unknown
        # blocks progress, then restore tree-facing alignment before motion.
        if self.c.get('fixed_scan_depth_observation') and self.state=='SURVEY':
            completed=self.navigation_camera_completed
            observed_here=(completed is not None and completed[0]==self.waypoint_index
                           and np.linalg.norm(np.asarray(position)-completed[1])<.3)
            needs_view=(self.route_blockage.get('unknown_count',0)>0 and not observed_here)
            if self.navigation_camera_active or needs_view:
                if not self.navigation_camera_active:
                    self.camera_gate={}
                    self.navigation_camera_active=True
                if not self._camera_observation_ready(now,np.asarray(position)):
                    return False
                self.navigation_camera_completed=(self.waypoint_index,np.asarray(position).copy())
                self.navigation_camera_active=False
                self.scan_start_aligned=False
                return False
        if self.state=='SURVEY' and not self.scan_start_aligned:
            target=self._camera_heading(position)
            dt=0. if self.mount_tick_stamp is None else max(0.,min(.1,now-self.mount_tick_stamp))
            self.mount_tick_stamp=now
            difference=math.atan2(math.sin(target-self.yaw),math.cos(target-self.yaw))
            self.yaw+=float(np.clip(difference,-self.c['yaw_rate']*dt,self.c['yaw_rate']*dt))
            self.goal_yaw=target
            error=math.atan2(math.sin(target-self.body_yaw_measured),math.cos(target-self.body_yaw_measured))
            measured=self.mount_feedback.get('measured',{})
            targets=self.mount_feedback.get('targets',{})
            if self.inspection_altitude is not None:
                settled=(abs(position[2]-self.inspection_altitude)<.08 and self.vehicle_speed<.08)
                self.scan_settled_since=(now if self.scan_settled_since is None else self.scan_settled_since) if settled else None
                self.scan_handoff.update(height_error_m=float(position[2]-self.inspection_altitude),
                    measured_speed_mps=float(self.vehicle_speed),settled=bool(settled))
                if self.scan_settled_since is None or now-self.scan_settled_since<.3:
                    if now-self.mount_phase_started>self.c.get('tree_mount_timeout',20.):
                        self.fail('scan_height_settling_timeout')
                    return False
            if (abs(error)>.08 or not self.mount_feedback.get('feedback_fresh')
                    or abs(measured.get('camera_pitch',99)-targets.get('camera_pitch',0))>.08):
                return False
            self.scan_start_aligned=True
        if self.wait_for_scan is not None:
            if self.grid.last_observation_stamp<=self.wait_for_scan:return False
            if self._scan_exit_endpoint() and not self._scan_endpoint_grid(self.grid).safe(self.command):
                # A newly integrated observation can revoke departure reserve
                # after arrival. Repair the same final sector before counting it.
                self.wait_for_scan=None;self.goal=None
                self.trajectory=None;self.elapsed=0.;self.batch_end_index=None
                return False
            self._record_pass(self.waypoint_index,now,position)
            self.wait_for_scan=None
            self.waypoint_index+=1
            MissionController._advance(self)
            self.trajectory=None;self.elapsed=0.;self.batch_end_index=None
            if self.waypoint_index==len(self.waypoints):
                if len([r for r in self.scan_sector_records if r['inspection_valid']])!=self.c['tree_view_count']:
                    self.fail('continuous_scan_camera_coverage_incomplete')
                    return False
                self.completed_trees.append(self.active_tree['id'])
                self.completed_centres.append(list(self.active_tree['xy']))
                self.active_tree=None;self.tree_phase='SEARCH';self.scan_orbit_ready=False
                self.view_target=None;self.inspection_view=0
                self.waypoint_index=0
            return False
        if self.state=='SURVEY' and self.trajectory is None and self.pending_plan is None:
            self.goal=None  # Re-evaluate the next canopy chunk, including new observations.
        return SurveyController._prepare_trajectory(self,now,position,range_value,range_stamp)

    def camera_navigation_view(self):
        if self.c.get('fixed_scan_depth_observation') and self.tree_phase=='SCAN':
            return self.navigation_camera_active
        return super().camera_navigation_view()

    def _trajectory_from_points(self,points,speed=None):
        grid=self.grid
        adaptive=self.tree_phase=='SCAN' and self.c.get('scan_adaptive_height')
        guard=self._route_observation_guard()
        if guard>0:self.grid=self._observation_grid(grid,guard)
        try:
            trajectory=SurveyController._trajectory_from_points(self,points,speed)
        finally:
            self.grid=grid
        if self.tree_phase=='SCAN' and not isinstance(trajectory,ContinuousTrajectory):
            raise ValueError('continuous scan smoothing not certified')
        if guard>0 and isinstance(trajectory,ContinuousTrajectory):
            from .canopy_scan import CanopyScanTrajectory,BoundaryCanopyTrajectory
            from .moving_trajectory import BoundaryTrajectory
            wrapper=BoundaryCanopyTrajectory if isinstance(trajectory,BoundaryTrajectory) else CanopyScanTrajectory
            lateral=self.c.get('transit_lateral_observation_guard',0.) if not adaptive else 0.
            if lateral>0:
                from .transit_guard import TransitObservationTrajectory,BoundaryTransitTrajectory
                wrapper=BoundaryTransitTrajectory if isinstance(trajectory,BoundaryTrajectory) else TransitObservationTrajectory
                trajectory=wrapper(trajectory,guard,lateral)
            else:trajectory=wrapper(trajectory,guard)
            trajectory.safe(self.grid)
            if adaptive:self.scan_curve_diagnostics=trajectory.canopy_certificate
            else:self.transit_curve_diagnostics=trajectory.canopy_certificate
        return trajectory

    def inspection_status(self):
        status=super().inspection_status()
        status['fixed_scan_navigation_camera_active']=self.navigation_camera_active
        status['scan_target_diagnostics']=self.scan_target_diagnostics
        status['scan_target_repairs']=self.scan_target_repairs
        status['scan_height_profile']=self.scan_height_diagnostics
        status['scan_curve_diagnostics']=self.scan_curve_diagnostics
        status['scan_endpoint_rejections']=self.scan_endpoint_rejections
        status['transit_curve_diagnostics']=self.transit_curve_diagnostics
        ids=np.argwhere(self.grid.state==OCCUPIED)
        ceiling=None
        if len(ids):
            centres=self.grid.centers(ids)
            overhead=(np.linalg.norm(centres[:,:2]-self.command[:2],axis=1)<=self.grid.radius+self.grid.res
                      ) & (centres[:,2]-self.grid.res/2>self.command[2])
            if np.any(overhead):ceiling=float(np.min(centres[overhead,2])-self.grid.res/2)
        status.update(scan_motion_policy=('continuous_CCW_front_camera_certified_stop_to_stop_chunks'
                      if self.c.get('lidar_fixed') else 'continuous_CCW_left_camera_certified_stop_to_stop_chunks'),
                      scan_anchor=self.waypoint_index,scan_anchor_count=self.scan_anchor_count,
                      scan_sector_records=self.scan_sector_records,
                      scan_start_aligned=self.scan_start_aligned,
                      continuous_scan=True,
                      scan_handoff=self.scan_handoff,
                      canopy_ascent_enabled=bool(self.c.get('rolling_canopy_ascent')),
                      observed_overhead_surface_z_m=ceiling,
                      observed_top_envelope_gap_m=None if ceiling is None else ceiling-self.command[2]-self.c['drone_dimensions'][2]/2,
                      canopy_gap_requested_m=self.c.get('tree_canopy_gap'),
                      route_execution_certificate=getattr(self.trajectory,'execution_certificate',{}),
                      canopy_stop_is_contact=False)
        return status
