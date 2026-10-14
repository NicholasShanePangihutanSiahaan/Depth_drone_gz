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
        if (sensor_mode and not config.get('sensor_rolling_localisation')) or not config.get('simulation_continuous_roll'):
            raise ValueError('continuous tree scan currently requires rolling GT simulation')
        super().__init__(config,grid,sensor_mode,asynchronous)
        self.scan_orbit_ready=False
        self.scan_start_aligned=False
        self.scan_sector_records=[]
        self.scan_settled_since=None
        self.scan_handoff={}
        self.approach_transition={}
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
            if ready and self.trajectory is not None:
                self.approach_transition=getattr(self.trajectory,'blend_diagnostics',self.approach_transition)
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
        self.waypoint_index=0
        self.view_target=self.goal=self.trajectory=None
        self.batch_end_index=None
        self.scan_orbit_ready=True
        self.scan_start_aligned=False
        self.scan_sector_records=[]
        self.nominal=[np.asarray(position).tolist(),*self.waypoints.tolist()]

    def _choose_goal(self,current):
        if self.tree_phase=='SCAN' and self.scan_orbit_ready and self.state=='SURVEY':
            self.goal=self.waypoints[self.waypoint_index].copy()
            return self.grid.safe(self.goal)
        return super()._choose_goal(current)

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
                    camera_policy='continuous_left_tree',inspection_valid=bool(useful),
                    body_yaw_error_rad=float(yaw_error),commanded_speed_mps=float(np.linalg.norm(self.command_v)))
        self.scan_sector_records.append(record)
        if useful:self.scan_records.append(record)
        self.inspection_view=len([r for r in self.scan_sector_records if r['inspection_valid']])

    def _on_motion(self,now,position):
        if (self.tree_phase=='APPROACH' and self.state=='SURVEY'
                and getattr(self.trajectory,'approach_ascent_blend',False)
                and self.elapsed>=self.trajectory.duration):
            # Preserve p/v/a; the ascent starts from the same moving vertical
            # boundary, not from measured overshoot or a rest setpoint.
            from .ascent import ContinuousAscent
            velocity,acceleration=self.command_v.copy(),self.command_a.copy()
            self.approach_transition.update(handoff_stamp=float(now),
                handoff_position=self.command.tolist(),handoff_velocity=velocity.tolist(),
                handoff_acceleration=acceleration.tolist(),handoff_policy='moving_C2_vertical_boundary')
            self._mount_phase('ASCEND',now)
            height=float(np.clip(self.active_tree['observed_stem_top']-.4,self.c['takeoff_altitude'],5.5))
            self.ascent_reference=ContinuousAscent(height,self.c)
            self.ascent_nominal_acceleration=acceleration
            self.view_target=np.r_[self.command[:2],height]
            return
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
        return SurveyController._prepare_trajectory(self,now,position,range_value,range_stamp)

    def _trajectory_from_points(self,points,speed=None):
        trajectory=SurveyController._trajectory_from_points(self,points,speed)
        if (self.c.get('smooth_approach_ascent') and self.tree_phase=='APPROACH'
                and self.state=='SURVEY' and self.active_tree is not None
                and self.view_target is not None and len(points)==2
                and np.linalg.norm(np.asarray(points[-1])-self.view_target)<1e-6):
            from .approach_blend import build_approach_blend
            blend,detail=build_approach_blend(self.grid,points[0],points[-1],self.active_tree,self.c)
            if blend is not None:
                self.goal=self.view_target=blend.sample(blend.duration)[0].copy()
                return blend
            trajectory.blend_diagnostics=detail
        if self.tree_phase=='SCAN' and not isinstance(trajectory,ContinuousTrajectory):
            raise ValueError('continuous scan smoothing not certified')
        return trajectory

    def _sample_next(self,dt,position,velocity):
        result=super()._sample_next(dt,position,velocity)
        if getattr(self.trajectory,'approach_ascent_blend',False):
            detail={}
            if not self.trajectory.candidate_safe(self.grid,*result[:3],diagnostics=detail):
                self.approach_transition.update(runtime_guard=detail)
                self.fail('approach_ascent_braking_guard')
        return result

    def final_braking_config(self):
        if getattr(self.trajectory,'approach_ascent_blend',False):
            return self.trajectory.c
        if self.c.get('smooth_approach_ascent') and self.tree_phase=='ASCEND':
            return dict(self.c,reaction_time=max(self.c['reaction_time'],self.c.get('tree_ascent_reaction_time',.7)))
        return self.c

    def final_reference_guard(self,acceleration):
        from .planning import brake_safe
        self.final_reference_guard_diagnostics={}
        # The extra shifted observation buffer decides when to START braking.
        # A certified stop must retain real body/reference stopping clearance,
        # not an additional future-ascent buffer above its stopping corridor.
        ref=getattr(self,'ascent_reference',None)
        if self.tree_phase=='ASCEND' and ref is not None and ref.stop is not None:
            self.final_reference_guard_diagnostics=dict(policy='certified_ascent_braking',
                observation_buffer_required=False,ascent_stop_reason=ref.reason,
                actual_and_reference_braking_still_required=True)
            return True
        if not (getattr(self.trajectory,'approach_ascent_blend',False)
                or self.c.get('smooth_approach_ascent') and self.tree_phase=='ASCEND'):
            return True
        guarded=self.command+np.array([0.,0.,self.c.get('tree_ascent_observation_guard',0.)])
        detail={}
        safe=bool(self.grid.line_safe(self.command,guarded) and brake_safe(self.grid,guarded,
            self.command_v,self.final_braking_config(),acceleration,diagnostics=detail))
        self.final_reference_guard_diagnostics=dict(policy='continue_ascent_observation_buffer',
            observation_buffer_required=True,safe=safe,braking=detail)
        return safe

    def inspection_status(self):
        status=super().inspection_status()
        status['approach_ascent_transition']=self.approach_transition
        ids=np.argwhere(self.grid.state==OCCUPIED)
        ceiling=None
        if len(ids):
            centres=self.grid.centers(ids)
            overhead=(np.linalg.norm(centres[:,:2]-self.command[:2],axis=1)<=self.grid.radius+self.grid.res
                      ) & (centres[:,2]-self.grid.res/2>self.command[2])
            if np.any(overhead):ceiling=float(np.min(centres[overhead,2])-self.grid.res/2)
        status.update(scan_motion_policy='continuous_CCW_left_camera_certified_stop_to_stop_chunks',
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
