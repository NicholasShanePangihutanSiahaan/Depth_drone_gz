"""Measured-tree inspection supervisor over the existing survey/flight stack."""
import math
import numpy as np
from .survey import SurveyController
from .control import MissionController
from .sensor_mounts import LIMITS


class TreeInspectionController(SurveyController):
    def __init__(self,config,grid,sensor_mode=False,asynchronous=True):
        if config.get('automatic_tilt_ascent'):
            if sensor_mode or not config.get('sensor_gimbals_enabled'):
                raise ValueError('automatic tilt ascent requires ground-truth gimbal simulation')
            angles = np.asarray(config.get('tree_tilt_angles',[.25,.7]),float)
            step = config.get('tree_ascent_step',.4)
            if angles.ndim!=1 or not len(angles) or not np.all(np.isfinite(angles)) or np.any((angles<0.) | (angles>LIMITS['lidar_pitch'][1])):
                raise ValueError('tree tilt angles must fit simulated pitch limits')
            if not math.isfinite(step) or not 0<step<=.5:
                raise ValueError('tree ascent step must be in (0, .5] metres')
        if int(config['tree_view_count']) != config['tree_view_count'] or config['tree_view_count'] < 1:
            raise ValueError('tree_view_count must be a positive integer')
        distances = np.asarray(config['tree_view_distances'],float)
        if distances.ndim != 1 or not len(distances) or not np.all(np.isfinite(distances)) or np.any(distances <= 0):
            raise ValueError('tree_view_distances must contain finite positive distances')
        super().__init__(config,grid,sensor_mode,asynchronous)
        self.catalog = []
        self.catalog_stamp = -math.inf
        self.tree_phase = 'SEARCH'
        self.active_tree = None
        self.completed_trees = []
        self.completed_centres = []
        self.inspection_view = 0
        self.view_target = None
        self.search_started = None
        self.finish_reason = ''
        self.tree_angle = 0.
        self.scan_records = []
        self.perception_statistics = {}
        self.rejected_views = []
        self.mount_feedback = {}
        self.mount_phase_started = None
        self.mount_settled_since = None
        self.mount_angle_index = 0
        self.mount_tick_stamp = None
        self.body_yaw_measured = 0.
        self.vehicle_speed = 0.
        self.inspection_altitude = None
        self.ascent_stop_reason = ''
        self.ascent_records = []
        self.view_settled_since = None
        self.depth_feedback = {}
        self.camera_gate = {}
        self.ascent_reference = None
        self.ascent_nominal_acceleration = np.zeros(3)
        self.ascent_pause_started = None
        self.ascent_map_pauses = 0
        self.ascent_map_resumes = 0
        self.subgoal_diagnostics = {}

    def _same_altitude_exploration(self):
        return bool(self.c.get('tree_exploration_same_altitude') and self.tree_phase in ('APPROACH','SCAN'))

    def _camera_observation_ready(self,now,position):
        from .depth_health import fresh_depth
        hint = self._mission_hint(position)
        delta = np.asarray(hint)-position
        target = math.atan2(delta[1],delta[0]) if np.linalg.norm(delta[:2])>.05 else self.yaw
        key = (self.active_tree['id'],self.tree_phase,self.inspection_view)
        if (self.camera_gate.get('key')!=key or
                np.linalg.norm(np.asarray(position)-self.camera_gate.get('origin',position))>.15):
            self.camera_gate = dict(key=key,origin=np.asarray(position).tolist(),started=now,settled=None,ready=False)
        if self.camera_gate['ready']:
            if fresh_depth(now,self.depth_feedback.get('last_integrated_stamp'),self.c['depth_camera_timeout']):
                return True
            self.camera_gate.update(ready=False,started=now,settled=None)
        dt = 0. if self.mount_tick_stamp is None else max(0.,min(.1,now-self.mount_tick_stamp))
        self.mount_tick_stamp = now
        self.goal_yaw = target
        error = math.atan2(math.sin(target-self.yaw),math.cos(target-self.yaw))
        self.yaw += float(np.clip(error,-self.c['yaw_rate']*dt,self.c['yaw_rate']*dt))
        measured = self.mount_feedback.get('measured',{}).get('camera_pitch')
        pitch_target = self.mount_feedback.get('targets',{}).get('camera_pitch')
        actual_error = math.atan2(math.sin(target-self.body_yaw_measured),math.cos(target-self.body_yaw_measured))
        settled = (self.vehicle_speed<.04 and abs(actual_error)<.08 and self.mount_feedback.get('feedback_fresh')
            and measured is not None and pitch_target is not None and measured<-.02 and pitch_target<-.02
            and abs(measured-pitch_target)<.08)
        self.camera_gate['settled'] = (self.camera_gate['settled'] if self.camera_gate['settled'] is not None else now) if settled else None
        stamp = self.depth_feedback.get('last_integrated_stamp')
        self.camera_gate.update(depth_stamp=stamp,yaw_error=float(actual_error),pitch_measured=measured,pitch_target=pitch_target)
        if (self.camera_gate['settled'] is not None and now-self.camera_gate['settled']>=self.c.get('tree_camera_observe_dwell',.4)
                and stamp is not None and self.camera_gate['settled']<stamp<=now
                and now-stamp<self.c['depth_camera_timeout']):
            self.camera_gate.update(ready=True,reason='fresh_depth_after_camera_settled')
            self.blocked_since = None
            return True
        self.camera_gate['reason'] = 'waiting_for_downward_camera_and_fresh_depth'
        if now-self.camera_gate['started']>self.c.get('tree_mount_timeout',20.):
            self.fail('navigation_camera_observation_timeout')
        return False

    def _live_reference(self,now,dt,position,velocity):
        if not (self.c.get('tree_continuous_ascent') and self.state=='SURVEY' and self.tree_phase=='ASCEND'):
            return None
        from .ascent import ContinuousAscent
        if self.ascent_reference is None:
            height = float(np.clip(self.active_tree['observed_stem_top']-.4,self.c['takeoff_altitude'],5.5))
            self.ascent_reference = ContinuousAscent(height,self.c)
            self.ascent_nominal_acceleration = np.zeros(3)
            self.view_target = np.array([*self.command[:2],height])
        ref = self.ascent_reference
        map_fresh = self.map_age_safe(now,position,velocity,ascending=True)
        if not map_fresh and ref.stop is None:
            from .planning import Brake
            ref.stop = Brake(self.command,self.command_v,self.ascent_nominal_acceleration,ref.c)
            ref.reason = 'ascent_map_too_old'
            self.ascent_pause_started = now
            self.ascent_map_pauses += 1
            self.ascent_records.append(dict(action='map_pause',position=self.command.tolist(),
                started=now,scan_stamp=float(self.grid.last_observation_stamp)))
        if ref.finished and np.linalg.norm(velocity)<.04:
            if ref.reason=='ascent_map_too_old':
                waited = now-self.ascent_pause_started
                ref.diagnostics.update(mode='hold_wait_for_fresh_map',wait_seconds=waited,
                    map_age_seconds=now-self.grid.last_observation_stamp)
                if waited>self.c.get('tree_mount_timeout',20.):
                    self.fail('ascent_mapping_recovery_timeout')
                    return None
                if not map_fresh or self.grid.last_observation_stamp<=self.ascent_pause_started:
                    # Common health, actual clearance and final PID gates still run.
                    return self.command.copy(),np.zeros(3),np.zeros(3),self.elapsed+dt
                self.ascent_map_resumes += 1
                self.ascent_records.append(dict(action='map_resume',position=self.command.tolist(),
                    waited_seconds=waited,scan_stamp=float(self.grid.last_observation_stamp)))
                self.ascent_reference = ref = ContinuousAscent(ref.height,self.c)
                self.ascent_nominal_acceleration = np.zeros(3)
                self.ascent_pause_started = None
                # Re-run full swept/braking/observation-guard checks below.
            else:
                self.ascent_stop_reason = ref.reason
                self.ascent_records.append(dict(action='continuous_stop',position=self.command.tolist(),reason=ref.reason,
                    scan_stamp=float(self.grid.last_observation_stamp)))
                self.ascent_reference = None
                self.view_target = None
                self._mount_phase('NORMALIZE',now)
                return None
        try:
            p,v,a = ref.step(self.grid,self.command,self.command_v,self.ascent_nominal_acceleration,dt)
            self.ascent_nominal_acceleration = a.copy()
        except (RuntimeError,ValueError):
            self.fail('ascent_backup_invalidated')
            return None
        return p,v,a,self.elapsed+dt

    def inspection_hover(self):
        """Only a completed arrival, never an intermediate planning stop."""
        return bool(self.state=='SURVEY' and self.tree_phase=='SCAN'
            and self.wait_for_scan is not None and self.view_target is not None
            and np.linalg.norm(self.command-self.view_target)<.2 and self.vehicle_speed<.04)

    def camera_navigation_view(self):
        return self.tree_phase=='APPROACH' or (self.c.get('inspection_transit_camera',False)
            and self.tree_phase=='SCAN' and not self.inspection_hover())

    def auto_mounts(self):
        return bool(self.c.get('automatic_tilt_ascent'))

    def lidar_target(self):
        if self.auto_mounts() and self.tree_phase in ('UPWARD_OBSERVE','ASCEND'):
            angles = self.c.get('tree_tilt_angles',[.25,.7])
            return angles[min(self.mount_angle_index,len(angles)-1)]
        return 0.

    def mount_permission(self):
        return self.auto_mounts() and self.state=='SURVEY' and self.tree_phase in ('ORIENT','UPWARD_OBSERVE','ASCEND','NORMALIZE')

    def _mount_phase(self,phase,now):
        self.tree_phase = phase
        self.mount_phase_started = now
        self.mount_settled_since = None
        self.goal = self.trajectory = None
        self.elapsed = self.dwell = 0.
        self.blocked_since = None

    def _mount_hold(self,now):
        # The shared tick's health, actual clearance, braking and tracking
        # checks run before this method. No reference motion is generated here.
        if self.pending_plan is not None:
            if not self.pending_plan.done():
                return False
            self.pending_plan = None  # Discard work from the preceding phase.
        dt = 0. if self.mount_tick_stamp is None else max(0.,min(.1,now-self.mount_tick_stamp))
        self.mount_tick_stamp = now
        if now-(self.mount_phase_started or now)>self.c.get('tree_mount_timeout',20.):
            self.fail('mount_observation_timeout')
            return False
        yaw = math.atan2(self.active_tree['xy'][1]-self.command[1],self.active_tree['xy'][0]-self.command[0])
        if self.tree_phase=='ORIENT':
            error = math.atan2(math.sin(yaw-self.yaw),math.cos(yaw-self.yaw))
            self.yaw += float(np.clip(error,-self.c['yaw_rate']*dt,self.c['yaw_rate']*dt))
            actual_error = math.atan2(math.sin(yaw-self.body_yaw_measured),math.cos(yaw-self.body_yaw_measured))
            if abs(actual_error)<.08 and self.vehicle_speed<.04:
                self.mount_angle_index = 0
                self._mount_phase('UPWARD_OBSERVE',now)
            return False
        measured = self.mount_feedback.get('measured',{}).get('lidar_pitch',math.inf)
        settled = self.mount_feedback.get('feedback_fresh',False) and abs(measured-self.lidar_target())<.025 and self.vehicle_speed<.04
        self.mount_settled_since = (self.mount_settled_since if self.mount_settled_since is not None else now) if settled else None
        if self.mount_settled_since is None or now-self.mount_settled_since<self.c.get('tree_tilt_dwell',.8):
            return False
        if self.grid.last_observation_stamp<=self.mount_settled_since:
            return False  # Must integrate a measurement after reaching the angle.
        if self.tree_phase=='NORMALIZE':
            if not self.mount_feedback.get('navigation_ready'):
                return False
            self.inspection_altitude = float(self.command[2])
            self.view_target = None
            self._mount_phase('SCAN',now)
        else:
            self.ascent_records.append(dict(action='observe',position=self.command.tolist(),
                pitch=float(measured),scan_stamp=float(self.grid.last_observation_stamp)))
            self.mount_angle_index += 1
            if self.mount_angle_index>=len(self.c.get('tree_tilt_angles',[.25,.7])):
                self.mount_angle_index -= 1
                self.view_target = None
                self._mount_phase('ASCEND',now)
            else:
                self.mount_settled_since = None
                self.mount_phase_started = now
        return False

    def receive_trees(self,data):
        if data.get('frame') != 'map' or data.get('source') != 'measured_lidar_geometry':
            return
        if data['stamp'] <= self.catalog_stamp:
            return
        self.catalog_stamp = data['stamp']
        self.perception_statistics = {k:data.get(k) for k in (
            'wall_ms','retained_points','stem_points','reason','ground_reference_m',
            'ground_reference_stamp','recent_ground_percentile_m','ground_policy',
            'stem_height_band_m','detections_count')}
        self.catalog = [t for t in data['tracks'] if t.get('confirmed')]

    def candidates(self):
        return [t for t in self.catalog if t['id'] not in self.completed_trees
                and self.survey_now-t['last_seen'] < 30.
                and not any(np.linalg.norm(np.asarray(t['xy'])-xy) < 1.
                            for xy in self.completed_centres)]

    def preflight_hint(self):
        choices = self.candidates()
        if not choices:
            return None
        tree = min(choices,key=lambda t:np.linalg.norm(np.asarray(t['xy'])-self.command[:2]))
        delta = self.command[:2]-tree['xy']
        xy = np.asarray(tree['xy'])+delta/max(np.linalg.norm(delta),1e-8)*max(self.c['tree_view_distances'])
        return [*xy,self.c['takeoff_altitude']]

    def _mission_hint(self,current):
        if self.state == 'SURVEY' and self.view_target is not None:
            if self.c.get('history_route_enabled') and self.tree_phase=='APPROACH':
                goal=self._history_goal(current,self.view_target)
                return np.asarray(current).copy() if goal is None else goal
            from .local_subgoal import inspection_hint, geometric_hint
            if self.executor is not None:
                # Camera preparation needs a direction, not a costly detour search.
                # This hint never authorizes flight; the detached planner certifies it.
                return geometric_hint(current,self.view_target)
            hint,self.subgoal_diagnostics = inspection_hint(self.grid,current,self.view_target,
                allow_vertical=self.tree_phase=='APPROACH')
            return hint
        return super()._mission_hint(current)

    def route_destination(self):
        if self.state=='SURVEY' and self.tree_phase=='APPROACH' and self.view_target is not None:
            return self.view_target.copy()
        return super().route_destination()

    def _planning_clone(self):
        clone = super()._planning_clone()
        if clone.state=='SURVEY' and clone.view_target is not None:
            # Do not reuse an unexamined live-camera hint as a planner goal.
            clone.goal = None
        return clone

    def _choose_goal(self,current):
        if self.state != 'SURVEY':
            return super()._choose_goal(current)
        if self.active_tree is None:
            limit = self.c.get('tree_demo_limit',0)
            if limit and len(self.completed_trees) >= limit:
                self.finish_reason = 'configured_demo_limit_not_entire_farm'
                self.state = 'RETURN'
                return super()._choose_goal(current)
            choices = self.candidates()
            if not choices:
                self.search_started = self.survey_now if self.search_started is None else self.search_started
                if self.survey_now-self.search_started >= self.c.get('tree_search_wait',10.):
                    self.finish_reason = 'no_unvisited_confirmed_visible_tree_not_entire_farm'
                    self.state = 'RETURN'
                    return super()._choose_goal(current)
                self.goal = np.asarray(current).copy()
                return False
            self.search_started = None
            self.active_tree = min(choices,key=lambda t:np.linalg.norm(np.asarray(t['xy'])-np.asarray(current)[:2])).copy()
            delta = np.asarray(current)[:2]-self.active_tree['xy']
            self.tree_angle = math.atan2(delta[1],delta[0])
            self.tree_phase = 'APPROACH'
            self.inspection_altitude = None
            self.ascent_stop_reason = ''
            self.inspection_view = 0
        tree = self.active_tree
        if self.view_target is not None and self.grid.occupied_goal(self.view_target):
            self.rejected_views.append(dict(tree_id=tree['id'],view=int(self.inspection_view),
                position=self.view_target.tolist(),stamp=float(self.grid.last_observation_stamp),
                reason='newly_observed_occupied_view'))
            self.rejected_views = self.rejected_views[-32:]
            self.view_target = None
        # Re-evaluate an unobserved view at rest after exploration: a newly
        # measured safe alternative can avoid chasing the original blind spot.
        if self.view_target is None or not self.grid.safe(self.view_target):
            angle = self.tree_angle+(self.inspection_view*2*math.pi/self.c['tree_view_count'] if self.tree_phase=='SCAN' else 0.)
            top = float(np.clip(tree['observed_stem_top']-.4,2.,5.5))
            if (self.auto_mounts() or self.c.get('continuous_tree_scanning')) and self.tree_phase=='APPROACH':
                top = self.c.get('tree_approach_altitude',self.c['takeoff_altitude'])
            elif self.auto_mounts() and self.inspection_altitude is not None:
                top = self.inspection_altitude
            heights = [top] if self.auto_mounts() else [top,max(2.,top-.5),self.c['takeoff_altitude']]
            views = []
            for height in heights:
                for radius in self.c['tree_view_distances']:
                    p = np.array([tree['xy'][0]+radius*math.cos(angle),tree['xy'][1]+radius*math.sin(angle),height])
                    p = self.grid.centers(self.grid.indices(p))
                    if self.grid.safe(p):
                        score = abs(height-top)+.15*radius+.05*np.linalg.norm(p-current)
                        views.append((score,p))
            if views:
                self.view_target = min(views,key=lambda item:item[0])[1]
            else:
                # Request observations toward a conservative stand-off view;
                # no unknown voxel is declared free to make the view reachable.
                radius = max(self.c['tree_view_distances'])
                self.view_target = np.array([tree['xy'][0]+radius*math.cos(angle),
                                             tree['xy'][1]+radius*math.sin(angle),top])
        self.goal = self._mission_hint(current)
        self.waypoints = np.asarray([self.goal.copy()])
        self.waypoint_index = 0
        delta = np.asarray(tree['xy'])-np.asarray(current)[:2]
        if self.c.get('scan_body_faces_path'):
            from .sensor_mounts import path_heading
            self.goal_yaw = path_heading(self.goal-np.asarray(current),self.yaw)
        else:
            self.goal_yaw = math.atan2(delta[1],delta[0])
        self.desired_pitch = 0.
        return self.grid.safe(self.goal)

    def _prepare_trajectory(self,now,position,range_value=None,range_stamp=None):
        if (self.c.get('history_route_enabled') and self.state=='SURVEY'
                and self.tree_phase=='SEARCH' and self.active_tree is None):
            # Select the final measured-tree approach goal BEFORE requesting a
            # history route. This does not authorize a geometric local flight.
            self._choose_goal(position)
        if getattr(self,'depth_navigation_paused',False):
            return False  # Keep the certified hold; do not accept pending plans.
        if self.auto_mounts() and self.state=='SURVEY' and self.tree_phase in ('ORIENT','UPWARD_OBSERVE','NORMALIZE'):
            return self._mount_hold(now)
        if self.wait_for_scan is not None:
            at_scan_view = (self.tree_phase=='SCAN' and self.view_target is not None
                and np.linalg.norm(self.command-self.view_target)<.2)
            if self.c.get('inspection_transit_camera') and at_scan_view and not self.inspection_hover():
                self.view_settled_since = None
                return False  # Arriving slowly is not yet a stationary inspection.
            if self.c.get('inspection_transit_camera') and self.inspection_hover():
                if now-self.wait_for_scan>self.c.get('tree_mount_timeout',20.):
                    self.fail('inspection_view_orientation_timeout')
                    return False
                target = math.atan2(self.active_tree['xy'][1]-position[1],self.active_tree['xy'][0]-position[0])
                dt = 0. if self.mount_tick_stamp is None else max(0.,min(.1,now-self.mount_tick_stamp))
                self.mount_tick_stamp = now
                error = math.atan2(math.sin(target-self.yaw),math.cos(target-self.yaw))
                self.goal_yaw = target
                self.yaw += float(np.clip(error,-self.c['yaw_rate']*dt,self.c['yaw_rate']*dt))
                actual_error = math.atan2(math.sin(target-self.body_yaw_measured),math.cos(target-self.body_yaw_measured))
                measured = self.mount_feedback.get('measured',{})
                targets = self.mount_feedback.get('targets',{})
                joints = ('camera_pitch','camera_yaw') if self.c.get('camera_yaw_enabled',True) else ('camera_pitch',)
                settled = (abs(actual_error)<.08 and self.mount_feedback.get('feedback_fresh',False)
                    and all(j in measured and j in targets and abs(measured[j]-targets[j])<.08 for j in joints)
                    and measured.get('camera_pitch',0.)>.02)
                self.view_settled_since = (self.view_settled_since if self.view_settled_since is not None else now) if settled else None
                if (self.view_settled_since is None or now-self.view_settled_since<self.c.get('tree_tilt_dwell',.8)
                        or self.grid.last_observation_stamp<=self.view_settled_since):
                    return False
            if self.grid.last_observation_stamp <= self.wait_for_scan:
                return False
            if self.auto_mounts() and self.tree_phase=='SCAN':
                measured = self.mount_feedback.get('measured',{})
                targets = self.mount_feedback.get('targets',{})
                if not self.mount_feedback.get('feedback_fresh') or any(
                        abs(measured.get(j,math.inf)-targets.get(j,0.))>.08
                        for j in (('camera_pitch','camera_yaw') if self.c.get('camera_yaw_enabled',True) else ('camera_pitch',))):
                    return False  # Do not count a view while its camera is moving.
            self.wait_for_scan = None
            self.view_settled_since = None
            reached = self.view_target is not None and np.linalg.norm(self.command-self.view_target) < .2
            if reached and self.tree_phase == 'APPROACH':
                if self.auto_mounts():
                    self._mount_phase('ORIENT',now)
                else:
                    self.tree_phase = 'SCAN'
                self.view_target = None
            elif reached and self.tree_phase=='ASCEND':
                self.ascent_records.append(dict(action='ascend',position=self.command.tolist(),
                    scan_stamp=float(self.grid.last_observation_stamp)))
                self.mount_angle_index = 0
                self.view_target = None
                self._mount_phase('UPWARD_OBSERVE',now)
            elif reached and self.tree_phase == 'SCAN':
                self.scan_records.append(dict(tree_id=self.active_tree['id'],view=int(self.inspection_view),
                    position=self.command.tolist(),scan_stamp=float(self.grid.last_observation_stamp),
                    body_yaw_measured=float(self.body_yaw_measured),
                    camera_pitch_measured=self.mount_feedback.get('measured',{}).get('camera_pitch'),
                    camera_policy='tree_up' if self.c.get('inspection_transit_camera') else 'legacy'))
                self.inspection_view += 1
                self.view_target = None
                if self.inspection_view >= self.c['tree_view_count']:
                    self.completed_trees.append(self.active_tree['id'])
                    self.completed_centres.append(list(self.active_tree['xy']))
                    self.active_tree = None
                    self.tree_phase = 'SEARCH'
            MissionController._advance(self)
            self.trajectory = None
            self.elapsed = 0.
            self.batch_end_index = self.pass_arrival = None
            self.waypoint_index = 0
            return False
        if (self.c.get('tree_camera_observe_before_plan') and self.state=='SURVEY'
                and self.tree_phase in ('SEARCH','APPROACH','SCAN')):
            if self.view_target is None:
                self._choose_goal(position)
            if self.active_tree is not None and not self._camera_observation_ready(now,position):
                return False
        if self.auto_mounts() and self.state=='SURVEY' and self.tree_phase=='ASCEND' and self.goal is None:
            desired = float(np.clip(self.active_tree['observed_stem_top']-.4,self.c['takeoff_altitude'],5.5))
            step = self.command.copy()
            step[2] = min(desired,step[2]+self.c.get('tree_ascent_step',.4))
            blockers = self.grid.corridor_blockers(self.command,step)
            safe = self.grid.line_safe(self.command,step)
            self.ascent_records.append(dict(action='check_next_step',position=self.command.tolist(),
                target=step.tolist(),safe=bool(safe),unknown_count=len(blockers['unknown']),
                occupied_count=len(blockers['occupied']),scan_stamp=float(self.grid.last_observation_stamp)))
            if step[2]-self.command[2]<.05 or not safe:
                self.ascent_stop_reason = 'desired_observed_height_reached' if safe else 'next_step_not_certified_free'
                self.view_target = None
                self._mount_phase('NORMALIZE',now)
                return False
            self.goal = self.view_target = step
            self.waypoints = np.asarray([step.copy()])
            self.waypoint_index = 0
        return super()._prepare_trajectory(now,position,range_value,range_stamp)

    def _guarded_exploration(self,now,current):
        if self.auto_mounts() and self.tree_phase=='ASCEND':
            return None  # No moving next-best-view search with tilted LiDAR.
        return super()._guarded_exploration(now,current)

    def _on_motion(self,now,position):
        if self.c.get('scan_body_faces_path') and not (self.c.get('inspection_transit_camera') and self.inspection_hover()):
            from .sensor_mounts import path_heading
            self.goal_yaw = path_heading(self.command_v,self.yaw)
        elif self.active_tree is not None:
            delta = np.asarray(self.active_tree['xy'])-np.asarray(position)[:2]
            target = math.atan2(delta[1],delta[0])
            self.goal_yaw = target  # The existing bounded yaw adapter applies this.

    def inspection_status(self):
        return dict(mission_type='sensor_detected_tree_inspection', tree_phase=self.tree_phase,
            body_heading_policy='path_in_transit_tree_at_view' if self.c.get('inspection_transit_camera') else 'path_tangent_hold_when_stationary' if self.c.get('scan_body_faces_path') else 'tree_facing',
            inspection_camera_policy='path_down' if self.camera_navigation_view() else 'tree_up',
            inspection_view_count=self.c['tree_view_count'],camera_yaw_enabled=self.c.get('camera_yaw_enabled',True),
            viewpoint_settled_since=self.view_settled_since,
            navigation_camera_gate=self.camera_gate,
            exploration_altitude_policy='same_altitude_only' if self._same_altitude_exploration() else 'legacy_3D',
            continuous_ascent=self.ascent_reference.diagnostics if self.ascent_reference is not None else {},
            ascent_map_pauses=self.ascent_map_pauses,ascent_map_resumes=self.ascent_map_resumes,
            inspection_subgoal_diagnostics=self.subgoal_diagnostics,
            perception_statistics=self.perception_statistics,
            tree_candidates=self.catalog, tree_catalog_stamp=self.catalog_stamp if math.isfinite(self.catalog_stamp) else None,
            inspection_tree_id=self.active_tree['id'] if self.active_tree else None,
            inspection_view=self.inspection_view, inspected_tree_ids=self.completed_trees,
            inspected_tree_centres=self.completed_centres,
            tree_scan_records=self.scan_records, tree_finish_reason=self.finish_reason,
            rejected_inspection_views=self.rejected_views,
            lidar_tilt_target=float(self.lidar_target()),lidar_tilt_fsm_authorized=bool(self.mount_permission()),
            lidar_tilt_change_authorized=bool(self.mount_permission() and self.tree_phase!='ASCEND' and self.vehicle_speed<.04),
            inspection_altitude=self.inspection_altitude,ascent_stop_reason=self.ascent_stop_reason,
            ascent_records=self.ascent_records[-64:],
            entire_farm_inspected=False, inspection_goal_hint=self.preflight_hint() if self.state=='TAKEOFF'
            else self.view_target.tolist() if self.view_target is not None else None)
