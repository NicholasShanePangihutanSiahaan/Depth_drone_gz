"""Bounded future-reference handoff. No ROS, publishers, or health bypasses."""
import math
import numpy as np
from .continuous import ContinuousTrajectory, AdaptivePhase


class MovingHandoff:
    def _moving_init(self):
        self.moving_future=None;self.moving_key=None
        self.moving_last_submit=-math.inf
        self.moving_status=dict(submitted=0,accepted=0,rejected=0,reason='idle')

    def _moving_stable(self):
        return (isinstance(self.trajectory,ContinuousTrajectory) and abs(self.phase.rate-1.)<1e-9
                and abs(self.phase.rate_dot)<1e-9 and abs(self.phase.rate_ddot)<1e-9)

    def _moving_index_at(self,elapsed):
        index=self.waypoint_index
        if self.state!='SURVEY' or self.batch_end_index is None:return index
        # Predict only a sequential prefix of anchors. Adoption requires that
        # the live measured-pass/new-scan logic has advanced to EXACTLY this index.
        for t in np.arange(self.elapsed,elapsed+1e-8,.025):
            if index>=self.batch_end_index:break
            if np.linalg.norm(self.trajectory.sample(t)[0]-self.waypoints[index])<=self.c.get('survey_pass_radius',.2):
                index+=1
        return index

    def _moving_reject(self,reason):
        self.moving_status.update(rejected=self.moving_status['rejected']+1,reason=reason)
        self.moving_key=None
        if self.moving_future is not None and (self.moving_future.done() or self.moving_future.cancel()):
            self.moving_future=None

    def _moving_crossing(self,next_elapsed,now):
        """Switch at the exact reference boundary inside a variable-duration tick.

        Both pieces have matching PVA. The common control path still checks
        the sampled command, its connecting line and its stopping corridor.
        """
        key=self.moving_key
        if key is None or not self.elapsed<key['elapsed']<=next_elapsed:return next_elapsed
        previous=self.elapsed
        remainder=next_elapsed-key['elapsed']
        self.elapsed=key['elapsed']
        self._moving_update(now)
        if self.trajectory is not key['trajectory']:
            return min(remainder,self.trajectory.duration)
        self.elapsed=previous
        return next_elapsed

    def _moving_update(self,now):
        if not self.c.get('moving_handoff_enabled'):return
        key=self.moving_key
        if self.moving_future is not None:
            if key is None:
                if self.moving_future.done():self.moving_future=None
                return
            valid=(not self.failure and self.state==key['state']
                and getattr(self,'tree_phase',None)==key['tree_phase']
                and self.trajectory is key['trajectory'] and self._moving_stable())
            if not valid:
                self._moving_reject('reference_or_phase_changed');return
            if self.elapsed>key['elapsed']+1e-7:
                self._moving_reject('handoff_deadline_missed');return
            if self.elapsed<key['elapsed']-1e-7:return
            if not self.moving_future.done():
                self._moving_reject('planner_not_ready_at_boundary');return
            try:
                clone,ready=self.moving_future.result()
            except Exception:
                self._moving_reject('planner_exception');return
            self.moving_future=None
            reason=None
            if now-key['submitted']>self.c.get('moving_result_max_age',1.5)+1e-8:reason='stale_result'
            elif not ready or clone.failure or clone.trajectory is None:reason='no_certified_candidate'
            elif clone.state!=self.state or getattr(clone,'tree_phase',None)!=key['tree_phase']:reason='candidate_phase_changed'
            elif self.waypoint_index!=key['index']:reason='measured_anchor_progress_mismatch'
            elif np.linalg.norm(clone.goal-self.goal)<self.grid.res/2:reason='no_endpoint_extension'
            else:
                actual=self.trajectory.sample(self.elapsed);initial=clone.trajectory.sample(0.)
                if any(np.linalg.norm(a-b)>1e-6 for a,b in zip(actual,initial)):reason='PVA_boundary_mismatch'
                elif not clone.trajectory.safe(self.grid):reason='new_map_invalidated_curve'
                elif self.c.get('depth_camera_enabled') and getattr(self,'tree_phase',None)=='APPROACH':
                    stamp=self.depth_feedback.get('last_integrated_stamp')
                    yaw=math.atan2(math.sin(clone.goal_yaw-self.goal_yaw),math.cos(clone.goal_yaw-self.goal_yaw))
                    if (stamp is None or not 0<=now-stamp<self.c['depth_camera_timeout']
                            or not self.mount_feedback.get('feedback_fresh') or abs(yaw)>.1):reason='camera_observation_required'
            if reason:
                self._moving_reject(reason);return
            before=self.trajectory.sample(self.elapsed)
            self.trajectory=clone.trajectory;self.goal=clone.goal.copy();self.elapsed=0.
            self.goal_yaw=clone.goal_yaw;self.batch_end_index=clone.batch_end_index;self.batch_knots=clone.batch_knots
            self.phase=AdaptivePhase(self.c);self.checked_map_version=-1
            self.paths.append(self.trajectory.points.copy());self.replans+=1
            self._accept_plan_auxiliary(clone,True)
            self.moving_status.update(accepted=self.moving_status['accepted']+1,reason='accepted_C2_live_map_checked',
                simulation_time=float(now),position=before[0].tolist(),speed_mps=float(np.linalg.norm(before[1])),
                PVA_jump=[float(np.linalg.norm(a-b)) for a,b in zip(before,self.trajectory.sample(0.))])
            self.moving_key=None
            return
        if (self.executor is None or self.pending_plan is not None or self.failure
                or self.state not in ('SURVEY','RETURN') or not self._moving_stable()
                or getattr(self,'tree_phase',None)=='ASCEND' or self.wait_for_scan is not None):return
        # Do not replace a trajectory that already reaches the actual phase goal.
        destination=self.route_destination()
        if self.state=='SURVEY' and getattr(self,'tree_phase',None)=='SCAN':
            if self.batch_end_index==len(self.waypoints)-1:return
        elif destination is None or np.linalg.norm(self.goal-destination)<.12:return
        if now-self.moving_last_submit<self.c.get('moving_replan_interval',1.):return
        if self.elapsed<max(.5,self.trajectory.duration*.35):return
        step=.05
        lead=math.ceil(self.c.get('moving_handoff_lead',.75)/step)*step
        elapsed=self.elapsed+lead
        if elapsed>=self.trajectory.duration-.3:return
        p,v,a=self.trajectory.sample(elapsed)
        if np.linalg.norm(v)<.04:return
        index=self._moving_index_at(elapsed)
        clone=self._planning_clone()
        clone.command=p.copy();clone.command_v=v.copy();clone.command_a=a.copy()
        clone.moving_boundary=(v.copy(),a.copy())
        clone.trajectory=clone.goal=None;clone.elapsed=clone.dwell=0.;clone.wait_for_scan=None
        clone.waypoint_index=index
        from .survey import prepare_survey_snapshot
        self.moving_future=self.executor.submit(prepare_survey_snapshot,clone,now,p.copy(),None,None)
        self.moving_key=dict(trajectory=self.trajectory,state=self.state,tree_phase=getattr(self,'tree_phase',None),
            index=index,elapsed=elapsed,submitted=now)
        self.moving_last_submit=now
        self.moving_status.update(submitted=self.moving_status['submitted']+1,reason='planning_future_PVA',
                                  scheduled_elapsed=float(elapsed))
