"""Scheduled C2 escape from a depth-recovery brake, with live measured-state gates.

The reference boundary is retained to avoid a setpoint jump. Measured P/V
certifies tracking and a separate actual stopping corridor; it is never
silently substituted for the reference or assumed stationary.
"""
import copy
import numpy as np
from .time_checks import fresh_stamp


def cancel(controller, reason):
    controller.depth_recovery_key=None
    future=getattr(controller,'depth_recovery_future',None)
    if future is not None and (future.done() or future.cancel()):
        controller.depth_recovery_future=None
    controller.depth_recovery_status=dict(reason=reason)


def request(controller, now, depth, timeout, episode):
    controller.depth_recovery_observation=(now,dict(depth),timeout,episode)
    if not controller.c.get('depth_moving_recovery') or controller.executor is None:return
    future=controller.depth_recovery_future
    if future is not None:
        if controller.depth_recovery_key is not None or not future.done():return
        controller.depth_recovery_future=None
    if (controller.failure!='depth_recovering' or controller.resume_state not in ('SURVEY','RETURN')
            or getattr(controller,'tree_phase',None)=='ASCEND' or controller.wait_for_scan is not None):return
    integrated=depth.get('last_integrated_stamp')
    if integrated is None or integrated<=episode or not fresh_stamp(now,integrated,timeout*.6):return
    # Predict a future point of the brake. If planning is late, KEEP braking.
    # At its stationary endpoint the same PVA can be adopted without stopping
    # the measured vehicle first, provided live tracking/stopping checks pass.
    for name in ('pending_plan','prefetched_plan','moving_future'):
        other=getattr(controller,name,None)
        if other is not None and not other.done():
            other.cancel()
            if not other.done():return
    brake=controller.brake
    lead=controller.c.get('depth_recovery_planning_lead',.5)
    predicted=copy.deepcopy(brake) if brake is not None else None
    if predicted is not None:
        predicted.step(min(lead,max(0.,predicted.duration-predicted.elapsed)))
        p,v,a=predicted.p.copy(),predicted.v.copy(),predicted.a.copy()
        elapsed=predicted.elapsed
    else:
        p,v,a=controller.command.copy(),controller.command_v.copy(),controller.command_a.copy()
        elapsed=None
    clone=controller._planning_clone()
    clone.command,clone.command_v,clone.command_a=p,v,a
    clone.moving_boundary=(v.copy(),a.copy())
    clone.state=controller.resume_state;clone.failure='';clone.brake=None
    clone.depth_navigation_paused=False
    clone.trajectory=None;clone.elapsed=clone.dwell=0.
    from .survey import prepare_survey_snapshot
    controller.depth_recovery_future=controller.executor.submit(prepare_survey_snapshot,clone,now,p.copy(),None,None)
    controller.depth_recovery_key=dict(brake=brake,elapsed=elapsed,submitted=now,episode=episode,
        state=clone.state,tree_phase=getattr(clone,'tree_phase',None),index=clone.waypoint_index)
    controller.depth_recovery_status=dict(reason='planning_C2_from_brake',submitted_sim=now,
        scheduled_brake_elapsed=elapsed,reference_speed_mps=float(np.linalg.norm(v)))


def adopt(controller, now, position, velocity):
    key=controller.depth_recovery_key;future=controller.depth_recovery_future
    if key is None or future is None:return False
    if not future.done():
        cancel(controller,'planner_not_ready_at_boundary');return False
    try:clone,ready=future.result()
    except Exception as exc:
        cancel(controller,'planner_exception:'+type(exc).__name__);return False
    controller.depth_recovery_future=None;controller.depth_recovery_key=None
    observation=getattr(controller,'depth_recovery_observation',None)
    reason=None;detail={}
    if controller.failure!='depth_recovering':reason='superseded_by_failure'
    elif now-key['submitted']>controller.c.get('moving_result_max_age',1.):reason='stale_result'
    elif observation is None or observation[3]!=key['episode']:reason='episode_changed'
    elif (controller.resume_state!=key['state'] or getattr(controller,'tree_phase',None)!=key['tree_phase']
            or controller.waypoint_index!=key['index']):reason='phase_or_anchor_changed'
    elif not ready or clone.failure or clone.trajectory is None:reason='no_certified_candidate'
    elif (clone.state!=key['state'] or getattr(clone,'tree_phase',None)!=key['tree_phase']
            or clone.waypoint_index!=key['index']):reason='candidate_phase_or_anchor_changed'
    else:
        _,depth,timeout,episode=observation
        integrated=depth.get('last_integrated_stamp')
        if (not fresh_stamp(now,depth.get('last_valid_stamp'),timeout)
                or integrated is None or integrated<=episode or not fresh_stamp(now,integrated,timeout*.6)):
            reason='depth_not_fresh_after_brake'
        elif not fresh_stamp(now,controller.grid.last_observation_stamp,controller.c['map_timeout']):reason='map_stale'
        elif controller.health.reason(now,controller.c,controller.sensor_mode):reason='sensor_health'
        elif not np.all(np.isfinite(np.r_[position,velocity])):reason='invalid_measured_state'
        elif np.linalg.norm(position-controller.command)>controller.c['tracking_limit']:reason='tracking_error'
        elif np.linalg.norm(velocity)>controller.c['speed']+.05:reason='measured_overspeed'
        else:
            from .planning import brake_safe
            from .super_policy import backup_grid,feedback_brake_safe
            live=copy.copy(clone);live.grid=controller.grid
            frozen_grid=backup_grid(clone);live_grid=backup_grid(live)
            initial=clone.trajectory.sample(0.)
            jumps=[float(np.linalg.norm(a-b)) for a,b in zip(
                (controller.command,controller.command_v,controller.command_a),initial)]
            detail['PVA_jump']=jumps
            if max(jumps)>1e-6:reason='PVA_boundary_mismatch'
            elif not controller.grid.safe(position,actual=True):reason='actual_body_blocked'
            elif not brake_safe(controller.grid,position,velocity,controller.c,controller.command_a,actual=True):reason='actual_stop_blocked'
            elif (frozen_grid.blocked.shape!=live_grid.blocked.shape or frozen_grid.res!=live_grid.res
                    or not np.array_equal(frozen_grid.lo,live_grid.lo)):
                reason='map_window_changed'
            elif np.any(live_grid.blocked & ~frozen_grid.blocked):
                # Conservatively retain the worker's FULL braking certificate:
                # every previously admissible voxel must remain admissible.
                # New free voxels are harmless; new blockers anywhere reject.
                reason='new_map_invalidated_braking_certificate'
            elif not clone.trajectory.safe(live_grid):reason='new_map_invalidated_curve'
            elif not brake_safe(live_grid,*initial[:2],controller.c,initial[2]):reason='reference_stop_blocked'
            elif controller.c.get('feedback_speed_envelope') and not feedback_brake_safe(live_grid,*initial,controller.c):
                reason='feedback_stop_blocked'
    if reason:
        controller.depth_recovery_status=dict(reason=reason,**detail);return False
    # Worker certified full braking; live curve and both initial stopping
    # corridors are checked above. Common per-tick checks remain active.
    controller.failure='';controller.state=key['state'];controller.brake=None
    for name in ('trajectory','goal','goal_yaw','desired_pitch','batch_end_index','batch_knots','phase'):
        setattr(controller,name,getattr(clone,name))
    controller.elapsed=controller.dwell=0.;controller.checked_map_version=-1
    controller.pending_plan=controller.prefetched_plan=controller.moving_future=None
    controller.prefetch_key=controller.moving_key=None
    controller.paths.append(controller.trajectory.points.copy());controller.replans+=1
    controller.route_speed_profile=getattr(clone,'route_speed_profile',{})
    controller._accept_plan_auxiliary(clone,True)
    controller.depth_recovery_accepted_episode=key['episode']
    controller.depth_recovery_status=dict(reason='accepted_C2_measured_stop_checked',simulation_time=now,
        measured_position_m=np.asarray(position).tolist(),measured_velocity_mps=np.asarray(velocity).tolist(),
        actual_speed_mps=float(np.linalg.norm(velocity)),reference_speed_mps=float(np.linalg.norm(controller.command_v)),
        **detail)
    return True
