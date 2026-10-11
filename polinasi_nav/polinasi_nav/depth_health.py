"""A stale integrated camera map never authorizes motion into unknown space."""
import math
import numpy as np


def fresh_depth(now,stamp,timeout):
    return stamp is not None and math.isfinite(stamp) and -1e-9<=now-stamp<=timeout+1e-9


class DepthHoldGuard:
    def __init__(self):
        self.wait_since = None
        self.status = {}

    def assess(self,now,depth,stationary_safe,timeout,hold_timeout=2.):
        integrated = depth.get('last_integrated_stamp')
        valid = depth.get('last_valid_stamp')
        if fresh_depth(now,integrated,timeout):
            self.wait_since = None
            action = 'fresh'
        else:
            self.wait_since = now if self.wait_since is None else self.wait_since
            action = ('hold_wait_for_integration' if stationary_safe
                and fresh_depth(now,valid,timeout) and now-self.wait_since<=hold_timeout else 'fail')
        self.status = dict(action=action,stationary_safe=bool(stationary_safe),
            integrated_age=None if integrated is None else now-integrated,
            valid_age=None if valid is None else now-valid,wait_since=self.wait_since,
            timeout_seconds=timeout,hold_timeout_seconds=hold_timeout)
        return action


class DepthMotionRecovery:
    """Brake before the depth deadline, then resume only from a fresh settled map."""
    def __init__(self):
        self.started = None
        self.count = 0
        self.status = {}

    def update(self, now, depth, controller, position, velocity, timeout, hold_timeout=2.):
        from .planning import brake_safe
        integrated = depth.get('last_integrated_stamp')
        valid = depth.get('last_valid_stamp')
        age = math.inf if integrated is None else now-integrated
        threshold = timeout*.6
        action = 'inactive'
        if self.started is not None:
            if controller.failure != 'depth_recovering':
                action = 'superseded_by_failure'
            elif not fresh_depth(now, valid, timeout) or now-self.started > hold_timeout:
                controller.failure = 'stale_depth_camera'
                action = 'fail'
            elif (controller.state == 'HOLD_ABORT' and controller.brake is None
                    and np.linalg.norm(velocity)<.04 and np.linalg.norm(controller.command_v)<.01
                    and integrated is not None and integrated>self.started
                    and fresh_depth(now,integrated,threshold)
                    and controller.grid.safe(position,actual=True)
                    and controller.grid.safe(controller.command)):
                controller.failure = ''
                controller.state = controller.resume_state
                controller.goal = controller.trajectory = None
                controller.pending_plan = None
                controller.elapsed = controller.dwell = 0.
                self.started = None
                action = 'resumed_replan'
            else:
                action = 'braking_or_waiting'
        elif (not controller.failure and controller.state=='SURVEY' and age>threshold
                and (controller.trajectory is not None or np.linalg.norm(controller.command_v)>.01)
                and fresh_depth(now,valid,timeout)):
            if (brake_safe(controller.grid,position,velocity,controller.c,controller.command_a,actual=True)
                    and brake_safe(controller.grid,controller.command,controller.command_v,
                        controller.c,controller.command_a)):
                self.started = now
                self.count += 1
                controller.fail('depth_recovering')
                action = 'certified_brake_started'
            else:
                controller.fail('insufficient_depth_recovery_stopping_clearance')
                action = 'fail'
        self.status = dict(action=action,started=self.started,recoveries=self.count,
            integrated_age=None if integrated is None else age,
            valid_age=None if valid is None else now-valid,
            preventive_threshold_seconds=threshold,hold_timeout_seconds=hold_timeout)
        return action
