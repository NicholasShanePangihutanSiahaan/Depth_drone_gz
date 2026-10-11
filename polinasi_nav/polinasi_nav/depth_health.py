"""A stale integrated camera map never authorizes motion into unknown space."""
import math


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
