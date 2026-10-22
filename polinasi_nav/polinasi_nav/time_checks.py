"""Inclusive measurement-age checks with only nanosecond roundoff tolerance."""
import math


def fresh_stamp(now,stamp,timeout):
    return (stamp is not None and math.isfinite(now) and math.isfinite(stamp)
            and math.isfinite(timeout) and -1e-9<=now-stamp<=timeout+1e-9)
