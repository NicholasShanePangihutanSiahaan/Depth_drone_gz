"""Inclusive measurement-age checks with only nanosecond roundoff tolerance."""
import math


def fresh_stamp(now,stamp,timeout):
    return (stamp is not None and math.isfinite(now) and math.isfinite(stamp)
            and math.isfinite(timeout) and -1e-9<=now-stamp<=timeout+1e-9)


class CausalStampQueue:
    """Select only received measurement times already reached by a tick's clock.

    Does not clamp timestamps, extrapolate the clock, or refresh an old sample.
    The caller still applies the original freshness timeout. Single-callback
    owner; a backwards control clock remains a separate terminal flight gate.
    """
    def __init__(self,capacity=500):
        from collections import deque
        self.pending=deque(maxlen=capacity)
        self.selected=-math.inf;self.latest_received=-math.inf
        self.overflows=self.obsolete_drops=self.invalid_drops=self.future_wait_ticks=0
        self.future_pending=0

    def push(self,stamp):
        if stamp is None or not math.isfinite(stamp):
            self.invalid_drops+=1;return False
        self.latest_received=float(stamp)
        if stamp<=self.selected:
            self.obsolete_drops+=1;return False
        if len(self.pending)==self.pending.maxlen:self.overflows+=1
        self.pending.append(float(stamp));return True

    def select(self,now):
        if not math.isfinite(now):return -math.inf
        eligible=[t for t in self.pending if t<=now]
        if eligible:self.selected=max(self.selected,max(eligible))
        # Retain future packets with their ORIGINAL acquisition timestamps.
        future=[t for t in self.pending if t>now]
        self.pending.clear();self.pending.extend(future)
        self.future_pending=len(future)
        if future:self.future_wait_ticks+=1
        return self.selected if self.selected<=now else -math.inf

    def status(self):
        def finite(x):return float(x) if math.isfinite(x) else None
        return dict(policy='latest_received_stamp_not_after_control_tick_clock',
            capacity=self.pending.maxlen,pending=len(self.pending),
            future_pending_at_last_tick=self.future_pending,
            selected_stamp=finite(self.selected),latest_received_stamp=finite(self.latest_received),
            future_wait_ticks=self.future_wait_ticks,overflow_drops=self.overflows,
            obsolete_drops=self.obsolete_drops,invalid_drops=self.invalid_drops)
