"""Bounded statistics; receipt gaps are not Ethernet packet-loss measurements."""
from collections import deque
import numpy as np


def stats(values):
    a = np.asarray(list(values), float)
    return dict(p50=float(np.percentile(a, 50)), p95=float(np.percentile(a, 95)),
                max=float(a.max())) if len(a) else dict(p50=None, p95=None, max=None)


class StreamHealth:
    def __init__(self, timeout, expected_hz=0.):
        self.timeout, self.expected = timeout, expected_hz
        self.count = self.invalid = self.out_of_order = self.estimated_missing = 0
        self.first = self.last = self.stamp = None
        self.gaps, self.ages, self.processing = (deque(maxlen=4096) for _ in range(3))
        self.max_gap = 0.

    def receive(self, receipt, stamp, valid=True, age=None, processing_ms=None):
        if self.last is not None:
            gap = receipt-self.last
            if gap >= 0:
                self.gaps.append(gap)
                self.max_gap = max(self.max_gap, gap)
                if self.expected > 0:
                    self.estimated_missing += max(0, int(round(gap*self.expected))-1)
        if self.stamp is not None and stamp <= self.stamp:
            self.out_of_order += 1
        self.stamp = stamp
        self.first = receipt if self.first is None else self.first
        self.last = receipt
        self.count += 1
        self.invalid += int(not valid)
        if age is not None and np.isfinite(age):
            self.ages.append(age)
        if processing_ms is not None:
            self.processing.append(processing_ms)

    def report(self, now):
        age = None if self.last is None else max(0., now-self.last)
        duration = 0. if self.first is None else now-self.first
        hz = ((self.count-1)/(self.last-self.first)
              if self.count > 1 and self.last > self.first else None)
        status = ('NOT TESTED' if not self.count else 'FAIL'
                  if age > self.timeout or self.invalid or self.out_of_order else 'PASS')
        return dict(status=status, scope='received_stream_integrity_not_sensor_accuracy',
                    count=self.count, invalid_messages=self.invalid, out_of_order=self.out_of_order,
                    receive_hz=hz, observation_seconds=duration, receipt_age_seconds=age,
                    receipt_gap_seconds=stats(self.gaps), maximum_gap_seconds=self.max_gap,
                    source_age_seconds=stats(self.ages), processing_ms=stats(self.processing),
                    statistics_window=4096, expected_hz=self.expected,
                    estimated_missing_messages=self.estimated_missing if self.expected else None,
                    loss_scope='receipt_gap_estimate_not_UDP_loss',
                    frequency_gate='NOT TESTED_expected_rate_is_reference_only')
