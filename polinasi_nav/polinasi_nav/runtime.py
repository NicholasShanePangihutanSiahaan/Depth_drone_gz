"""Wall-time diagnostics; never changes simulation deadlines or control state."""
import functools
import time
from collections import deque


def timed_job(function, submitted_ns, *args):
    """Monotonic clocks are shared by processes on this Linux host."""
    started = time.monotonic_ns()
    result = function(*args)
    return (*result, dict(worker_started_ns=started,
                         worker_finished_ns=time.monotonic_ns(), submitted_ns=submitted_ns))


class PipelineTiming:
    """One sample per map version at its FIRST final ROS publication.

    This measures map availability + current command safety, not proof that
    the reference/MPC solution was generated from this particular scan.
    """
    def __init__(self):
        self.samples = deque(maxlen=2048)
        self.last_version = None
        self.maximum = 0.
        self.count = 0
        self.last = None

    def record(self, trace, version, published_ns, simulation_age):
        if not trace or version == self.last_version:
            return
        names = ('received_ns', 'submitted_ns', 'worker_started_ns',
                 'worker_finished_ns', 'integrated_ns', 'navigation_received_ns')
        try:
            stamps = [int(trace[name]) for name in names]+[int(published_ns)]
        except (KeyError, ValueError, TypeError):
            return
        if any(b < a for a, b in zip(stamps, stamps[1:])):
            return
        stages = ('receive_to_submit', 'worker_queue', 'map_worker',
                  'worker_result_delivery', 'snapshot_transport_and_decode',
                  'navigation_to_setpoint')
        total = (stamps[-1]-stamps[0])/1e6
        self.last = dict(map_version=int(version), scan_stamp=trace.get('scan_stamp'),
            total_wall_ms=total, measurement_to_publish_sim_seconds=simulation_age,
            stages_wall_ms={k:(b-a)/1e6 for k,a,b in zip(stages, stamps, stamps[1:])})
        self.samples.append(total)
        self.maximum = max(self.maximum, total)
        self.count += 1
        self.last_version = version

    def status(self):
        values = sorted(self.samples)
        def percentile(p):
            return values[min(len(values)-1, int((len(values)-1)*p))] if values else None
        return dict(scope='mapping_ROS_receive_to_first_final_ROS_setpoint_using_map_for_safety',
            clock='same_host_monotonic_wall_time', samples=self.count,
            rolling_window=len(values), p50_wall_ms=percentile(.5), p95_wall_ms=percentile(.95),
            max_wall_ms=self.maximum if self.count else None, last=self.last,
            excludes='sensor_acquisition_before_ROS_receive; MAVROS_FCU_delivery; vehicle_response',
            reference_causality='latest_map_safety_check; reference_and_MPC_may_use_older_snapshot')


class CallbackTiming:
    def __init__(self):
        self.max_ms = {}
        self.last = None

    def record(self, name, seconds):
        value = seconds*1000.
        self.max_ms[name] = max(value, self.max_ms.get(name, 0.))
        self.last = dict(name=name, wall_ms=value)


def measured(name):
    def decorate(function):
        @functools.wraps(function)
        def wrapped(self, *args, **kwargs):
            started = time.perf_counter()
            try:
                return function(self, *args, **kwargs)
            finally:
                timing = getattr(self, 'callback_timing', None)
                if timing is not None:
                    timing.record(name, time.perf_counter()-started)
        return wrapped
    return decorate
