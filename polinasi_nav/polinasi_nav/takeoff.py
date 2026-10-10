"""Original beehive takeoff sequence, with bounded ACK/progress monitoring.

Based on pratesting_works.py and missions/basic_orbit.py in the preserved
original repository: GUIDED -> armed -> one NAV_TAKEOFF -> hover telemetry.
The ACK/progress bounds follow its simple_single_tree_mission.py.
No position setpoints are emitted during the autopilot's takeoff phase.
"""


class TakeoffSequence:
    def __init__(self, now, altitude, settle=.35, hover_time=2.):
        self.phase = 'ARM_SETTLE'
        self.since = now
        self.altitude = altitude
        self.settle, self.hover_time = settle, hover_time
        self.attempts = 0
        self.result = None
        self.max_altitude = altitude
        self.hover_since = None
        self.failure = ''

    def acknowledge(self, result):
        if self.phase == 'WAIT_ACK':
            self.result = result

    def step(self, now, altitude, armed, hovering, safe, speed):
        if not armed:
            self.failure = 'takeoff_disarmed'
            return 'abort'
        elapsed = now-self.since
        if self.phase == 'ARM_SETTLE' and elapsed >= self.settle:
            self.phase, self.since = 'WAIT_ACK', now
            self.attempts += 1
            self.result = None
            return 'takeoff'
        if self.phase == 'WAIT_ACK':
            if self.result in (0, 5):
                self.phase, self.since = 'CLIMB', now
            elif self.result is not None or elapsed >= 4.:
                result = self.result if self.result is not None else 255
                if result in (1, 255) and self.attempts < 3:
                    self.phase, self.since = 'RETRY_WAIT', now
                else:
                    self.failure = f'takeoff_rejected_{result}'
                    return 'abort'
        elif self.phase == 'RETRY_WAIT' and elapsed >= 2.:
            self.phase, self.since = 'ARM_SETTLE', now-self.settle
        elif self.phase == 'CLIMB':
            self.max_altitude = max(self.max_altitude, altitude)
            # Original hover telemetry remains the altitude-completion gate.
            # Fresh observations and low measured speed are additional safety
            # gates before handing off to collision-checked navigation.
            if hovering and safe and speed < .15:
                self.hover_since = now if self.hover_since is None else self.hover_since
                if now-self.hover_since >= self.hover_time:
                    self.phase = 'READY'
                    return 'ready'
            else:
                self.hover_since = None
            if elapsed >= 20. and self.max_altitude-self.altitude < .2:
                self.failure = 'takeoff_no_climb'
                return 'abort'
            if elapsed >= 45.:
                self.failure = 'takeoff_hover_timeout'
                return 'abort'
        return None
