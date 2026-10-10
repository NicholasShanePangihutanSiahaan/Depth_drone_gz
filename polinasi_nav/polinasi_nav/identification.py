"""Small PVA identification manoeuvres; no ArduPilot gain changes or AUTOTUNE."""
import math
import numpy as np
from .control import MissionController
from .planning import brake_safe
from .mapping import VoxelMap, FREE, OCCUPIED


def empty_arena_fixture(config):
    """Known simulation geometry, NOT an observed LiDAR occupancy map."""
    if not config.get('identification_simulation_only') or not config.get('identification_empty_arena'):
        raise ValueError('empty arena requires the simulation identification preset')
    grid = VoxelMap(config)
    grid.state[:] = FREE
    grid.seen[:] = 0.
    heights = grid.lo[2]+(np.arange(grid.shape[2])+.5)*grid.res
    grid.state[:, :, heights < 0.] = OCCUPIED
    grid.rebuild(0.)
    return grid


class Excitation:
    """Analytic sine/chirp with a sin^4 envelope: P/V/A/J vanish at both ends."""
    def __init__(self, duration, amplitude, frequency, sweep=0.):
        self.duration, self.amplitude, self.frequency, self.sweep = duration, amplitude, frequency, sweep

    def sample(self, t):
        if t <= 0 or t >= self.duration:
            return np.zeros(4)
        w = math.pi/self.duration
        # sin^4(wt) = (3 - 4 cos(2wt) + cos(4wt))/8
        envelope = np.array([sum(c*k**n*math.cos(k*t+n*math.pi/2)
            for c, k in ((-.5, 2*w), (.125, 4*w))) for n in range(4)])
        envelope[0] += .375
        phase = 2*math.pi*(self.frequency*t+.5*self.sweep*t*t/self.duration)
        d = 2*math.pi*(self.frequency+self.sweep*t/self.duration)
        dd = 2*math.pi*self.sweep/self.duration
        s, c = math.sin(phase), math.cos(phase)
        carrier = np.array([s, c*d, -s*d*d+c*dd, -c*d**3-3*s*d*dd])
        return self.amplitude*np.array([envelope[0]*carrier[0],
            envelope[1]*carrier[0]+envelope[0]*carrier[1],
            envelope[2]*carrier[0]+2*envelope[1]*carrier[1]+envelope[0]*carrier[2],
            envelope[3]*carrier[0]+3*envelope[2]*carrier[1]+3*envelope[1]*carrier[2]+envelope[0]*carrier[3]])


def protocol(profile):
    if profile == 'train':
        return [('HOVER', 4., []),
            ('X_SINE', 14., [(0, .16, .10, 0.)]), ('REST_X', 2., []),
            ('X_SWEEP', 14., [(0, .14, .10, .08)]), ('REST_X2', 2., []),
            ('Y_SINE', 14., [(1, .16, .12, 0.)]), ('REST_Y', 2., []),
            ('Y_SWEEP', 14., [(1, .14, .09, .09)]), ('REST_Y2', 2., []),
            ('Z_SWEEP', 16., [(2, .08, .08, .06)]), ('REST_Z', 2., []),
            ('YAW', 10., [(3, .15, .10, 0.)]), ('FINAL_HOVER', 4., [])]
    if profile == 'validation':
        return [('HOVER', 4., []), ('DIAGONAL', 16., [(0, .12, .13, 0.), (1, .10, .10, .02)]),
            ('REST', 3., []), ('XYZ', 16., [(0, .10, .09, .04), (1, .12, .12, -.02), (2, .06, .11, 0.)]),
            ('FINAL_HOVER', 4., [])]
    raise ValueError('identification profile must be train or validation')


class IdentificationController(MissionController):
    def __init__(self, config, grid):
        super().__init__(config, grid, False)
        self.airborne_state = 'IDENTIFY'
        self.waypoints = np.asarray([config['home']])
        self.profile = config['identification_profile']
        self.steps = protocol(self.profile)
        self.index, self.phase_time, self.identification_phase = 0, 0., 'WAIT_TAKEOFF'
        self.anchor = None
        self.completed_protocol = False
        # Fail configuration before arming if analytic command limits cannot fit.
        for _, duration, signals in self.steps:
            for t in np.linspace(0., duration, 501):
                values = np.zeros((4, 4))
                for axis, amp, freq, sweep in signals:
                    values[axis] = Excitation(duration, amp, freq, sweep).sample(t)
                if (np.linalg.norm(values[:3, 1]) > config['command_speed'] or
                    np.linalg.norm(values[:3, 2]) > config['acceleration'] or
                    np.linalg.norm(values[:3, 3]) > config['jerk'] or abs(values[3, 1]) > config['yaw_rate']):
                    raise ValueError('identification excitation exceeds configured limits')

    def tick(self, now, dt, position, velocity, range_value=None, range_stamp=None):
        reason = self.health.reason(now, self.c, False,
            require_lidar=not self.c.get('identification_empty_arena', False))
        if reason:
            self.fail(reason)
        if self.failure:
            return self._braking(dt)
        if self.state in ('LAND', 'COMPLETE'):
            return self.command.copy(), np.zeros(3)
        if (np.linalg.norm(np.asarray(position)-self.command) > self.c['tracking_limit'] or
                np.linalg.norm(velocity) > self.c['speed']+.05):
            self.fail('identification_tracking_or_speed')
            return self._braking(dt)
        if not brake_safe(self.grid, position, velocity, self.c, self.command_a, actual=True):
            self.fail('identification_stopping_clearance')
            return self._braking(dt)
        if self.anchor is None:
            self.anchor = self.command.copy()
            self.anchor_yaw = self.yaw
        name, duration, signals = self.steps[self.index]
        self.identification_phase = name
        t = min(duration, self.phase_time+dt)
        values = np.zeros((4, 4))
        for axis, amp, freq, sweep in signals:
            values[axis] = Excitation(duration, amp, freq, sweep).sample(t)
        p = self.anchor+values[:3, 0]
        v, a = values[:3, 1], values[:3, 2]
        if (not self.grid.line_safe(self.command, p) or not brake_safe(self.grid, p, v, self.c, a)
                or np.max(np.abs(np.asarray(position)-self.anchor)) > .65):
            self.fail('identification_command_or_geofence')
            return self._braking(dt)
        self.command, self.command_v, self.command_a = p, v, a
        self.yaw = self.anchor_yaw+values[3, 0]
        self.phase_time = t
        if t >= duration:
            self.index += 1
            self.phase_time = 0.
            if self.index == len(self.steps):
                self.completed_protocol = True
                self.state, self.land_requested = 'LAND', True
                self.identification_phase = 'LAND'
        return self.command.copy(), self.command_v.copy()

    def predictive_reference(self, count=10, step=.1):
        rows = []
        for k in range(count):
            index, t = self.index, self.phase_time+k*step
            while index < len(self.steps)-1 and t > self.steps[index][1]:
                t -= self.steps[index][1]
                index += 1
            _, duration, signals = self.steps[min(index, len(self.steps)-1)]
            values = np.zeros((4, 4))
            for axis, amp, freq, sweep in signals:
                values[axis] = Excitation(duration, amp, freq, sweep).sample(min(t, duration))
            rows.append(np.r_[self.anchor+values[:3, 0], values[:3, 1], values[:3, 2]])
        return np.asarray(rows)
