"""Bounded C2 quintic flight chunks and jerk-aware adaptive time scaling.

These are reference trajectories, not a replacement for ArduPilot's attitude
controller. A chunk ends at rest so an unknown next chunk never requires an
unchecked moving handoff. Interior knots may be crossed without stopping.
"""
import math
import numpy as np


def split_bezier(controls):
    level = np.asarray(controls)
    left, right = [level[0]], [level[-1]]
    while len(level) > 1:
        level = (level[:-1]+level[1:])/2
        left.append(level[0])
        right.append(level[-1])
    return np.asarray(left), np.asarray(right[::-1])


def evaluate(controls, u):
    level = np.asarray(controls)
    while len(level) > 1:
        level = (1-u)*level[:-1]+u*level[1:]
    return level[0]


def derivative_bound(controls, depth=4):
    # Subdivision tightens a mathematically conservative convex-hull bound.
    if depth == 0:
        return float(np.linalg.norm(controls, axis=1).max())
    left, right = split_bezier(controls)
    return max(derivative_bound(left, depth-1), derivative_bound(right, depth-1))


def curve_safe(controls, grid, depth=0, certificate=None):
    lower = grid.indices(np.min(controls, axis=0)-1e-9)
    upper = grid.indices(np.max(controls, axis=0)+1e-9)
    if not grid.inside(lower) or not grid.inside(upper):
        return False
    region = tuple(slice(int(a), int(b)+1) for a, b in zip(lower, upper))
    if not np.any(grid.blocked[region]):
        if certificate is not None:
            certificate.append((lower.copy(), upper.copy()))
        return True  # The entire curve lies inside this observed-free box.
    if depth >= 12:
        return False  # Unresolved contact is rejected, not sampled away.
    left, right = split_bezier(controls)
    return (curve_safe(left, grid, depth+1, certificate) and
            curve_safe(right, grid, depth+1, certificate))


class ContinuousTrajectory:
    def __init__(self, points, config, tangent_scale=1., speed=None):
        points = np.asarray(points, dtype=float)
        points = points[np.r_[True, np.linalg.norm(np.diff(points, axis=0), axis=1) > 1e-8]]
        if len(points) < 2:
            raise ValueError('continuous trajectory needs two distinct points')
        self.points = points
        self.c = config
        vmax = min(config['speed'], config['command_speed'], speed or config['speed'])
        alpha = config.get('adaptive_rate_limit', .25)
        beta = config.get('adaptive_rate_acceleration', .4)
        # Reserve derivative headroom for the *actually commanded* time dilation.
        amax = config['acceleration']-vmax*alpha
        jmax = config['jerk']-3*amax*alpha-vmax*beta
        if min(amax, jmax) <= 0:
            raise ValueError('adaptive scaling has no acceleration/jerk reserve')
        distance = np.linalg.norm(np.diff(points, axis=0), axis=1)
        self.certificate = None
        self.geometry_checks = self.geometry_cache_hits = 0
        self.timing_policy = 'global_scale_legacy'
        if config.get('local_speed_profile', False):
            self._local_profile(distance, vmax, amax, jmax, tangent_scale)
            return
        durations = np.maximum(.2, 1.875*distance/vmax)
        velocities = np.zeros_like(points)
        for i in range(1, len(points)-1):
            before, after = points[i]-points[i-1], points[i+1]-points[i]
            before, after = before/np.linalg.norm(before), after/np.linalg.norm(after)
            direction = before+after
            if np.dot(before, after) > -.8 and np.linalg.norm(direction) > 1e-8:
                magnitude = tangent_scale*.8*min(distance[i-1]/durations[i-1], distance[i]/durations[i])
                velocities[i] = direction/np.linalg.norm(direction)*magnitude
        self.controls = []
        for i in range(len(points)-1):
            duration = durations[i]
            p, q, v, w = points[i], points[i+1], velocities[i], velocities[i+1]
            self.controls.append(np.array([p, p+v*duration/5, p+2*v*duration/5,
                                            q-2*w*duration/5, q-w*duration/5, q]))
        self.durations = durations
        bv, ba, bj = self.bounds()
        self.slow(max(1., bv/vmax, math.sqrt(ba/amax), np.cbrt(bj/jmax)))

    def _local_profile(self, distance, vmax, amax, jmax, tangent_scale):
        """Forward/backward knot envelope + LOCAL quintic derivative bounds.

        Not TOPPRA or a globally time-optimal optimizer. Shared knot derivatives
        keep C2 continuity. Each difficult piece is lengthened locally rather
        than applying its worst scaling factor to every straight piece.
        """
        directions = np.zeros_like(self.points)
        caps = np.full(len(self.points), vmax*tangent_scale)
        caps[[0, -1]] = 0.
        for i in range(1, len(self.points)-1):
            before = (self.points[i]-self.points[i-1])/distance[i-1]
            after = (self.points[i+1]-self.points[i])/distance[i]
            direction = before+after
            dot = np.clip(np.dot(before, after), -1., 1.)
            if dot <= -.8 or np.linalg.norm(direction) < 1e-8:
                caps[i] = 0.
                continue
            directions[i] = direction/np.linalg.norm(direction)
            curvature = 2*math.sin(math.acos(dot)/2)/max(min(distance[i-1], distance[i]), 1e-8)
            caps[i] = min(caps[i], math.sqrt(min(amax, self.c.get('adaptive_turn_acceleration', amax))/max(curvature, 1e-8)))
        # Propagate the finite acceleration needed before/after a slow knot.
        for i in range(1, len(caps)):
            caps[i] = min(caps[i], math.sqrt(caps[i-1]**2+amax*distance[i-1]))
        for i in range(len(caps)-2, -1, -1):
            caps[i] = min(caps[i], math.sqrt(caps[i+1]**2+amax*distance[i]))
        durations = np.maximum(.2, distance/vmax)
        for iteration in range(64):
            # Limit tangent lengths locally; otherwise stretching a short
            # segment while retaining a large knot velocity can overshoot it.
            magnitudes = caps.copy()
            for i in range(1, len(caps)-1):
                magnitudes[i] = min(magnitudes[i], 1.2*distance[i-1]/durations[i-1],
                                    1.2*distance[i]/durations[i])
            velocities = directions*magnitudes[:, None]
            controls = []
            factors = []
            for i, duration in enumerate(durations):
                p, q = self.points[i:i+2]
                v, w = velocities[i:i+2]
                control = np.array([p, p+v*duration/5, p+2*v*duration/5,
                                    q-2*w*duration/5, q-w*duration/5, q])
                controls.append(control)
                derivative, bound = control, []
                for order in range(1, 4):
                    derivative = (6-order)*np.diff(derivative, axis=0)/duration
                    bound.append(derivative_bound(derivative, depth=2))
                factors.append(max(bound[0]/vmax, math.sqrt(bound[1]/amax), np.cbrt(bound[2]/jmax)))
            factors = np.asarray(factors)
            if np.max(factors) <= 1.+1e-10:
                self.controls, self.durations = controls, durations
                self.knot_speed_caps = caps
                self.knot_speeds = magnitudes
                self.profile_iterations = iteration+1
                self.timing_policy = 'local_forward_backward_quintic_bounds'
                self.slow(1.)
                return
            durations *= np.where(factors > 1., np.maximum(1.02, factors*1.01), 1.)
        raise ValueError('local speed profile did not converge within bounded iterations')

    def bounds(self):
        maxima = np.zeros(3)
        for controls, duration in zip(self.controls, self.durations):
            derivative = controls
            for order in range(1, 4):
                derivative = (6-order)*np.diff(derivative, axis=0)/duration
                maxima[order-1] = max(maxima[order-1], derivative_bound(derivative))
        return maxima

    def slow(self, factor):
        self.durations *= factor
        self.ends = np.cumsum(self.durations)
        self.duration = float(self.ends[-1])

    def derivatives(self, elapsed):
        i = min(int(np.searchsorted(self.ends, max(0., elapsed), side='right')), len(self.controls)-1)
        begin = self.ends[i-1] if i else 0.
        duration = self.durations[i]
        u = float(np.clip((elapsed-begin)/duration, 0., 1.))
        derivative = self.controls[i]
        values = [evaluate(derivative, u)]
        for order in range(1, 4):
            derivative = (6-order)*np.diff(derivative, axis=0)/duration
            values.append(evaluate(derivative, u))
        return values

    def sample(self, elapsed):
        return tuple(self.derivatives(elapsed)[:3])

    def safe(self, grid):
        key = (grid.res, tuple(grid.lo), tuple(grid.shape))
        if self.c.get('trajectory_geometry_cache', False) and self.certificate is not None:
            previous_key, boxes = self.certificate
            if previous_key == key and all(not np.any(grid.blocked[tuple(
                    slice(int(a), int(b)+1) for a,b in zip(lower, upper))]) for lower, upper in boxes):
                self.geometry_cache_hits += 1
                return True
        self.geometry_checks += 1
        boxes = []
        safe = all(curve_safe(controls, grid, certificate=boxes) for controls in self.controls)
        self.certificate = (key, boxes) if safe else None
        return safe


class AdaptivePhase:
    """Smooth changes of phase speed, with exact integration and chain rule."""
    def __init__(self, config):
        self.alpha = config.get('adaptive_rate_limit', .25)
        self.beta = config.get('adaptive_rate_acceleration', .4)
        self.minimum = config.get('adaptive_min_scale', .25)
        self.start = self.target = self.rate = 1.
        self.elapsed = self.duration = 0.
        self.rate_dot = self.rate_ddot = 0.

    def integral(self, t):
        if self.duration == 0:
            return self.target*t
        u = min(t/self.duration, 1.)
        return (self.start*min(t, self.duration)+(self.target-self.start)*self.duration*
                (2.5*u**4-3*u**5+u**6)+self.target*max(0., t-self.duration))

    def step(self, dt, target):
        target = float(np.clip(target, self.minimum, 1.))
        if self.elapsed >= self.duration and abs(target-self.target) > .02:
            self.start, self.target, self.elapsed = self.rate, target, 0.
            delta = abs(self.target-self.start)
            self.duration = max(.5, 1.875*delta/self.alpha, math.sqrt(5.774*delta/self.beta))
        advance = self.integral(self.elapsed+dt)-self.integral(self.elapsed)
        self.elapsed += dt
        if self.duration == 0 or self.elapsed >= self.duration:
            self.rate, self.rate_dot, self.rate_ddot = self.target, 0., 0.
        else:
            u, delta = self.elapsed/self.duration, self.target-self.start
            self.rate = self.start+delta*(10*u**3-15*u**4+6*u**5)
            self.rate_dot = delta*(30*u**2-60*u**3+30*u**4)/self.duration
            self.rate_ddot = delta*(60*u-180*u**2+120*u**3)/self.duration**2
        return advance
