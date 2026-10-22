"""3D A*, bounded quintic trajectories and safety gates."""
import heapq
import itertools
import math
from dataclasses import dataclass
from .time_checks import fresh_stamp
import numpy as np


def plan(grid, start, goal, max_expansions=50000, diagnostics=None):
    """26-connected A*, with swept edge checking (no corner cutting)."""
    start, goal = np.asarray(start, float), np.asarray(goal, float)
    if diagnostics is not None:
        diagnostics['reason'] = 'endpoint_not_safe'
    if not grid.safe(start) or not grid.safe(goal):
        return None
    from .path_cost import clearance_cost_field,line_travel_cost
    field=clearance_cost_field(grid)
    if diagnostics is not None and field is not None:
        diagnostics.update(clearance_preference_m=grid.c['planner_clearance_preference'],
            clearance_cost_weight=grid.c['planner_clearance_cost_weight'],
            clearance_basis='distance_to_inflated_blocked_mask_not_physical_clearance')
    direct_safe=grid.line_safe(start, goal)
    direct_cost=line_travel_cost(grid,start,goal,field) if direct_safe else math.inf
    def direct_result(fallback=None):
        if diagnostics is not None:
            diagnostics.update(reason='direct',travel_cost=float(direct_cost))
            if fallback:diagnostics['clearance_search_fallback']=fallback
        return [start,goal]
    if direct_safe and (field is None or direct_cost<=np.linalg.norm(goal-start)+1e-9):
        return direct_result()
    s, g = tuple(grid.indices(start)), tuple(grid.indices(goal))
    if not grid.line_safe(start, grid.centers(s)) or not grid.line_safe(grid.centers(g), goal):
        if direct_safe:return direct_result('endpoint_grid_connection')
        if diagnostics is not None:
            diagnostics['reason'] = 'endpoint_grid_connection'
        return None
    queue = [(0., s)]
    costs, parent = {s: 0.}, {}
    closed = set()
    moves = [np.array(x) for x in itertools.product((-1, 0, 1), repeat=3) if any(x)]
    while queue and len(closed) < max_expansions:
        _, u = heapq.heappop(queue)
        if u in closed:
            continue
        closed.add(u)
        if u == g:
            ids = [u]
            while u != s:
                u = parent[u]
                ids.append(u)
            points = [start] + [grid.centers(i) for i in reversed(ids)] + [goal]
            # Shortcut only if safe AND no more expensive than the removed
            # subpath. Otherwise shortening would restore obstacle-hugging.
            prefix=np.r_[0.,np.cumsum([line_travel_cost(grid,a,b,field)
                for a,b in zip(points[:-1],points[1:])])]
            result, index = [points[0]], 0
            while index < len(points)-1:
                nxt = len(points)-1
                while nxt > index+1 and (not grid.line_safe(points[index], points[nxt])
                        or (field is not None and line_travel_cost(grid,points[index],points[nxt],field)
                            > prefix[nxt]-prefix[index]+1e-9)):
                    nxt -= 1
                result.append(points[nxt])
                index = nxt
            result_cost=sum(line_travel_cost(grid,a,b,field)
                for a,b in zip(result[:-1],result[1:]))
            if direct_safe and direct_cost<=result_cost+1e-9:
                return direct_result()
            if diagnostics is not None:
                diagnostics.update(reason='detour', expansions=len(closed),travel_cost=result_cost)
            return result
        for move in moves:
            v = tuple(np.asarray(u)+move)
            if v in closed or not grid.inside(np.asarray(v)) or grid.blocked[v]:
                continue
            # The seven intermediate cells of a diagonal must be traversable.
            axes = np.flatnonzero(move)
            corner_safe = True
            for mask in itertools.product((0, 1), repeat=len(axes)):
                mid = np.asarray(u).copy()
                mid[axes] += move[axes] * mask
                if grid.blocked[tuple(mid)]:
                    corner_safe = False
                    break
            if not corner_safe:
                continue
            distance = np.linalg.norm(move)*grid.res
            candidate = costs[u] + distance*(1. if field is None else (field[u]+field[v])/2)
            if candidate < costs.get(v, math.inf):
                costs[v], parent[v] = candidate, u
                h = np.linalg.norm(np.asarray(v)-g)*grid.res
                heapq.heappush(queue, (candidate+h, v))
    if direct_safe:return direct_result('search_limit' if queue else 'disconnected')
    if diagnostics is not None:
        diagnostics.update(reason='search_limit' if queue else 'disconnected', expansions=len(closed))
    return None


class Trajectory:
    """Stop-to-stop quintic segments: position, velocity and acceleration continuous.

    Every segment is straight. Bounded analytic derivatives and collision checks
    apply to the ACTUAL interpolation, with no post-planner slew/terrain node.
    """
    def __init__(self, points, config, speed=None):
        self.points = np.asarray(points)
        d = np.linalg.norm(np.diff(self.points, axis=0), axis=1)
        vmax = min(config['speed'], config['command_speed'], speed or config['speed'])
        self.durations = np.maximum.reduce((1.875*d/vmax,
                                           np.sqrt(5.774*d/config['acceleration']),
                                           np.cbrt(60*d/config['jerk']),
                                           np.full(len(d), 0.1)))
        self.ends = np.cumsum(self.durations)
        self.duration = float(self.ends[-1])

    def sample(self, elapsed):
        i = min(int(np.searchsorted(self.ends, max(0., elapsed), side='right')), len(self.durations)-1)
        begin = self.ends[i-1] if i else 0.
        t = self.durations[i]
        u = float(np.clip((elapsed-begin)/t, 0., 1.))
        delta = self.points[i+1]-self.points[i]
        s = 10*u**3-15*u**4+6*u**5
        velocity = delta*(30*u**2-60*u**3+30*u**4)/t
        acceleration = delta*(60*u-180*u**2+120*u**3)/t**2
        return self.points[i]+delta*s, velocity, acceleration

    def slow(self, factor):
        self.durations *= factor
        self.ends = np.cumsum(self.durations)
        self.duration = float(self.ends[-1])

    def safe(self, grid):
        return all(grid.line_safe(a, b) for a, b in zip(self.points[:-1], self.points[1:]))


class OrbitTrajectory:
    """One smooth, continuous tree-facing circuit, starting and ending at rest."""
    def __init__(self, tree, config):
        self.center = np.array([tree[0], tree[1], config['orbit_altitude']], float)
        self.radius = config['orbit_radius']
        self.duration = 1.875*2*math.pi*self.radius/min(config['speed'], config['command_speed'], config['orbit_speed'])
        for _ in range(20):
            values = [self.sample(t) for t in np.linspace(0., self.duration, 401)]
            accelerations = np.array([x[2] for x in values])
            jerk = np.diff(accelerations, axis=0)/(self.duration/400)
            if (np.linalg.norm(accelerations, axis=1).max() <= config['acceleration']
                    and np.linalg.norm(jerk, axis=1).max() <= config['jerk']):
                break
            self.duration *= 1.2
        self.refresh_points()

    def refresh_points(self):
        self.points = np.array([self.sample(t)[0] for t in np.linspace(0., self.duration, 721)])

    def slow(self, factor):
        self.duration *= factor
        self.refresh_points()

    def sample(self, elapsed):
        u = float(np.clip(elapsed/self.duration, 0., 1.))
        theta = math.pi+2*math.pi*(10*u**3-15*u**4+6*u**5)
        omega = 2*math.pi*(30*u**2-60*u**3+30*u**4)/self.duration
        alpha = 2*math.pi*(60*u-180*u**2+120*u**3)/self.duration**2
        radial = np.array([math.cos(theta), math.sin(theta), 0.])
        tangent = np.array([-math.sin(theta), math.cos(theta), 0.])
        return self.center+self.radius*radial, self.radius*omega*tangent, self.radius*(alpha*tangent-omega**2*radial)

    def safe(self, grid):
        return all(grid.line_safe(a, b) for a, b in zip(self.points[:-1], self.points[1:]))


@dataclass
class Health:
    cloud_stamp: float = -math.inf
    pose_stamp: float = -math.inf
    imu_stamp: float = -math.inf
    localisation_ok: bool = False

    def reason(self, now, config, sensor_mode=False, require_lidar=True):
        if not self.localisation_ok:
            return 'localisation_loss'
        if not fresh_stamp(now,self.pose_stamp,config['pose_timeout']):
            return 'stale_localisation'
        if require_lidar and not fresh_stamp(now,self.cloud_stamp,config['cloud_timeout']):
            return 'stale_lidar'
        if sensor_mode and not fresh_stamp(now,self.imu_stamp,config['pose_timeout']):
            return 'stale_imu'
        return ''

    def status(self,now,config):
        def finite(value):return float(value) if value is not None and math.isfinite(value) else None
        return dict(pose_stamp=finite(self.pose_stamp),cloud_stamp=finite(self.cloud_stamp),
            imu_stamp=finite(self.imu_stamp),pose_age=finite(now-self.pose_stamp),
            cloud_age=finite(now-self.cloud_stamp),imu_age=finite(now-self.imu_stamp),
            pose_timeout_seconds=float(config['pose_timeout']),
            cloud_timeout_seconds=float(config['cloud_timeout']),
            numeric_roundoff_tolerance_seconds=1e-9,localisation_ok=bool(self.localisation_ok))


def brake_safe(grid, position, velocity, config, acceleration=None, actual=False, diagnostics=None):
    def result(value, reason):
        if diagnostics is not None:
            diagnostics.update(safe=bool(value), reason=reason, actual_margin=actual)
        return value
    speed = float(np.linalg.norm(velocity))
    if diagnostics is not None:
        diagnostics.update(position=np.asarray(position).tolist(),
                           velocity=np.asarray(velocity).tolist(), speed_mps=speed)
    if speed < 1e-6:
        safe = grid.safe(position, actual)
        if not safe and diagnostics is not None and hasattr(grid, 'describe_blockage'):
            diagnostics['blockage'] = grid.describe_blockage(grid.indices(position), actual)
        return result(safe, 'stationary_safe' if safe else 'stationary_blocked')
    acceleration = np.zeros(3) if acceleration is None else np.asarray(acceleration)
    reaction_end = np.asarray(position)+np.asarray(velocity)*config['reaction_time']
    if diagnostics is not None:
        diagnostics.update(acceleration=acceleration.tolist(), reaction_end=reaction_end.tolist(),
                           reaction_time_seconds=float(config['reaction_time']))
    if not grid.line_safe(position, reaction_end, actual):
        if diagnostics is not None and hasattr(grid, 'describe_blockage'):
            detail = {}
            grid.line_safe(position, reaction_end, actual, diagnostics=detail)
            diagnostics['blockage'] = detail
        return result(False, 'reaction_segment_blocked')
    stop = Brake(reaction_end, velocity, acceleration, config)
    if diagnostics is not None:
        diagnostics.update(predicted_max_speed_mps=stop.max_speed, stop_duration_seconds=stop.duration)
    if stop.max_speed > config['speed']+1e-6:
        return result(False, 'predicted_speed_limit')
    safe = stop.path_safe(grid, actual, diagnostics)
    return result(safe, 'safe' if safe else diagnostics.get('path_reason', 'braking_path_blocked')
                  if diagnostics is not None else 'braking_path_blocked')


class Brake:
    """Smooth stop seeded with the previous commanded velocity/acceleration.

    The whole braking curve is rechecked. If no stopping corridor exists the
    caller reports emergency_unavoidable, instead of pretending a hold is safe.
    """
    def __init__(self, position, velocity, acceleration, config):
        self.p = np.asarray(position).copy()
        self.v = np.asarray(velocity).copy()
        self.a = np.asarray(acceleration).copy()
        self.c = config
        self.elapsed = 0.
        self.finished = False
        duration = max(0.3, 3*np.linalg.norm(self.v)/config['acceleration'],
                       np.sqrt(12*np.linalg.norm(self.v)/config['jerk']),
                       3*np.linalg.norm(self.a)/config['jerk'])
        # Quintic boundary-value stop preserves initial acceleration, ending
        # with zero velocity AND acceleration. Increase duration until bounded.
        for _ in range(40):
            end = self.p+self.v*duration/2+self.a*duration**2/12
            coeff = np.zeros((6, 3))
            coeff[:3] = [self.p, self.v, self.a/2]
            t = duration
            matrix = [[t**3, t**4, t**5], [3*t**2, 4*t**3, 5*t**4], [6*t, 12*t**2, 20*t**3]]
            rhs = [end-(self.p+self.v*t+self.a*t*t/2), -self.v-self.a*t, -self.a]
            coeff[3:] = np.linalg.solve(matrix, rhs)
            times = np.linspace(0., t, 201)
            accelerations = sum(k*(k-1)*times[:, None]**(k-2)*coeff[k] for k in range(2, 6))
            jerks = sum(k*(k-1)*(k-2)*times[:, None]**(k-3)*coeff[k] for k in range(3, 6))
            if (np.linalg.norm(accelerations, axis=1).max() <= config['acceleration']+1e-8
                    and np.linalg.norm(jerks, axis=1).max() <= config['jerk']+1e-8):
                break
            duration *= 1.1
        else:
            raise ValueError('cannot construct a bounded stop')
        self.coeff, self.duration = coeff, duration
        velocities = sum(k*times[:, None]**(k-1)*coeff[k] for k in range(1, 6))
        self.max_speed = float(np.linalg.norm(velocities, axis=1).max())

    def path_safe(self, grid, actual=False, diagnostics=None):
        # A polynomial lies inside its Bernstein/Bezier control hull. Bound
        # each short interval with that hull, so thin voxel intersections cannot
        # be missed between samples of the curved braking trajectory.
        count = max(25, int(self.duration*max(self.max_speed, 0.01)*8/grid.res)+1)
        times = np.linspace(self.elapsed, self.duration, count+1)
        start, width = times[:-1], times[1:]-times[:-1]
        local = np.zeros((count, 6, 3))
        for k in range(6):
            for j in range(k, 6):
                local[:, k] += (math.comb(j, k)*start**(j-k)*width**k)[:, None]*self.coeff[j]
        controls = np.zeros_like(local)
        for i in range(6):
            for k in range(i+1):
                controls[:, i] += math.comb(i, k)/math.comb(5, k)*local[:, k]
        lo, hi = grid.indices(controls.min(axis=1)), grid.indices(controls.max(axis=1))
        if not np.all(grid.inside(lo)) or not np.all(grid.inside(hi)):
            if diagnostics is not None:
                index = int(np.flatnonzero(~(grid.inside(lo) & grid.inside(hi)))[0])
                diagnostics.update(path_reason='braking_outside_window',
                    interval_seconds=[float(times[index]),float(times[index+1])],
                    blocked_box_min=grid.centers(lo[index]).tolist(),
                    blocked_box_max=grid.centers(hi[index]).tolist())
                if hasattr(grid, 'describe_blockage'):
                    cell = lo[index] if not grid.inside(lo[index]) else hi[index]
                    diagnostics['blockage'] = grid.describe_blockage(cell, actual)
            return False
        blocked = grid.blocked_actual if actual else grid.blocked
        for index, (lower, upper) in enumerate(zip(lo, hi)):
            region = tuple(slice(int(a), int(b)+1) for a, b in zip(lower, upper))
            if np.any(blocked[region]):
                if diagnostics is not None:
                    diagnostics.update(path_reason='braking_box_blocked',
                        interval_seconds=[float(times[index]),float(times[index+1])],
                        witness_policy='conservative_Bezier_hull_box_not_confirmed_curve_collision',
                        blocked_box_min=grid.centers(lower).tolist(),
                        blocked_box_max=grid.centers(upper).tolist(),
                        raw_unknown_cells=int(np.count_nonzero(grid.state[region] == -1)),
                        raw_occupied_cells=int(np.count_nonzero(grid.state[region] == 1)))
                    if hasattr(grid, 'describe_blockage'):
                        cell = lower+np.argwhere(blocked[region])[0]
                        diagnostics['blockage'] = grid.describe_blockage(cell, actual)
                return False
        return True

    def step(self, dt):
        self.elapsed = min(self.elapsed+dt, self.duration)
        t, b = self.elapsed, self.coeff
        self.p = sum(b[k]*t**k for k in range(6))
        self.v = sum(k*b[k]*t**(k-1) for k in range(1, 6))
        self.a = sum(k*(k-1)*b[k]*t**(k-2) for k in range(2, 6))
        self.finished = self.elapsed >= self.duration
        return self.p.copy()
