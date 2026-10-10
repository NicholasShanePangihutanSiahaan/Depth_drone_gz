"""Deterministic analytic ray sensor and a deliberately simple flight plant."""
import math
import numpy as np
from scipy.spatial.transform import Rotation

SCENARIOS = ('clear_orbit', 'leaf_blocked', 'vertical_detour', 'fully_blocked', 'sensor_dropout')


class Scene:
    def __init__(self, config, scenario='clear_orbit'):
        if scenario not in SCENARIOS:
            raise ValueError(scenario)
        self.c, self.scenario = config, scenario
        lo, hi = config['bounds_min'], config['bounds_max']
        self.boxes = [(np.array([lo[0], lo[1], -0.6]), np.array([hi[0], hi[1], 0.]))]
        # Asymmetric static landmarks keep the synthetic LIO problem observable.
        self.boxes += [(np.array([-3.8, -5.5, 0.]), np.array([-3.2, -4.8, 4.5])),
                       (np.array([10.8, 4.8, 0.]), np.array([11.6, 5.6, 5.5])),
                       (np.array([1., 4.4, 0.]), np.array([1.8, 5., 3.5]))]
        for tree in config['trees']:
            t = np.asarray(tree)
            self.boxes.append((t+[-0.2, -0.2, -t[2]], t+[0.2, 0.2, -0.2]))
        if scenario == 'leaf_blocked':
            self.boxes.append((np.array([1.5, -1.6, 0.]), np.array([3., -0.3, 2.9])))
        if scenario in ('vertical_detour', 'fully_blocked'):
            self.boxes.append((np.array([2.7, lo[1], 0.]),
                               np.array([3.3, hi[1], 2.8 if scenario == 'vertical_detour' else hi[2]])))

    def cast(self, origin, directions, max_range):
        origin, directions = np.asarray(origin), np.asarray(directions)
        distance = np.full(len(directions), max_range)
        for lo, hi in self.boxes:
            parallel = np.abs(directions) < 1e-12
            inverse = np.divide(1., directions, out=np.zeros_like(directions), where=~parallel)
            t1, t2 = (lo-origin)*inverse, (hi-origin)*inverse
            near = np.where(parallel, -np.inf, np.minimum(t1, t2))
            far = np.where(parallel, np.inf, np.maximum(t1, t2))
            missed_parallel = np.any(parallel & ((origin < lo) | (origin > hi)), axis=1)
            enter, leave = near.max(axis=1), far.min(axis=1)
            hit = (~missed_parallel) & (leave >= np.maximum(enter, 0))
            candidate = np.where(enter > 0, enter, leave)
            distance = np.minimum(distance, np.where(hit, candidate, np.inf))
        return distance, distance < max_range-1e-6

    def scan(self, position, rotation=None, noise=0., phase=0):
        """360 deg azimuth, -7..52 deg elevation, scan-end acquisition stamp.

        All rays are simultaneous: no rolling acquisition distortion. This is
        reduced-rate first-return geometry, not the Livox scan pattern/driver.
        """
        rotation = np.eye(3) if rotation is None else rotation
        az = np.linspace(-math.pi, math.pi, self.c['lidar_azimuth_samples'], endpoint=False)
        az += phase * 0.003
        el = np.linspace(math.radians(-7), math.radians(52), self.c['lidar_elevation_samples'])
        az, el = np.meshgrid(az, el)
        directions = np.column_stack((np.cos(el).ravel()*np.cos(az).ravel(),
                                      np.cos(el).ravel()*np.sin(az).ravel(), np.sin(el).ravel()))
        mount = self.c['lidar_mount']
        mount_r = Rotation.from_euler('xyz', mount[3:]).as_matrix()
        world_r = rotation @ mount_r
        origin = np.asarray(position)+rotation@np.asarray(mount[:3])
        distance, hits = self.cast(origin, directions@world_r.T, self.c['lidar_range'])
        if noise:
            distance[hits] += np.random.default_rng(phase).normal(0., noise, hits.sum())
        valid = distance >= self.c['lidar_min_range']
        local_points = directions[valid]*distance[valid, None]
        body_points = local_points@mount_r.T+mount[:3]
        self_mask = np.all(np.abs(body_points) <= np.asarray(self.c['drone_dimensions'])/2+self.c['self_filter_padding'], axis=1)
        local_points, hits = local_points[~self_mask], hits[valid][~self_mask]
        return origin, local_points@world_r.T+origin, hits, local_points

    def surface_clearance(self, position):
        """Exact minimum AABB separation for a yaw-independent vehicle envelope.

        Negative means overlap. Desired safety margin is assessed separately.
        """
        dims = np.asarray(self.c['drone_dimensions'])/2
        extent = np.array([np.linalg.norm(dims[:2]), np.linalg.norm(dims[:2]), dims[2]])
        separation = []
        for lo, hi in self.boxes:
            gaps = np.maximum(lo-(position+extent), (position-extent)-hi)
            separation.append(np.linalg.norm(np.maximum(gaps, 0)) if np.any(gaps > 0) else gaps.max())
        return float(min(separation))


class Plant:
    """Acceleration-limited point-mass tracking plant; not ArduPilot physics."""
    def __init__(self, position, config):
        self.p = np.asarray(position, float).copy()
        self.v = np.zeros(3)
        self.a = np.zeros(3)
        self.c = config

    def step(self, target, feedforward, dt):
        desired = 2.5*(np.asarray(target)-self.p)+1.8*(np.asarray(feedforward)-self.v)
        desired *= min(1., self.c['acceleration']/max(np.linalg.norm(desired), 1e-9))
        change = desired-self.a
        self.a += change*min(1., self.c['jerk']*dt/max(np.linalg.norm(change), 1e-9))
        self.v += self.a*dt
        self.p += self.v*dt
        return self.p.copy()
