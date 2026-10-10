"""Sparse whole-farm history plus bounded rolling navigation occupancy.

History is measured/prior data, never simulator ground-truth geometry. Missing
cells are unknown. Historical free cells have a bounded, explicitly configured
lifetime before they may be restored into the collision-checking window.
"""
import copy
import numpy as np
from .mapping import VoxelMap, FREE, OCCUPIED


def navigation_window(config, centre):
    c = copy.deepcopy(config)
    lo, hi = np.asarray(c['bounds_min']), np.asarray(c['bounds_max'])
    size = np.minimum(np.asarray(c['navigation_window_size']), hi-lo)
    # Quantised window anchors preserve exact global/local voxel correspondence.
    anchor = np.floor((np.asarray(centre)-size/2-lo)/c['resolution'])*c['resolution']+lo
    anchor = np.maximum(lo, np.minimum(anchor, hi-size))
    c.update(bounds_min=anchor.tolist(), bounds_max=(anchor+size).tolist())
    return VoxelMap(c)


class SparseHistory:
    def __init__(self, config):
        self.c = copy.deepcopy(config)
        self.lo = np.asarray(config['bounds_min'], dtype=float)
        self.res = config['resolution']
        self.shape = tuple(np.ceil((np.asarray(config['bounds_max'])-self.lo)/self.res).astype(int))
        self.cells = {}  # flat global index -> (state, last measured timestamp)
        self.prior = set()  # Explicit fixed surveyed launch pad only.
        self.occupied_keys = set()
        self.observed_columns = set()
        self.last_observation_stamp = -np.inf

    def indices(self, points):
        return np.floor((np.asarray(points)-self.lo)/self.res).astype(int)

    def update(self, ids, state, stamp, prior=False):
        ids = np.asarray(ids, dtype=int).reshape(-1, 3)
        ids = ids[np.all((ids >= 0) & (ids < self.shape), axis=1)]
        keys = np.unique(np.ravel_multi_index(ids.T, self.shape)) if len(ids) else []
        changed = set(np.asarray(keys, dtype=np.int64).tolist())
        if state != OCCUPIED:
            changed.difference_update(self.occupied_keys)
        self.cells.update(dict.fromkeys(changed, (state, stamp)))
        if state == OCCUPIED:
            self.occupied_keys.update(changed)
        self.observed_columns.update(key//self.shape[2] for key in changed)
        if prior:
            self.prior.update(changed)
        changed = list(changed)
        return changed

    def integrate(self, origin, points, hits, stamp, roi=None):
        self.last_integration_stats = dict(ray_samples_considered=0,
            ray_samples_retained=0, updated_voxels=0, retained_fraction=0., region=roi)
        points = np.asarray(points).reshape(-1, 3)
        hits = np.asarray(hits, dtype=bool)
        valid = np.all(np.isfinite(points), axis=1)
        points, hits = points[valid], hits[valid]
        if not len(points):
            return []
        delta, lengths = points-origin, np.linalg.norm(points-origin, axis=1)
        free_ids = []
        considered = retained = 0
        clipping = {}
        if roi is not None:
            from .corridor_mapping import contains, sample_intersections
            samples, clipping = sample_intersections(origin, points, roi, self.res/3)
            considered, retained = clipping['full_trace_samples'], len(samples)
            free_ids.append(self.indices(samples))
        else:
            steps = np.arange(0., max(lengths.max(), self.res)+self.res/3, self.res/3)
            for offset in range(0, len(points), 512):
                fractions = steps[None, :]/np.maximum(lengths[offset:offset+512, None], 1e-9)
                samples = origin+fractions[..., None]*delta[offset:offset+512, None, :]
                samples = samples[fractions < 1.]
                considered += len(samples)
                retained += len(samples)
                free_ids.append(self.indices(samples))
        # Deduplicate across the entire scan, including shared near-sensor
        # cells. Previously every ray batch refreshed the same dictionary cells.
        changed = self.update(np.vstack(free_ids), FREE, stamp)
        hit_points = points[hits]
        if roi is not None:
            hit_points = hit_points[contains(hit_points, roi)]
        changed.extend(self.update(self.indices(hit_points), OCCUPIED, stamp))
        self.last_integration_stats = dict(ray_samples_considered=considered,
            ray_samples_retained=retained, updated_voxels=len(set(changed)),
            retained_fraction=retained/max(1, considered), region=roi, clipping=clipping)
        self.last_observation_stamp = stamp
        return changed

    def restore(self, grid, now):
        if not self.cells:
            return
        # Lookup only the bounded navigation window, not every historical
        # voxel in the farm on every scan. Global history can grow indefinitely.
        local = np.indices(grid.shape).reshape(3, -1).T
        ids = self.indices(grid.centers(local))
        keys = np.ravel_multi_index(ids.T, self.shape)
        values = np.asarray([self.cells.get(int(key), (-1, -np.inf)) for key in keys])
        known = values[:, 0] != -1
        keys, local, values = keys[known], local[known], values[known]
        if not len(keys):
            return
        prior = np.fromiter((int(key) in self.prior for key in keys), dtype=bool)
        live = (values[:, 0] == OCCUPIED) | prior | (now-values[:, 1] <= self.c['free_ttl'])
        grid.state[tuple(local[live].T)] = values[live, 0].astype(np.int8)
        grid.seen[tuple(local[live].T)] = np.where(prior[live], now, values[live, 1])

    def delta(self, keys):
        keys = np.asarray(sorted(set(keys)), dtype=np.int64)
        ids = np.column_stack(np.unravel_index(keys, self.shape)) if len(keys) else np.empty((0, 3), dtype=int)
        states = np.array([self.cells[int(k)][0] for k in keys], dtype=np.int8)
        stamps = np.array([self.cells[int(k)][1] for k in keys])
        priors = np.array([int(k) in self.prior for k in keys])
        return ids, states, stamps, priors

    def apply_delta(self, delta):
        ids, states, stamps, priors = delta
        keys = np.ravel_multi_index(ids.T, self.shape) if len(ids) else []
        self.cells.update(zip(np.asarray(keys).tolist(), zip(states.tolist(), stamps.tolist())))
        self.occupied_keys.update(np.asarray(keys)[states == OCCUPIED].tolist())
        self.observed_columns.update(np.unique(np.asarray(keys)//self.shape[2]).tolist())
        self.prior.update(np.asarray(keys)[priors].tolist())
        if len(stamps):
            self.last_observation_stamp = max(self.last_observation_stamp, float(np.max(stamps)))

    def snapshot(self, mode, stamp, points, truncated, status):
        keys = np.fromiter(self.cells, dtype=np.int64)
        ids = np.column_stack(np.unravel_index(keys, self.shape)) if len(keys) else np.empty((0, 3), dtype=int)
        states = np.array([self.cells[int(k)][0] for k in keys])
        times = np.array([self.cells[int(k)][1] for k in keys])
        occupied, free = ids[states == OCCUPIED], ids[states == FREE]
        total = int(np.prod(self.shape))
        # Unknown is implicit, not millions of duplicated XYZ arrays in HTML.
        # The complete unknown set is exactly the grid minus listed known cells.
        unknown_samples = []
        for key in range(0, total, max(1, int(np.ceil(total/6000)))):
            if key not in self.cells:
                unknown_samples.append(list(map(int, np.unravel_index(key, self.shape))))
        return dict(format_version=2, frame='map', localisation_mode=mode,
            simulation_time=float(stamp), map_observation_time=float(self.last_observation_stamp) if np.isfinite(self.last_observation_stamp) else None,
            resolution=self.res, origin=self.lo.tolist(), shape=list(map(int, self.shape)),
            points=points, point_resolution=.1, point_limit_reached=truncated,
            occupied=occupied.tolist(), free=free.tolist(), unknown=unknown_samples,
            unknown_count=total-len(keys), unknown_encoding='implicit_unlisted_cells',
            known_cell_observation_times=times.tolist(), known_cell_indices=ids.tolist(),
            surveyed_pad_indices=[list(map(int, np.unravel_index(k, self.shape))) for k in sorted(self.prior)],
            observed_volume_fraction=len(keys)/total,
            observed_xy_fraction=len(self.observed_columns)/(self.shape[0]*self.shape[1]),
            free_navigation_ttl_seconds=self.c['free_ttl'], environment_policy=self.c.get('map_environment', 'unspecified'),
            lidar_max_range_m=self.c['lidar_range'], simulation_real_time_factor=self.c.get('simulation_real_time_factor'),
            coverage_roi=self.c.get('coverage_roi'),
            coverage_polygon=self.c.get('coverage_polygon'),
            coverage_region_source=self.c.get('coverage_region_source'),
            mission_kind='mapping', mission_status=status,
            note='Whole-farm historical observations, not navigation certification. Free history may be old; unknown is NOT empty. '
                 'The navigation window independently checks expiry, obstacles, localisation and live LiDAR. '
                 'Unlisted cells are unknown; unknown display is sampled. No LiDAR localisation validation.')


class RollingMapper:
    def __init__(self, config):
        self.config = copy.deepcopy(config)
        self.history = SparseHistory(config)

    def integrate(self, grid, origin, points, hits, stamp, pad, now, centre, roi=None):
        changed = self.history.integrate(origin, points, hits, stamp, roi)
        updated = navigation_window(self.config, centre)
        self.history.restore(updated, now)
        if pad is not None:
            updated.seed_launch_pad(*pad, stamp)
            ids = np.argwhere((updated.state == FREE) & (updated.seen == stamp))
            centres = updated.centers(ids)
            home, measured_range = pad
            ground = home[2]-measured_range
            mask = ((np.abs(centres[:, 0]-home[0]) < 1.4) & (np.abs(centres[:, 1]-home[1]) < 1.4)
                    & (centres[:, 2] > ground+.15) & (centres[:, 2] < self.config['takeoff_altitude']+1.2))
            ids = ids[mask]
            world_ids = self.history.indices(updated.centers(ids))
            changed.extend(self.history.update(world_ids, FREE, stamp, prior=True))
        # Reapply only the fixed previously surveyed pad, never a moving bubble.
        updated.last_observation_stamp = stamp
        updated.version = grid.version+1
        updated.rebuild(now)
        updated.mapping_roi_stats = self.history.last_integration_stats
        return updated, stamp, self.history.delta(changed)


_MAPPER = None


def initialise_rolling_mapper(config):
    global _MAPPER
    _MAPPER = RollingMapper(config)


def integrate_rolling_snapshot(*args):
    return _MAPPER.integrate(*args)


def write_history_snapshot(directory, history, mode, stamp, points, truncated, status, flight_paths=None):
    from .map_log import MapLog
    data = history.snapshot(mode, stamp, points, truncated, status)
    if flight_paths is not None:
        data['flight_paths'] = flight_paths
    MapLog(directory).write(data)
