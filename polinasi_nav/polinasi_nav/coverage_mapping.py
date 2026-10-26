"""Sparse whole-farm history plus bounded rolling navigation occupancy.

History is measured/prior data, never simulator ground-truth geometry. Missing
cells are unknown. Historical free cells have a bounded, explicitly configured
lifetime before they may be restored into the collision-checking window.
"""
import copy
import time
from types import SimpleNamespace
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
        self.window_index=None
        self.cells = {}  # flat global index -> (state, last measured timestamp)
        self.assumptions = {}  # global index -> (FOV prior code, derivation stamp); not measured
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
        if self.window_index is not None and changed:
            ids=np.column_stack(np.unravel_index(changed,self.shape))
            self.window_index.update(ids,state,stamp,prior)
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

    def window_lookup(self,grid):
        # Coordinates only: reusable within one update, never cached occupancy.
        local=np.indices(grid.shape).reshape(3,-1).T
        keys=np.ravel_multi_index(self.indices(grid.centers(local)).T,self.shape)
        return local,keys

    def restore(self, grid, now, lookup=None):
        if self.window_index is not None:
            self.window_index.restore(self,grid,now)
            return
        if not self.cells:return
        # Keep lookup bounded to the local window. Structured fromiter avoids
        # a large nested Python list and a second conversion of state/stamp.
        local,keys=self.window_lookup(grid) if lookup is None else lookup
        values=np.fromiter((self.cells.get(key,(-1,-np.inf)) for key in keys),
            dtype=[('state','i1'),('stamp','f8')],count=len(keys))
        known=values['state']!=-1
        keys,local,values=keys[known],local[known],values[known]
        if not len(keys):return
        prior=np.fromiter(map(self.prior.__contains__,keys),dtype=bool,count=len(keys))
        live=(values['state']==OCCUPIED)|prior|(now-values['stamp']<=self.c['free_ttl'])
        grid.state[tuple(local[live].T)]=values['state'][live]
        grid.seen[tuple(local[live].T)]=np.where(prior[live],now,values['stamp'][live])

    def fov_priors(self,grid,origin=None,rotation=None,profile=None,stamp=None,lookup=None):
        if self.window_index is not None:self.window_index.restore_fov(self,grid)
        else:
            _,keys=self.window_lookup(grid) if lookup is None else lookup
            values=np.fromiter((self.assumptions.get(k,(0.,-np.inf))[0] for k in keys),
                               dtype=np.int8,count=len(keys))
            grid.assumed_free[:]=(values.reshape(grid.shape)==1)
            grid.assumed_occupied[:]=(values.reshape(grid.shape)==2)
        if origin is None:return None
        from .fixed_fov import classify
        ids,codes=classify(grid,origin,rotation,profile)
        global_ids=self.indices(grid.centers(ids))
        keys=np.ravel_multi_index(global_ids.T,self.shape) if len(ids) else []
        self.assumptions.update((int(k),(int(code),float(stamp))) for k,code in zip(keys,codes))
        if self.window_index is not None:self.window_index.update_fov(global_ids,codes)
        grid.assumed_free[tuple(ids.T)]=codes==1
        grid.assumed_occupied[tuple(ids.T)]=codes==2
        return global_ids,codes,float(stamp)

    def delta(self, keys):
        keys = np.asarray(sorted(set(keys)), dtype=np.int64)
        ids = np.column_stack(np.unravel_index(keys, self.shape)) if len(keys) else np.empty((0, 3), dtype=int)
        states = np.array([self.cells[int(k)][0] for k in keys], dtype=np.int8)
        stamps = np.array([self.cells[int(k)][1] for k in keys])
        priors = np.array([int(k) in self.prior for k in keys], dtype=bool)
        return ids, states, stamps, priors

    def apply_delta(self, delta):
        ids, states, stamps, priors = delta[:4]
        if len(delta)==5:
            fov_ids,codes,derived_stamp=delta[4]
            fov_keys=np.ravel_multi_index(fov_ids.T,self.shape) if len(fov_ids) else []
            self.assumptions.update((int(k),(int(code),float(derived_stamp))) for k,code in zip(fov_keys,codes))
            if self.window_index is not None:self.window_index.update_fov(fov_ids,codes)
        keys = np.ravel_multi_index(ids.T, self.shape) if len(ids) else []
        self.cells.update(zip(np.asarray(keys).tolist(), zip(states.tolist(), stamps.tolist())))
        self.occupied_keys.update(np.asarray(keys)[states == OCCUPIED].tolist())
        self.observed_columns.update(np.unique(np.asarray(keys)//self.shape[2]).tolist())
        self.prior.update(np.asarray(keys)[priors].tolist())
        if self.window_index is not None:self.window_index.update(ids,states,stamps,priors)
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
        measured_keys=set(map(int,keys))
        assumed_keys=[k for k in self.assumptions if k not in measured_keys]
        assumed_free=[list(map(int,np.unravel_index(k,self.shape))) for k in assumed_keys if self.assumptions[k][0]==1]
        virtual_occupied=[list(map(int,np.unravel_index(k,self.shape))) for k in assumed_keys if self.assumptions[k][0]==2]
        return dict(format_version=2, frame='map', localisation_mode=mode,
            assumed_free=assumed_free,assumed_occupied=virtual_occupied,
            assumption_policy='persistent_user_FOV_prior_not_measured',
            assumption_cell_indices=[list(map(int,np.unravel_index(k,self.shape))) for k in self.assumptions],
            assumption_derivation_times=[v[1] for v in self.assumptions.values()],
            assumption_codes=[v[0] for v in self.assumptions.values()],
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
            measurement_sources=['LiDAR','simulated_RGBD_not_ZED_stereo'] if self.c.get('depth_camera_enabled') else ['LiDAR'],
            coverage_roi=self.c.get('coverage_roi'),
            coverage_polygon=self.c.get('coverage_polygon'),
            coverage_region_source=self.c.get('coverage_region_source'),
            mission_kind='mapping', mission_status=status,
            note='Whole-farm historical observations, not navigation certification. Free history may be old; unknown is NOT empty. '
                 'The navigation window independently checks expiry, obstacles, localisation and live LiDAR. '
                 'Unlisted cells are unknown; unknown display is sampled. No LiDAR localisation validation.')


def rolling_input(grid):
    """Worker owns history; old window arrays are not inputs to reconstruction."""
    return SimpleNamespace(version=int(grid.version),
        depth_integrated_stamp=getattr(grid, 'depth_integrated_stamp', None))


class RollingMapper:
    def __init__(self, config):
        self.config = copy.deepcopy(config)
        self.history = SparseHistory(config)
        self.geometry_cache={} if config.get('mapping_geometry_reuse',False) else None
        if config.get('mapping_block_index',True):
            from .block_window_index import BlockWindowIndex
            self.history.window_index=BlockWindowIndex(self.history.shape)

    def integrate(self, grid, origin, points, hits, stamp, pad, now, centre, roi=None, auxiliary=(), fixed_fov=None):
        started = time.perf_counter()
        changed = self.history.integrate(origin, points, hits, stamp, roi)
        lidar_stats = copy.deepcopy(self.history.last_integration_stats)
        lidar_finished = time.perf_counter()
        depth_stats = []
        for observation in auxiliary:
            depth_origin, depth_points, depth_stamp, depth_roi = observation
            changed.extend(self.history.integrate(depth_origin,depth_points,
                np.ones(len(depth_points),dtype=bool),depth_stamp,depth_roi))
            depth_stats.append(copy.deepcopy(self.history.last_integration_stats))
        depth_finished = time.perf_counter()
        updated = navigation_window(self.config, centre)
        lookup=None if self.history.window_index is not None else self.history.window_lookup(updated)
        self.history.restore(updated,now,lookup=lookup)
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
        fov_delta=None
        if self.config.get('fixed_fov_assumptions'):
            fov_delta=self.history.fov_priors(updated,lookup=lookup,**fixed_fov) if fixed_fov is not None else self.history.fov_priors(updated,lookup=lookup)
        updated.rebuild(now,geometry_cache=self.geometry_cache)
        updated.mapping_roi_stats = lidar_stats
        updated.depth_roi_stats = depth_stats
        updated.depth_integrated_stamp = max((o[2] for o in auxiliary),
            default=getattr(grid,'depth_integrated_stamp',None))
        delta = self.history.delta(changed)
        if fov_delta is not None:delta=(*delta,fov_delta)
        finished = time.perf_counter()
        updated.mapping_worker_stages = dict(lidar_wall_ms=(lidar_finished-started)*1000,
            depth_wall_ms=(depth_finished-lidar_finished)*1000,
            restore_inflate_delta_wall_ms=(finished-depth_finished)*1000,
            total_wall_ms=(finished-started)*1000,
            depth_observation_stamps=[float(o[2]) for o in auxiliary],
            input_policy='rolling_metadata_only',geometry_reuse=updated.geometry_reuse,
            geometry_reuse_enabled=self.geometry_cache is not None,
            block_window_index=self.history.window_index.status() if self.history.window_index is not None else None)
        return updated, stamp, delta


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
