"""Bounded three-state voxel map. Unknown space is never navigable."""
import numpy as np
from scipy.ndimage import binary_dilation, distance_transform_edt, maximum_filter1d

UNKNOWN, FREE, OCCUPIED = -1, 0, 1


def select_rays(hits, sequence, surface_stride, free_stride):
    groups = (np.flatnonzero(hits), np.flatnonzero(~np.asarray(hits, bool)))
    selected = [ids[sequence % min(stride, len(ids))::stride]
        for ids, stride in zip(groups, (surface_stride, free_stride)) if len(ids)]
    return np.concatenate(selected).astype(int) if selected else np.empty(0, int)


def integrate_snapshot(snapshot, origin, world_points, ray_hits, stamp, pad, now):
    """Picklable worker entry point; creates no ROS node or control publisher."""
    snapshot.integrate(origin, world_points, ray_hits, stamp)
    if pad is not None:
        snapshot.seed_launch_pad(*pad, stamp)
    snapshot.rebuild(now)
    return snapshot, stamp


class VoxelMap:
    def __init__(self, config):
        self.c = config
        self.res = config['resolution']
        self.lo = np.array(config['bounds_min'], dtype=float)
        self.shape = tuple(np.ceil((np.array(config['bounds_max']) - self.lo) / self.res).astype(int))
        self.state = np.full(self.shape, UNKNOWN, dtype=np.int8)
        self.seen = np.full(self.shape, -np.inf)
        self.version = 0
        self.last_observation_stamp = -np.inf
        self.blocked = np.ones(self.shape, dtype=bool)
        self.blocked_actual = np.ones(self.shape, dtype=bool)
        self.distance = np.zeros(self.shape)
        # A horizontal circumscribed cylinder covers the footprint at ANY yaw.
        dimensions = np.asarray(config['drone_dimensions'])
        self.radius = np.linalg.norm(dimensions[:2] / 2) + config['clearance'] + config['tracking_margin']
        self.half_z = dimensions[2] / 2 + config['clearance'] + config['tracking_margin']
        nxy = int(np.ceil(self.radius / self.res)) + 1
        nz = int(np.ceil(self.half_z / self.res)) + 1
        offsets = np.indices((2*nxy+1, 2*nxy+1, 2*nz+1)).transpose(1, 2, 3, 0)
        offsets = offsets - [nxy, nxy, nz]
        # Minkowski expansion includes voxel extent and quantisation at BOTH ends.
        near_xy = np.maximum(np.abs(offsets[..., :2]) - 1, 0) * self.res
        near_z = np.maximum(np.abs(offsets[..., 2]) - 1, 0) * self.res
        self.kernel = (np.linalg.norm(near_xy, axis=-1) <= self.radius) & (near_z <= self.half_z)
        self.actual_kernel = (np.linalg.norm(near_xy, axis=-1) <= self.radius-config['tracking_margin']) & (near_z <= self.half_z-config['tracking_margin'])

    def indices(self, points):
        return np.floor((np.asarray(points) - self.lo) / self.res).astype(int)

    def inside(self, indices):
        return np.all((indices >= 0) & (indices < self.shape), axis=-1)

    def centers(self, indices):
        return self.lo + (np.asarray(indices) + 0.5) * self.res

    def integrate(self, origin, endpoints, hits, stamp):
        """Ray clearing only to a measured hit or an explicitly valid no-return.

        Points behind a hit remain unknown. Occupied voxels are sticky for a
        mission: foliage is never erased by a later ray through a leaf gap.
        NaN and dropped rays provide NO free-space evidence.
        """
        endpoints = np.asarray(endpoints, dtype=float).reshape(-1, 3)
        hits = np.asarray(hits, dtype=bool)
        valid = np.all(np.isfinite(endpoints), axis=1)
        endpoints, hits = endpoints[valid], hits[valid]
        if len(endpoints) == 0:
            return
        delta = endpoints - origin
        lengths = np.linalg.norm(delta, axis=1)
        # Batch ray samples, spacing < half voxel; handle boundary directions.
        steps = np.arange(0, max(lengths.max(), self.res) + self.res/3, self.res/3)
        for offset in range(0, len(endpoints), 512):
            d = delta[offset:offset+512]
            length = lengths[offset:offset+512]
            fractions = steps[None, :] / np.maximum(length[:, None], 1e-9)
            points = np.asarray(origin) + fractions[..., None] * d[:, None, :]
            ids = self.indices(points)
            good = (fractions < 1.0) & self.inside(ids)
            ids = np.unique(ids[good], axis=0)
            key = tuple(ids.T)
            free = self.state[key] != OCCUPIED
            ids = ids[free]
            self.state[tuple(ids.T)] = FREE
            self.seen[tuple(ids.T)] = stamp
        ids = self.indices(endpoints[hits])
        ids = ids[self.inside(ids)]
        self.state[tuple(ids.T)] = OCCUPIED
        self.seen[tuple(ids.T)] = stamp
        self.version += 1
        self.last_observation_stamp = stamp

    def rebuild(self, now):
        stale = (self.state == FREE) & (now - self.seen > self.c['free_ttl'])
        if np.any(stale):
            self.state[stale] = UNKNOWN
            self.version += 1
        self.blocked = self.inflate(self.state != FREE, self.kernel)
        self.blocked_actual = self.inflate(self.state != FREE, self.actual_kernel)
        # Conservative distance to occupied voxel surfaces (unknown checked separately).
        self.distance = distance_transform_edt(self.state != OCCUPIED) * self.res - np.sqrt(3)*self.res
        # Distance to the inflated occupied/unknown boundary for adaptive speed.
        self.free_distance = distance_transform_edt(~self.blocked)*self.res-np.sqrt(3)*self.res

    def expire(self, now):
        """Invalidate aged free cells without recomputing occupied distances.

        The live survey callback uses this while full rebuilds run elsewhere.
        Inflation of newly unknown cells is ORed into the existing blocked grid.
        """
        stale = (self.state == FREE) & (now-self.seen > self.c['free_ttl'])
        if np.any(stale):
            self.state[stale] = UNKNOWN
            self.version += 1
            self.blocked |= self.inflate(stale, self.kernel)
            self.blocked_actual |= self.inflate(stale, self.actual_kernel)
            self.free_distance = distance_transform_edt(~self.blocked)*self.res-np.sqrt(3)*self.res

    @staticmethod
    def inflate(source, kernel):
        # Exact cylindrical kernel: XY disk times contiguous Z interval.
        # Identical inflation/border behaviour, with less fine-grid work.
        z = kernel.shape[2]//2
        xy = binary_dilation(source, structure=kernel[:, :, z:z+1], border_value=1)
        height = int(np.any(kernel, axis=(0, 1)).sum())
        return maximum_filter1d(xy, size=height, axis=2, mode='constant', cval=1)

    def safe(self, point, actual=False):
        i = self.indices(point)
        blocked = self.blocked_actual if actual else self.blocked
        return bool(self.inside(i) and not blocked[tuple(i)])

    def line_safe(self, start, end, actual=False):
        # Exact grid-plane crossings: fixed-spacing samples can miss a thin
        # intersection with a blocked corner, even with very small spacing.
        start, end = np.asarray(start), np.asarray(end)
        delta = end-start
        crossings = [np.array([0., 1.])]
        for axis in range(3):
            if abs(delta[axis]) < 1e-12:
                continue
            lower = int(np.floor((min(start[axis], end[axis])-self.lo[axis])/self.res))
            upper = int(np.ceil((max(start[axis], end[axis])-self.lo[axis])/self.res))
            planes = self.lo[axis]+np.arange(lower, upper+1)*self.res
            fractions = (planes-start[axis])/delta[axis]
            crossings.append(fractions[(fractions > 0.) & (fractions < 1.)])
        planes = np.unique(np.concatenate(crossings))
        times = np.concatenate((planes, (planes[:-1]+planes[1:])/2))
        points = start+times[:, None]*delta
        # Supercover: a line exactly touching a voxel face/edge also checks
        # its neighbouring cells. This removes zero-clearance corner shortcuts.
        low = self.indices(points-1e-9)
        high = self.indices(points+1e-9)
        ids = np.concatenate([np.where(bits, high, low) for bits in
                              ((0,0,0), (0,0,1), (0,1,0), (0,1,1),
                               (1,0,0), (1,0,1), (1,1,0), (1,1,1))])
        blocked = self.blocked_actual if actual else self.blocked
        return bool(np.all(self.inside(ids)) and not np.any(blocked[tuple(ids.T)]))

    def visible(self, start, end):
        """Visibility is a thin optical ray; unknown also prevents confirmation."""
        count = max(2, int(np.ceil(np.linalg.norm(np.asarray(end)-start)/(self.res/4))))
        points = np.linspace(start, end, count)
        ids = self.indices(points)
        return bool(np.all(self.inside(ids)) and np.all(self.state[tuple(ids.T)] == FREE))

    def corridor_blockers(self, start, end):
        """Diagnostic swept nominal corridor, not a replacement for A*.

        Include the same full body/margin/voxel extent as flight checks. Cells
        outside this volume cannot by themselves justify an exploration move.
        No occupancy or free-space evidence is modified here.
        """
        start, end = np.asarray(start), np.asarray(end)
        count = max(2, int(np.ceil(np.linalg.norm(end-start)/(self.res/3)))+1)
        indices = self.indices(np.linspace(start, end, count))
        inside = self.inside(indices)
        mask = np.zeros(self.shape, dtype=bool)
        mask[tuple(indices[inside].T)] = True
        # Dilation here selects a volume; outside is NOT an occupied source.
        volume = binary_dilation(mask, structure=self.kernel, border_value=0)
        unknown = np.argwhere(volume & (self.state == UNKNOWN))
        occupied = np.argwhere(volume & (self.state == OCCUPIED))
        return dict(unknown=unknown, occupied=occupied,
                    outside=bool(not np.all(inside)))

    def clearance(self, point):
        i = self.indices(point)
        if not self.inside(i):
            return 0.0
        return float(self.distance[tuple(i)] - max(self.radius-self.c['clearance'], self.half_z-self.c['clearance']))

    def seed_launch_pad(self, home, range_value, stamp):
        """Explicit small surveyed launch-pad prior, never an entire free map.

        Used only after a valid downward range reading; assumes a surveyed flat
        2.8 m square pad clear to takeoff altitude. See simulation limitations.
        """
        ids = np.indices(self.shape).reshape(3, -1).T
        p = self.centers(ids)
        ground = home[2] - range_value
        good = ((np.abs(p[:, 0]-home[0]) < 1.4) & (np.abs(p[:, 1]-home[1]) < 1.4)
                & (p[:, 2] > ground+0.15) & (p[:, 2] < self.c['takeoff_altitude']+1.2))
        free_ids = ids[good]
        key = tuple(free_ids.T)
        unchanged = self.state[key] != OCCUPIED
        free_ids = free_ids[unchanged]
        self.state[tuple(free_ids.T)] = FREE
        self.seen[tuple(free_ids.T)] = stamp
        self.version += 1
