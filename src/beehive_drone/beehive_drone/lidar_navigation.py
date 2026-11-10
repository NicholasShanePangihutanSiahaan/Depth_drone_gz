"""PointCloud2 decoding and conservative angular visibility for finite LiDAR returns."""
import math
import numpy as np


def decode_xyz(msg):
    fields = {f.name: f for f in msg.fields}
    if not all(n in fields and fields[n].datatype in (7, 8) for n in ('x', 'y', 'z')):
        raise ValueError('XYZ float fields required')
    if msg.width < 1 or msg.height < 1 or msg.row_step < msg.width*msg.point_step:
        raise ValueError('invalid point cloud dimensions')
    if len(msg.data) < msg.row_step*msg.height:
        raise ValueError('truncated point cloud')
    formats = [('>' if msg.is_bigendian else '<') + ('f4' if fields[n].datatype == 7 else 'f8')
               for n in ('x', 'y', 'z')]
    dtype = np.dtype(dict(names=['x', 'y', 'z'], formats=formats,
                         offsets=[fields[n].offset for n in ('x', 'y', 'z')],
                         itemsize=msg.point_step))
    cloud = np.ndarray((msg.height, msg.width), dtype=dtype, buffer=msg.data,
                       strides=(msg.row_step, msg.point_step))
    return np.column_stack([cloud[n].ravel() for n in ('x', 'y', 'z')])


class LidarView:
    """Finite returns only. Unmeasured angular cells never certify free space.

    Native sensor FLU coordinates. Max 2-degree angular cells; min-pooling
    preserves the closest hit. No-return/NaN points cannot open a corridor.
    """
    def __init__(self, points, origin, rotation, max_range=10., bin_degrees=2.):
        self.origin, self.rotation = np.asarray(origin), np.asarray(rotation)
        self.max_range = max_range
        self.bin = math.radians(bin_degrees)
        self.naz = int(math.ceil(2*math.pi/self.bin))
        self.nel = int(math.ceil(math.pi/self.bin))
        ranges = np.linalg.norm(points, axis=1)
        valid = np.all(np.isfinite(points), axis=1) & (ranges >= .2) & (ranges <= max_range)
        self.local = points[valid]
        self.ranges = np.full((self.naz, self.nel), np.nan)
        if len(self.local):
            az, el, r = self.coordinates(self.local)
            flat = np.full(self.naz*self.nel, np.inf)
            np.minimum.at(flat, az*self.nel+el, r)
            self.ranges = flat.reshape(self.naz, self.nel)
            self.ranges[~np.isfinite(self.ranges)] = np.nan

    def coordinates(self, local):
        r = np.linalg.norm(local, axis=1)
        az = ((np.arctan2(local[:, 1], local[:, 0])+math.pi)/self.bin).astype(int)%self.naz
        el = np.clip(((np.arctan2(local[:, 2], np.linalg.norm(local[:, :2], axis=1))
                       + math.pi/2)/self.bin).astype(int), 0, self.nel-1)
        return az, el, r

    def points(self):
        return self.local@self.rotation.T+self.origin

    def ranges_at(self, points):
        az, el, r = self.coordinates((np.asarray(points)-self.origin)@self.rotation)
        return r, self.ranges[az, el]

    def corridor_seen(self, start, end, radius, half_height, extra=.25):
        delta = np.asarray(end)[:2]-np.asarray(start)[:2]
        length = np.linalg.norm(delta)
        if length < 1e-6:
            return True
        direction = delta/length
        side = np.array([-direction[1], direction[0]])
        # Inspect a distant cross-section so the shallow downward MID-360 FOV
        # can observe the lower envelope. Nearby obstacles are also retained
        # in the fused occupancy map. If coverage is absent, use camera fallback.
        length = max(length+radius+extra, 4.5)
        offsets = np.linspace(-radius, radius, max(3, int(2*radius/.05)+1))
        heights = np.linspace(-half_height, half_height, max(3, int(2*half_height/.05)+1))
        probes = np.array([[*(start[:2]+direction*length+side*s), start[2]+z]
                           for s in offsets for z in heights])
        r, measured = self.ranges_at(probes)
        # Cell extent adds conservative angular uncertainty to measured distance.
        return bool(np.all(np.isfinite(measured) & (measured > r+.15)))
