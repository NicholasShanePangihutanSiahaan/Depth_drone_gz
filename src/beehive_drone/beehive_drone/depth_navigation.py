"""ROS-independent depth geometry, obstacle memory, and local path planning.

All planning distances refer to the vehicle centre. Unknown space may occur in
an A* candidate, but the supervisor must observe the next segment before moving.
No target-tree mask is used: a target trunk is still a collision obstacle.
"""

import heapq
import math

import numpy as np


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def rotation(q):
    q = np.asarray(q, dtype=float)
    norm = np.linalg.norm(q)
    if not np.isfinite(norm) or norm < 1e-6:
        raise ValueError('invalid attitude quaternion')
    x, y, z, w = q / norm
    return np.array([
        [1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
        [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
        [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)],
    ])


def decode_depth(data, width, height, step, encoding, bigendian=False):
    formats = {'32FC1': ('f4', 1.0), '16UC1': ('u2', 0.001)}
    if encoding not in formats or width <= 0 or height <= 0:
        raise ValueError('depth must be nonempty 32FC1 metres or 16UC1 mm')
    kind, scale = formats[encoding]
    dtype = np.dtype(('>' if bigendian else '<') + kind)
    if step < width * dtype.itemsize or len(data) < step * height:
        raise ValueError('truncated depth image')
    return np.ndarray((height, width), dtype=dtype, buffer=data,
                      strides=(step, dtype.itemsize)).astype(float) * scale


class DepthView:
    """One image and the camera pose at acquisition; optical XYZ to map XYZ."""

    def __init__(self, depth, intrinsics, origin, camera_rotation,
                 min_range=0.2, max_range=10.0, inf_is_clear=False):
        self.depth = depth
        self.fx, self.fy, self.cx, self.cy = intrinsics
        if min(self.fx, self.fy) <= 0:
            raise ValueError('invalid CameraInfo intrinsics')
        self.origin = np.asarray(origin)
        self.rotation = camera_rotation  # camera FLU -> map
        self.min_range, self.max_range = min_range, max_range
        self.inf_is_clear = inf_is_clear

    def points(self, stride=4):
        v, u = np.mgrid[0:self.depth.shape[0]:stride,
                        0:self.depth.shape[1]:stride]
        d = self.depth[::stride, ::stride]
        good = np.isfinite(d) & (d >= self.min_range) & (d <= self.max_range)
        # Image: right/down/forward; FLU: forward/left/up.
        local = np.column_stack((d[good], -(u[good]-self.cx)*d[good]/self.fx,
                                 -(v[good]-self.cy)*d[good]/self.fy))
        return local @ self.rotation.T + self.origin

    def ranges_at(self, points):
        local = (np.asarray(points) - self.origin) @ self.rotation
        forward = local[:, 0]
        safe = np.maximum(forward, 1e-6)
        u = np.rint(self.cx-self.fx*local[:, 1]/safe).astype(int)
        v = np.rint(self.cy-self.fy*local[:, 2]/safe).astype(int)
        inside = ((forward > self.min_range) & (u >= 2) & (v >= 2)
                  & (u < self.depth.shape[1]-2) & (v < self.depth.shape[0]-2))
        values = np.full(len(points), np.nan)
        indices = np.flatnonzero(inside)
        if len(indices):
            # Conservative patch: a thin branch in any pixel blocks the ray.
            patch = np.stack([self.depth[v[indices]+dv, u[indices]+du]
                              for dv in (-1, 0, 1) for du in (-1, 0, 1)])
            if self.inf_is_clear:
                patch = np.where(np.isposinf(patch), self.max_range, patch)
            patch = np.where((patch >= self.min_range) & np.isfinite(patch),
                             patch, np.nan)
            valid = np.all(np.isfinite(patch), axis=0)
            values[indices[valid]] = np.min(patch[:, valid], axis=0)
        return forward, values

    def corridor_seen(self, start, end, radius, half_height, extra=0.25):
        """Check depth across the swept frontal envelope, including braking room."""
        start, end = np.asarray(start), np.asarray(end)
        delta = end[:2]-start[:2]
        distance = np.linalg.norm(delta)
        if distance < 1e-5:
            return True
        direction = delta/distance
        side = np.array([-direction[1], direction[0]])
        # At very short distance the footprint is outside a front camera's
        # FOV; its prior observations are protected by the obstacle memory.
        length = max(distance+radius+extra, 1.4)
        probes = np.array([
            [*(start[:2] + direction*length + side*offset), start[2]+height]
            for offset in np.linspace(-radius, radius, 7)
            for height in (-half_height, 0.0, half_height)
        ])
        forward, measured = self.ranges_at(probes)
        return bool(np.all(np.isfinite(measured) & (measured > forward)))


class ObstacleMemory:
    """Voxel hits persist through camera turns; only observed free rays clear them."""

    def __init__(self, resolution=0.15, radius=10.0):
        self.resolution, self.radius = resolution, radius
        self.cells = {}
        self.clear_counts = {}

    def update(self, view, position):
        if self.cells:
            keys = list(self.cells)
            points = (np.array(keys, dtype=float)+0.5)*self.resolution
            forward, measured = view.ranges_at(points)
            free = np.isfinite(measured) & (measured > forward+0.35)
            far = np.linalg.norm(points[:, :2]-position[:2], axis=1) > self.radius
            for key, clear, distant in zip(keys, free, far):
                count = self.clear_counts.get(key, 0)+1 if clear else 0
                self.clear_counts[key] = count
                if distant or count >= 3:
                    self.cells.pop(key, None)
                    self.clear_counts.pop(key, None)
        points = view.points()
        keys = np.unique(np.floor(points/self.resolution).astype(int), axis=0)
        for row in keys:
            key = tuple(row)
            self.cells[key] = True
            self.clear_counts[key] = 0

    def slice(self, altitude, half_height):
        if not self.cells:
            return np.empty((0, 2))
        points = (np.array(list(self.cells), dtype=float)+0.5)*self.resolution
        keep = abs(points[:, 2]-altitude) <= half_height+self.resolution
        return points[keep, :2]


class LocalGrid:
    """Inflated occupancy and bounded eight-connected A* without corner cutting."""

    def __init__(self, centre, obstacles, resolution=0.2, radius=7.0,
                 clearance=0.8):
        self.resolution = resolution
        self.size = 2*int(math.ceil(radius/resolution))+1
        self.origin = np.floor(np.asarray(centre)/resolution)*resolution-radius
        self.blocked = np.zeros((self.size, self.size), dtype=bool)
        cells = np.unique(np.rint((obstacles-self.origin)/resolution).astype(int), axis=0)
        inflation = int(math.ceil((clearance+resolution*0.71)/resolution))
        for dx in range(-inflation, inflation+1):
            for dy in range(-inflation, inflation+1):
                if math.hypot(dx, dy)*resolution > clearance+resolution*0.71:
                    continue
                shifted = cells + (dx, dy)
                inside = np.all((shifted >= 0) & (shifted < self.size), axis=1)
                if np.any(inside):
                    self.blocked[shifted[inside, 0], shifted[inside, 1]] = True

    def cell(self, xy):
        return tuple(np.rint((np.asarray(xy)-self.origin)/self.resolution).astype(int))

    def xy(self, cell):
        return self.origin+np.asarray(cell)*self.resolution

    def free(self, cell):
        return (0 <= cell[0] < self.size and 0 <= cell[1] < self.size
                and not self.blocked[cell])

    def segment_free(self, a, b):
        a, b = np.asarray(a), np.asarray(b)
        n = max(2, int(np.linalg.norm(b-a)/(self.resolution*0.3))+1)
        return all(self.free(self.cell(p)) for p in np.linspace(a, b, n))

    def plan(self, start, goal):
        first, last = self.cell(start), self.cell(goal)
        if not self.free(first) or not self.free(last):
            return []
        if self.segment_free(start, goal):
            return [np.asarray(start), np.asarray(goal)]
        queue = [(0.0, first)]
        costs, parent = {first: 0.0}, {}
        visited = set()
        while queue and len(visited) < self.size*self.size:
            _, current = heapq.heappop(queue)
            if current in visited:
                continue
            visited.add(current)
            if current == last:
                path = [np.asarray(goal)]
                while current != first:
                    path.append(self.xy(current))
                    current = parent[current]
                path.append(np.asarray(start))
                return path[::-1]
            for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1),
                           (1, 1), (1, -1), (-1, 1), (-1, -1)):
                nxt = (current[0]+dx, current[1]+dy)
                if not self.free(nxt):
                    continue
                if dx and dy and (not self.free((current[0]+dx, current[1]))
                                  or not self.free((current[0], current[1]+dy))):
                    continue
                cost = costs[current]+math.hypot(dx, dy)
                if cost >= costs.get(nxt, float('inf')):
                    continue
                costs[nxt], parent[nxt] = cost, current
                heuristic = math.hypot(nxt[0]-last[0], nxt[1]-last[1])
                heapq.heappush(queue, (cost+heuristic, nxt))
        return []

    def next_waypoint(self, start, path, lookahead):
        """Shortcut only when the entire inflated segment remains collision free."""
        best = np.asarray(start)
        for p in path[1:]:
            delta = p-start
            distance = np.linalg.norm(delta)
            candidate = start+delta*min(1.0, lookahead/max(distance, 1e-9))
            if not self.segment_free(start, candidate):
                break
            best = candidate
            if distance >= lookahead:
                break
        return best
