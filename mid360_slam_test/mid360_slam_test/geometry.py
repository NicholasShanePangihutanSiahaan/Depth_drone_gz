"""SE(3) poses: T_a_b maps coordinates in frame b into frame a."""
import numpy as np
from scipy.spatial.transform import Rotation


def pose(position=(0., 0., 0.), quaternion=(0., 0., 0., 1.)):
    t = np.eye(4)
    p, q = np.asarray(position, float), np.asarray(quaternion, float)
    if p.shape != (3,) or q.shape != (4,) or not np.all(np.isfinite(np.r_[p, q])) or np.linalg.norm(q) < 1e-9:
        raise ValueError('invalid pose')
    t[:3, :3], t[:3, 3] = Rotation.from_quat(q).as_matrix(), p
    return t


def inverse(t):
    out = np.eye(4)
    out[:3, :3] = t[:3, :3].T
    out[:3, 3] = -out[:3, :3]@t[:3, 3]
    return out


def transform(t, points):
    return np.asarray(points)@t[:3, :3].T+t[:3, 3]


def angle(t):
    return float(np.degrees(Rotation.from_matrix(t[:3, :3]).magnitude()))


def pack(t):
    return dict(position=t[:3, 3].tolist(), quaternion=Rotation.from_matrix(t[:3, :3]).as_quat().tolist())


def unpack(data):
    return pose(data['position'], data['quaternion'])


def downsample(points, voxel, maximum):
    points = np.asarray(points, float).reshape(-1, 3)
    points = points[np.all(np.isfinite(points), axis=1)]
    if not len(points):
        return points
    _, ids = np.unique(np.floor(points/voxel).astype(np.int64), axis=0, return_index=True)
    points = points[np.sort(ids)]
    if len(points) > maximum:
        points = points[np.linspace(0, len(points)-1, maximum, dtype=int)]
    return points
