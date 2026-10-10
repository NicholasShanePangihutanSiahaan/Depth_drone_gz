"""Bounded, mission-directed next-best-view selection in observed free space.

A viewpoint is a drone position, not an unknown cell to fly into. Information
gain predicts first unknown cells visible inside the mounted LiDAR coverage;
only later real measurements may mark those cells free or occupied.
"""
from dataclasses import dataclass
import numpy as np
from scipy.spatial.transform import Rotation
from .mapping import FREE, UNKNOWN, OCCUPIED


@dataclass
class ExplorationView:
    point: np.ndarray
    path: list
    gain: int
    score: float


def visible_unknown(grid, position, yaw=0.):
    """Coarse first-intersection rays with occupied-cell occlusion.

    This predicts potential observations, not actual free-space evidence.
    Unseen cells beyond the first unknown cell on a ray are not counted.
    Grazing occlusions can be missed by this coarse scoring approximation;
    it never clears cells or replaces the exact flight/braking safety gate.
    """
    c = grid.c
    azimuth, elevation = np.meshgrid(np.linspace(-np.pi, np.pi, 40, endpoint=False),
                                    np.deg2rad(np.linspace(-7., 52., 7)))
    directions = np.column_stack((np.cos(elevation).ravel()*np.cos(azimuth).ravel(),
                                 np.cos(elevation).ravel()*np.sin(azimuth).ravel(),
                                 np.sin(elevation).ravel()))
    body_rotation = Rotation.from_euler('z', yaw).as_matrix()
    mount = c['lidar_mount']
    sensor_rotation = body_rotation@Rotation.from_euler('xyz', mount[3:]).as_matrix()
    origin = np.asarray(position)+body_rotation@np.asarray(mount[:3])
    ranges = np.arange(max(c['lidar_min_range'], grid.res/2),
                       min(c['lidar_range'], c['exploration_look_range']), grid.res/2)
    if not len(ranges):
        return np.empty((0, 3), dtype=int)
    points = origin+(directions@sensor_rotation.T)[:, None, :]*ranges[None, :, None]
    ids = grid.indices(points)
    inside = grid.inside(ids)
    states = np.full(inside.shape, OCCUPIED, dtype=np.int8)
    states[inside] = grid.state[tuple(ids[inside].T)]
    blocked = states != FREE
    first = blocked.argmax(axis=1)
    rays = np.flatnonzero(blocked.any(axis=1))
    rays = rays[states[rays, first[rays]] == UNKNOWN]
    return np.unique(ids[rays, first[rays]], axis=0)


def choose_view(grid, current, mission_hint, visited=(), yaw=0., relevant_unknown=None):
    """Choose a local directly reachable view; never weaken unknown inflation.

    Straight observed-free hops keep selection bounded and avoid expensive
    searches inside the controller. Mission planning still uses 3D A*.
    """
    c = grid.c
    current, hint = np.asarray(current), np.asarray(mission_hint)
    if not grid.safe(current):
        return None
    points = grid.centers(np.argwhere(~grid.blocked))
    distance = np.linalg.norm(points-current, axis=1)
    goal_distance = np.linalg.norm(points-hint, axis=1)
    keep = ((distance >= c['exploration_min_move']) & (distance <= c['exploration_radius'])
            & (points[:, 2] >= c['exploration_min_altitude'])
            & (points[:, 2] <= c['exploration_max_altitude'])
            & (goal_distance <= np.linalg.norm(current-hint)+c['exploration_backtrack']))
    points = points[keep]
    for previous in visited:
        points = points[np.linalg.norm(points-previous, axis=1) >= c['exploration_revisit_distance']]
    if not len(points):
        return None
    order = np.argsort(np.linalg.norm(points-hint, axis=1)+0.25*np.linalg.norm(points-current, axis=1))
    candidates = []
    for point in points[order]:
        if candidates and min(np.linalg.norm(point-p) for p in candidates) < c['exploration_revisit_distance']:
            continue
        if not grid.line_safe(current, point):
            continue
        candidates.append(point)
        if len(candidates) >= c['exploration_candidates']:
            break
    best = None
    relevant = None
    if relevant_unknown is not None:
        relevant = np.zeros(grid.shape, dtype=bool)
        relevant[tuple(np.asarray(relevant_unknown, dtype=int).reshape(-1, 3).T)] = True
        already_visible = visible_unknown(grid, current, yaw)
        # Require new route-relevant visibility, not more generic unknown cells.
        relevant[tuple(already_visible.T)] = False
    for point in candidates:
        unknown = visible_unknown(grid, point, yaw)
        if relevant is not None:
            unknown = unknown[relevant[tuple(unknown.T)]]
        minimum = c.get('exploration_min_relevant_gain', 1) if relevant is not None else c['exploration_min_gain']
        if len(unknown) < minimum:
            continue
        cells = grid.centers(unknown)
        gain = np.exp(-np.linalg.norm(cells-hint, axis=1)/c['exploration_look_range']).sum()
        progress = np.linalg.norm(current-hint)-np.linalg.norm(point-hint)
        score = gain+3.*progress-0.5*np.linalg.norm(point-current)
        if best is None or score > best.score:
            best = ExplorationView(point.copy(), [current.copy(), point.copy()], len(unknown), float(score))
    return best
