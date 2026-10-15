"""User-requested fixed-LiDAR assumptions; NEVER measured free-space evidence.

Angles are in the physical LiDAR frame. Upper hits project a virtual ceiling
z = range * cos(38 degrees), per azimuth. Verified no-return at the boundary
extends the upper free assumption to map bounds. Lower unknown is assumed free
without a range limit. Persistent priors do not bypass live flight health gates.
"""
import numpy as np


def boundary_profile(points, hits):
    points = np.asarray(points, float).reshape(-1, 3)
    hits = np.asarray(hits, bool)
    profile = np.full(360, np.nan)
    if not len(points):
        return profile
    ranges = np.linalg.norm(points, axis=1)
    elevations = np.arctan2(points[:, 2], np.linalg.norm(points[:, :2], axis=1))
    valid = (np.all(np.isfinite(points), axis=1) & (ranges > 0.)
             & (np.abs(elevations-np.deg2rad(52.)) < np.deg2rad(.2)))
    bins = np.zeros(len(points),dtype=int)
    bins[valid] = np.rint(np.rad2deg(np.arctan2(points[valid, 1], points[valid, 0]))).astype(int) % 360
    # If duplicates exist, the nearest finite hit wins over no-return.
    for index in np.flatnonzero(valid):
        ceiling = ranges[index]*np.cos(np.deg2rad(38.)) if hits[index] else np.inf
        azimuth = bins[index]
        if np.isnan(profile[azimuth]) or ceiling < profile[azimuth]:
            profile[azimuth] = ceiling
    return profile


def classify(grid, origin, rotation, profile):
    """Return local indices and prior codes: 1 assumed free, 2 virtual blocked.

    Existing measured occupancy and timestamps are not edited. Missing boundary
    rays create no new upper prior. Prior persistence is managed by history.
    """
    from .mapping import UNKNOWN
    ids = np.argwhere(grid.state == UNKNOWN)
    if not len(ids):
        return ids, np.empty(0, dtype=np.uint8)
    local = (grid.centers(ids)-np.asarray(origin)) @ np.asarray(rotation)
    elevation = np.arctan2(local[:, 2], np.linalg.norm(local[:, :2], axis=1))
    azimuth = np.rint(np.rad2deg(np.arctan2(local[:, 1], local[:, 0]))).astype(int) % 360
    cap = np.asarray(profile)[azimuth]
    lower = elevation < np.deg2rad(-7.)
    upper = (elevation > np.deg2rad(52.)) & ~np.isnan(cap)
    # Virtual ceiling occupies every unknown voxel touching/above its height.
    half_diagonal = np.sqrt(3)*grid.res/2
    blocked = upper & (local[:, 2]+half_diagonal >= cap)
    free = lower | (upper & ~blocked)
    selected = free | blocked
    return ids[selected], np.where(blocked[selected], 2, 1).astype(np.uint8)


def status(grid):
    unknown = grid.state == -1
    return dict(enabled=bool(grid.c.get('fixed_fov_assumptions')),
        policy='persistent_user_FOV_prior_not_sensor_observation',
        lower_degrees=-7., upper_degrees=52., lower_range_limit_m=None,
        upper_hit_rule='virtual_ceiling_z_equals_range_cos_38_degrees_per_azimuth',
        upper_valid_no_return_rule='free_to_map_bounds',
        dropout_policy='retain_prior_without_refreshing_observation_time',
        health_gates_bypassed=False,
        assumed_free_cells=int(np.count_nonzero(grid.assumed_free & unknown)),
        virtual_blocked_cells=int(np.count_nonzero(grid.assumed_occupied & unknown)),
        measured_occupied_preserved=True)
