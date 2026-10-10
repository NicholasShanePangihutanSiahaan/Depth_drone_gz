"""Gazebo-only decoding of explicit +infinity (no hit within sensor range).

This does not infer free space from arbitrary NaN/dropped Livox measurements.
Geometry/order must match the configured organized 360 x 32 GPU scan.
"""
import numpy as np


def directions():
    azimuth, elevation = np.meshgrid(np.linspace(-np.pi, 3.12413936107, 360),
                                    np.linspace(-.12217304764, .90757121104, 32))
    return np.column_stack((np.cos(elevation).ravel()*np.cos(azimuth).ravel(),
                           np.cos(elevation).ravel()*np.sin(azimuth).ravel(),
                           np.sin(elevation).ravel()))


def normalize(points, rings, maximum):
    points = np.asarray(points, float)
    expected = directions()
    if points.shape != expected.shape or not np.isfinite(maximum) or maximum <= 0:
        raise ValueError('simulated ray shape/range mismatch')
    if not np.array_equal(np.asarray(rings), np.repeat(np.arange(32), 360)):
        raise ValueError('simulated ray ring/order mismatch')
    finite = np.all(np.isfinite(points), axis=1)
    ranges = np.linalg.norm(points[finite], axis=1)
    reliable = finite.copy()
    reliable[finite] = ranges > .1
    if np.count_nonzero(reliable) < 16:
        raise ValueError('not enough finite rays to verify simulated scan geometry')
    unit = points[reliable]/np.linalg.norm(points[reliable], axis=1)[:, None]
    if np.max(np.linalg.norm(unit-expected[reliable], axis=1)) > .003:
        raise ValueError('simulated ray angles do not match organized geometry')
    # Gazebo multiplies +/- infinite DEPTH by the ray direction. Only an
    # all-infinite tuple with signs consistent with POSITIVE depth is a valid
    # no-return. Negative depth and NaN remain invalid, never cleared.
    no_return = np.all(np.isinf(points), axis=1) & np.all(np.sign(points) == np.sign(expected), axis=1)
    result = points.copy()
    result[no_return] = expected[no_return]*maximum
    return result, dict(total_rays=len(points), finite_hits=int(finite.sum()),
                        explicit_no_return=int(no_return.sum()),
                        invalid_rays=int((~finite & ~no_return).sum()),
                        source='verified_organized_gazebo_gpu_lidar')
