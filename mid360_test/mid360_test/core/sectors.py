"""Distances in sensor axes, never free-space inference from missing returns."""
import math
import numpy as np

DIRECTIONS = {'front': 0., 'right': -90., 'left': 90., 'back': 180.}


def measure(points, half_width=5., elevation_min=-5., elevation_max=5.,
            minimum=.2, maximum=70., tags=None, reject_tag_mask=0):
    if not (0 < half_width < 45 and -90 <= elevation_min < elevation_max <= 90
            and 0 < minimum < maximum and math.isfinite(maximum)):
        raise ValueError('invalid sector/range limits')
    p = np.asarray(points, dtype=float).reshape(-1, 3)
    ranges = np.linalg.norm(p, axis=1)
    valid = np.all(np.isfinite(p), axis=1) & (ranges >= minimum) & (ranges <= maximum)
    if tags is not None:
        tags = np.asarray(tags, dtype=np.uint8)
        if tags.shape != (len(p),):
            raise ValueError('point/tag count mismatch')
        valid &= (tags & reject_tag_mask) == 0
    elif reject_tag_mask:
        raise ValueError('tag filtering requested but tag field missing')
    finite_p = np.where(np.isfinite(p), p, 0.)
    azimuth = np.degrees(np.arctan2(finite_p[:, 1], finite_p[:, 0]))
    elevation = np.degrees(np.arctan2(finite_p[:, 2], np.hypot(finite_p[:, 0], finite_p[:, 1])))
    valid &= (elevation >= elevation_min) & (elevation <= elevation_max)
    result = {}
    for name, centre in DIRECTIONS.items():
        difference = (azimuth-centre+180.) % 360.-180.
        selected = valid & (np.abs(difference) <= half_width+1e-10)
        r = ranges[selected]
        result[name] = dict(distance_cm=float(r.min()*100.) if len(r) else None,
                            points=int(len(r)),
                            p10_cm=float(np.percentile(r, 10)*100.) if len(r) else None)
    return dict(sectors=result, missing=[k for k, v in result.items() if not v['points']],
                total_points=len(p), eligible_points=int(valid.sum()),
                nonfinite_points=int((~np.all(np.isfinite(p), axis=1)).sum()),
                zero_returns=int((np.all(p == 0, axis=1)).sum()),
                sector_half_width_deg=half_width,
                elevation_deg=[elevation_min, elevation_max], range_m=[minimum, maximum],
                distance_definition='Euclidean_from_sensor_origin', deskewed=False)
