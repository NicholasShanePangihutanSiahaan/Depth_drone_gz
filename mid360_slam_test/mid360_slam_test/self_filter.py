"""Measured drone-centred sphere expressed in native LiDAR coordinates.

lidar_coordinates accepts the measured center directly, without pitch rotation.
body_offset retains the original body-axis conversion for older recordings.

a/b follow the level drone/body axes; +X forward, +Y left, +Z up.
T_body_lidar has positive pitch-down Ry(theta), hence body displacement
(-b,0,-a) is expressed in LiDAR using its inverse Ry(-theta).
"""
import math
import numpy as np


class BodySphere:
    def __init__(self, pitch_down_deg=0., a_cm=0., b_cm=0., radius_cm=0., enabled=False,
                 center_mode='body_offset', center_lidar_cm=(0., 0., 0.)):
        values = (pitch_down_deg, a_cm, b_cm, radius_cm)
        if any(isinstance(v, (bool, str)) or not np.isfinite(v) for v in values):
            raise ValueError('sphere parameters must be finite numbers')
        if not -90. <= pitch_down_deg <= 90. or min(a_cm, b_cm, radius_cm) < 0:
            raise ValueError('pitch must be -90..90 deg and a/b/radius nonnegative')
        if not isinstance(enabled, bool):
            raise ValueError('sphere enabled must be boolean')
        if enabled and radius_cm <= 0:
            raise ValueError('enabled sphere requires a measured positive radius')
        self.enabled = enabled
        self.pitch_down_deg = float(pitch_down_deg)
        self.a_cm, self.b_cm, self.radius_cm = map(float, (a_cm, b_cm, radius_cm))
        theta = math.radians(pitch_down_deg)
        a, b = a_cm*.01, b_cm*.01
        self.center = np.array([-b*math.cos(theta)+a*math.sin(theta), 0.,
                                -b*math.sin(theta)-a*math.cos(theta)])
        if center_mode not in ('body_offset', 'lidar_coordinates'):
            raise ValueError('unknown sphere center mode')
        self.center_mode = center_mode
        if center_mode == 'lidar_coordinates':
            center = np.asarray(center_lidar_cm, dtype=float)
            if center.shape != (3,) or not np.all(np.isfinite(center)):
                raise ValueError('center_lidar_cm requires three finite coordinates')
            self.center = center * .01
        self.radius_m = radius_cm*.01

    def inside(self, points):
        points = np.asarray(points, dtype=float).reshape(-1, 3)
        if not self.enabled:
            return np.zeros(len(points), dtype=bool)
        finite = np.all(np.isfinite(points), axis=1)
        out = np.zeros(len(points), dtype=bool)
        delta = points[finite]-self.center
        out[finite] = np.einsum('ij,ij->i', delta, delta) <= self.radius_m**2
        return out

    def metadata(self):
        return dict(enabled=self.enabled, pitch_down_deg=self.pitch_down_deg,
            a_cm=self.a_cm, b_cm=self.b_cm, radius_cm=self.radius_cm,
            center_lidar_m=self.center.tolist(), radius_m=self.radius_m,
            center_mode=self.center_mode,
            offset_measurement_axes=('lidar_native' if self.center_mode == 'lidar_coordinates'
                                     else 'drone_body_X_forward_Y_left_Z_up'),
            rotation=('none; center supplied in lidar frame' if self.center_mode == 'lidar_coordinates'
                      else 'Ry(-pitch_down) @ [-b,0,-a]'),
            removed_points_policy='ignored_not_free_space',
            does_not_level_cloud_or_change_IMU_extrinsics=True)
