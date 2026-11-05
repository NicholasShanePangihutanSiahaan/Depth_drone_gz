"""Experimental strapdown integration; yaw gauge and zero velocity set at rest."""
from collections import deque
import math
import numpy as np
from scipy.spatial.transform import Rotation


class ImuIntegrator:
    def __init__(self, acceleration_unit='g', calibration_seconds=2., min_samples=100,
                 max_dt=.1, gyro_limit=.03, accel_std_limit=.15, gravity=9.80665):
        if acceleration_unit not in ('g', 'm_s2'):
            raise ValueError('acceleration_unit must be g or m_s2; verify the driver')
        if min(calibration_seconds, max_dt, gyro_limit, accel_std_limit, gravity) <= 0 or min_samples < 2:
            raise ValueError('invalid integration settings')
        self.scale = gravity if acceleration_unit == 'g' else 1.
        self.duration, self.minimum, self.max_dt = calibration_seconds, min_samples, max_dt
        self.gyro_limit, self.accel_limit, self.g = gyro_limit, accel_std_limit, gravity
        self.samples = deque(maxlen=10000)
        self.rotation = Rotation.identity()
        self.velocity = np.zeros(3)
        self.bias = np.zeros(3)
        self.last_stamp = self.origin_stamp = None
        self.previous_omega = self.previous_accel = None
        self.state, self.reason = 'CALIBRATING', 'keep_sensor_stationary'

    def invalidate(self, reason):
        self.state, self.reason = 'INVALID', reason

    def update(self, stamp, gyro, acceleration):
        gyro, acceleration = np.asarray(gyro, float), np.asarray(acceleration, float)*self.scale
        if self.state == 'INVALID':
            return self.snapshot(stamp)
        if (gyro.shape != (3,) or acceleration.shape != (3,) or
                not np.all(np.isfinite(np.r_[gyro, acceleration, stamp])) or stamp <= 0):
            self.invalidate('invalid_imu_values_or_stamp')
            return self.snapshot(stamp)
        dt = None if self.last_stamp is None else stamp-self.last_stamp
        if dt is not None and (dt <= 0 or dt > self.max_dt):
            self.invalidate('imu_timestamp_order_or_gap_restart_required')
            return self.snapshot(stamp)
        self.last_stamp = stamp
        if self.state == 'CALIBRATING':
            # A quiet constant velocity is indistinguishable from rest here.
            # Operator must actually hold the sensor stationary at startup.
            if np.linalg.norm(gyro) > self.gyro_limit or abs(np.linalg.norm(acceleration)-self.g) > .5:
                self.samples.clear()
                self.reason = 'motion_or_wrong_acceleration_unit'
                return self.snapshot(stamp)
            self.samples.append((stamp, gyro.copy(), acceleration.copy()))
            if len(self.samples) < self.minimum or stamp-self.samples[0][0] < self.duration:
                return self.snapshot(stamp)
            gyros = np.array([s[1] for s in self.samples])
            accels = np.array([s[2] for s in self.samples])
            if np.max(accels.std(axis=0)) > self.accel_limit:
                self.samples.clear()
                self.reason = 'unstable_calibration_restart_window'
                return self.snapshot(stamp)
            self.bias = gyros.mean(axis=0)
            unit = accels.mean(axis=0)
            unit /= np.linalg.norm(unit)
            axis = np.cross(unit, [0., 0., 1.])
            sine = np.linalg.norm(axis)
            if sine > 1e-10:
                self.rotation = Rotation.from_rotvec(axis/sine*math.atan2(sine, unit[2]))
            elif unit[2] < 0:
                self.rotation = Rotation.from_rotvec([math.pi, 0., 0.])
            # Resolve remaining yaw gauge so the initial projected +X is forward.
            initial_yaw = self.rotation.as_euler('xyz')[2]
            self.rotation = Rotation.from_euler('z', -initial_yaw)*self.rotation
            self.previous_omega = gyro-self.bias
            self.previous_accel = self.rotation.apply(acceleration)-[0., 0., self.g]
            self.origin_stamp = stamp
            self.state, self.reason = 'ESTIMATING', 'imu_only_drift_unbounded'
            self.samples.clear()
            return self.snapshot(stamp)
        omega = gyro-self.bias
        self.rotation = self.rotation*Rotation.from_rotvec((self.previous_omega+omega)*.5*dt)
        world_accel = self.rotation.apply(acceleration)-[0., 0., self.g]
        self.velocity += (self.previous_accel+world_accel)*.5*dt
        self.previous_omega, self.previous_accel = omega, world_accel
        return self.snapshot(stamp)

    def snapshot(self, stamp):
        valid = self.state == 'ESTIMATING'
        return dict(stamp=float(stamp) if stamp is not None and math.isfinite(stamp) else None,
                    state=self.state, reason=self.reason, valid=valid,
                    quaternion_xyzw=self.rotation.as_quat().tolist() if valid else None,
                    rpy_deg=self.rotation.as_euler('xyz', degrees=True).tolist() if valid else None,
                    velocity_mps=self.velocity.tolist() if valid else None,
                    speed_mps=float(np.linalg.norm(self.velocity)) if valid else None,
                    integration_seconds=float(stamp-self.origin_stamp) if valid else None,
                    gyro_bias_rad_s=self.bias.tolist(),
                    reference_frame='lidar_test_initial',
                    velocity_point='IMU_origin_not_lever_arm_corrected',
                    initial_velocity_assumption='operator_stationary_zero',
                    quality='experimental_not_localisation', covariance_available=False)
