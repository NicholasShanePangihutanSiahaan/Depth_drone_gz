"""Experimental scan-to-map ICP with IMU rotation/translation prediction.

ICP = iterative closest point: align a new laser scan to previous scans.
This is a small simulation reference, not a flight-qualified FAST-LIO replacement.
No function accepts simulator position. The launch origin is a coordinate gauge.
"""
import numpy as np
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation


def voxel_downsample(points, resolution=0.18):
    points = np.asarray(points)
    if not len(points):
        return points.reshape(0, 3)
    _, ids = np.unique(np.floor(points/resolution).astype(int), axis=0, return_index=True)
    return points[ids]


class LidarImuOdometry:
    def __init__(self, mount, origin=(0., 0., 0.)):
        self.mount_p = np.asarray(mount[:3])
        self.mount_r = Rotation.from_euler('xyz', mount[3:]).as_matrix()
        self.p = np.asarray(origin, float).copy()
        self.r = np.eye(3)
        self.v = np.zeros(3)
        self.reference = None
        self.tree = None
        self.last_imu = None
        self.last_scan = None
        self.last_good = None
        self.gravity_samples = []
        self.initialized = False
        self.ok = False
        self.reason = 'initialising'
        self.rmse = np.inf
        self.covariance = np.eye(6)*1.

    def imu(self, gyro_sensor, acceleration_sensor, stamp):
        gyro = self.mount_r @ np.asarray(gyro_sensor)
        acceleration = self.mount_r @ np.asarray(acceleration_sensor)
        if self.last_imu is not None and stamp <= self.last_imu:
            self.ok, self.reason = False, 'imu_timestamp_order'
            return
        if not self.initialized:
            if np.linalg.norm(gyro) < 0.05 and abs(np.linalg.norm(acceleration)-9.81) < 0.2:
                self.gravity_samples.append(acceleration)
            else:
                self.gravity_samples = []
            if len(self.gravity_samples) >= 40:
                gravity = np.mean(self.gravity_samples, axis=0)
                # Gravity fixes roll/pitch; yaw is the arbitrary launch gauge.
                unit = gravity/np.linalg.norm(gravity)
                axis = np.cross(unit, [0., 0., 1.])
                sine = np.linalg.norm(axis)
                angle = np.arctan2(sine, unit[2])
                self.r = Rotation.from_rotvec(axis/max(sine, 1e-9)*angle).as_matrix()
                self.initialized = True
            self.last_imu = stamp
            return
        if self.last_imu is not None:
            dt = stamp-self.last_imu
            if dt > 0.10:
                self.ok, self.reason = False, 'imu_gap'
            else:
                self.r = self.r@Rotation.from_rotvec(gyro*dt).as_matrix()
                world_accel = self.r@acceleration-[0., 0., 9.81]
                self.p += self.v*dt+0.5*world_accel*dt*dt
                self.v += world_accel*dt
        self.last_imu = stamp

    def scan(self, points_sensor, stamp):
        if not self.initialized or self.last_imu is None or not 0 <= stamp-self.last_imu < 0.1:
            self.ok, self.reason = False, 'imu_not_ready'
            return False
        if self.last_scan is not None and stamp <= self.last_scan:
            self.ok, self.reason = False, 'scan_timestamp_order'
            return False
        body = voxel_downsample(np.asarray(points_sensor)@self.mount_r.T+self.mount_p)
        self.last_scan = stamp
        if len(body) < 80:
            self.ok, self.reason = False, 'insufficient_returns'
            return False
        singular = np.linalg.svd(body-body.mean(axis=0), compute_uv=False)
        if singular[-1]/max(singular[0], 1e-9) < 0.04:
            self.ok, self.reason = False, 'planar_degeneracy'
            return False
        if self.reference is None:
            self.reference = body@self.r.T+self.p
            self.tree = cKDTree(self.reference)
            self.ok, self.reason = True, 'bootstrap'
            self.last_good = stamp
            self.covariance = np.eye(6)*0.02
            return True
        predicted_p, predicted_r = self.p.copy(), self.r.copy()
        p, r = predicted_p.copy(), predicted_r.copy()
        for _ in range(15):
            transformed = body@r.T+p
            distances, ids = self.tree.query(transformed)
            inliers = distances < 0.5
            if inliers.sum() < 80 or inliers.mean() < 0.35:
                self.ok, self.reason = False, 'poor_overlap'
                return False
            a, b = transformed[inliers], self.reference[ids[inliers]]
            ca, cb = a.mean(axis=0), b.mean(axis=0)
            u, _, vt = np.linalg.svd((a-ca).T@(b-cb))
            correction = vt.T@u.T
            if np.linalg.det(correction) < 0:
                vt[-1] *= -1
                correction = vt.T@u.T
            shift = cb-correction@ca
            r, p = correction@r, correction@p+shift
            if np.linalg.norm(shift) < 1e-4 and np.linalg.norm(Rotation.from_matrix(correction).as_rotvec()) < 1e-4:
                break
        transformed = body@r.T+p
        distances, _ = self.tree.query(transformed)
        inliers = distances < 0.5
        self.rmse = float(np.sqrt(np.mean(distances[inliers]**2)))
        angle_error = np.linalg.norm(Rotation.from_matrix(predicted_r.T@r).as_rotvec())
        if self.rmse > 0.18 or np.linalg.norm(p-predicted_p) > 0.5 or angle_error > 0.35:
            self.ok, self.reason = False, 'registration_rejected'
            return False
        self.p, self.r = p, r
        self.covariance = np.eye(6)*max(0.005, self.rmse**2)
        # Keep a bounded recent local map, with spatial culling.
        combined = np.vstack((self.reference, transformed))
        combined = combined[np.linalg.norm(combined-p, axis=1) < 15.]
        self.reference = voxel_downsample(combined)[-40000:]
        self.tree = cKDTree(self.reference)
        self.ok, self.reason, self.last_good = True, 'tracking', stamp
        return True
