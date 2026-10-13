"""Experimental scan-to-map ICP with IMU rotation/translation prediction.

ICP = iterative closest point: align a new laser scan to previous scans.
This is a small simulation reference, not a flight-qualified FAST-LIO replacement.
No function accepts simulator position. The launch origin is a coordinate gauge.
"""
import numpy as np
from collections import deque
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation


def voxel_downsample(points, resolution=0.18):
    points = np.asarray(points)
    if not len(points):
        return points.reshape(0, 3)
    _, ids = np.unique(np.floor(points/resolution).astype(int), axis=0, return_index=True)
    return points[ids]


class LidarImuOdometry:
    def __init__(self, mount, origin=(0., 0., 0.), body_imu=False, bootstrap_duration=0.):
        self.mount_p = np.asarray(mount[:3])
        self.mount_r = Rotation.from_euler('xyz', mount[3:]).as_matrix()
        self.body_imu = body_imu
        self.bootstrap_duration=bootstrap_duration
        self.bootstrap_started=None
        self.bootstrap_complete=bootstrap_duration==0.
        self.bootstrap_stationary=False
        self.normals=None
        self.normal_valid=None
        self.keyframes=[]
        self.last_keyframe_position=None
        self.translation_information=[]
        self.p = np.asarray(origin, float).copy()
        self.r = np.eye(3)
        self.v = np.zeros(3)
        self.reference = None
        self.tree = None
        self.last_imu = None
        self.last_scan = None
        self.last_good = None
        self.gravity_samples = []
        self.gyro_samples=[]
        self.gyro_bias=np.zeros(3)
        self.gravity_magnitude=9.81
        self.omega=np.zeros(3)
        self.initialized = False
        self.ok = False
        self.reason = 'initialising'
        self.rmse = np.inf
        self.point_rmse=np.inf
        self.last_registration={}
        self.accepted_position=None
        self.imu_shock_until=-np.inf
        self.imu_shock_peak=0.
        self.velocity_disagreement_scans=0
        self.previous_velocity_disagreement=None
        self.accepted_positions=deque(maxlen=3)
        self.covariance = np.eye(6)*1.

    def freeze_keyframe(self,points):
        self.keyframes.append((self.p.copy(),points.copy()))
        self.keyframes=[self.keyframes[0],*self.keyframes[1:][-11:]]
        self.last_keyframe_position=self.p.copy()
        combined=voxel_downsample(np.vstack([cloud for _,cloud in self.keyframes]))
        if len(combined)>40000:
            combined=combined[np.linspace(0,len(combined)-1,40000,dtype=int)]
        self.reference=combined
        self.rebuild_reference()

    def rebuild_reference(self):
        self.tree=cKDTree(self.reference)
        if not self.body_imu:return
        _,ids=self.tree.query(self.reference,k=12)
        neighbours=self.reference[ids]
        centred=neighbours-neighbours.mean(axis=1,keepdims=True)
        values,vectors=np.linalg.eigh(np.einsum('nki,nkj->nij',centred,centred))
        self.normals=vectors[:,:,0]
        self.normal_valid=(values[:,1]>1e-5)&(values[:,0]<.15*np.maximum(values[:,1],1e-9))

    def imu(self, gyro_sensor, acceleration_sensor, stamp):
        imu_r = np.eye(3) if self.body_imu else self.mount_r
        gyro = imu_r @ np.asarray(gyro_sensor)
        acceleration = imu_r @ np.asarray(acceleration_sensor)
        if self.last_imu is not None and stamp <= self.last_imu:
            self.ok, self.reason = False, 'imu_timestamp_order'
            return
        if not self.initialized:
            if np.linalg.norm(gyro) < 0.05 and abs(np.linalg.norm(acceleration)-9.81) < 0.2:
                self.gravity_samples.append(acceleration)
                self.gyro_samples.append(gyro)
            else:
                self.gravity_samples = []
                self.gyro_samples=[]
            if len(self.gravity_samples) >= 40:
                gravity = np.mean(self.gravity_samples, axis=0)
                if self.body_imu:
                    self.gravity_magnitude=float(np.linalg.norm(gravity))
                    self.gyro_bias=np.mean(self.gyro_samples,axis=0)
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
                gyro=gyro-self.gyro_bias
                self.omega=gyro.copy()
                self.r = self.r@Rotation.from_rotvec(gyro*dt).as_matrix()
                world_accel = self.r@acceleration-[0., 0., self.gravity_magnitude]
                if self.body_imu and np.linalg.norm(world_accel)>30.:
                    self.imu_shock_until=stamp+.5
                    self.imu_shock_peak=max(self.imu_shock_peak,float(np.linalg.norm(world_accel)))
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
        if not self.bootstrap_complete:
            if not self.bootstrap_stationary:
                self.ok,self.reason=False,'bootstrap_requires_disarmed_stationary_launch'
                return False
            self.bootstrap_started=stamp if self.bootstrap_started is None else self.bootstrap_started
            transformed=body@self.r.T+self.p
            self.reference=voxel_downsample(transformed if self.reference is None else np.vstack((self.reference,transformed)))[-40000:]
            self.v[:]=0.
            if stamp-self.bootstrap_started<self.bootstrap_duration-1e-9:
                self.ok,self.reason=False,'collecting_stationary_full_roll_reference'
                return False
            self.bootstrap_complete=True
            self.freeze_keyframe(self.reference)
            self.ok,self.reason,self.last_good=True,'full_roll_bootstrap',stamp
            self.accepted_position=self.p.copy()
            self.accepted_positions.append((stamp,self.p.copy()))
            self.covariance=np.eye(6)*.02
            return True
        if self.reference is None:
            self.reference = body@self.r.T+self.p
            if self.body_imu:self.freeze_keyframe(self.reference)
            else:self.rebuild_reference()
            self.ok, self.reason = True, 'bootstrap'
            self.last_good = stamp
            self.accepted_position=self.p.copy()
            self.accepted_positions.append((stamp,self.p.copy()))
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
            if self.body_imu:
                normals=self.normals[ids[inliers]]
                good=self.normal_valid[ids[inliers]]
                a,b,normals=a[good],b[good],normals[good]
                self.translation_information=np.linalg.eigvalsh(normals.T@normals/max(1,len(normals))).tolist()
                if len(a)<80:
                    self.ok,self.reason=False,'insufficient_plane_constraints'
                    return False
                residual=np.einsum('ij,ij->i',normals,a-b)
                jac=np.column_stack((np.cross(a-p,normals),normals))
                weights=np.minimum(1.,.08/np.maximum(np.abs(residual),1e-9))
                # IMU orientation prior prevents sparse raster correspondences
                # from tilting gravity and manufacturing translational drift.
                prior=np.eye(6)*np.array([20.,20.,5.,.2,.2,.2])
                orientation=Rotation.from_matrix(r@predicted_r.T).as_rotvec()
                delta=np.linalg.lstsq(np.vstack((jac*weights[:,None],prior)),
                    np.r_[-residual*weights,-prior@np.r_[orientation,p-predicted_p]],rcond=None)[0]
                r=Rotation.from_rotvec(delta[:3]).as_matrix()@r
                p+=delta[3:]
                if np.linalg.norm(delta)<1e-4:break
                continue
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
        distances, ids = self.tree.query(transformed)
        inliers = distances < 0.5
        self.point_rmse = float(np.sqrt(np.mean(distances[inliers]**2)))
        self.rmse=self.point_rmse
        plane_count=0
        if self.body_imu:
            good=inliers & self.normal_valid[ids]
            plane_count=int(np.count_nonzero(good))
            if plane_count<80:
                self.ok,self.reason=False,'insufficient_plane_constraints'
                return False
            errors=np.einsum('ij,ij->i',self.normals[ids[good]],transformed[good]-self.reference[ids[good]])
            self.rmse=float(np.sqrt(np.mean(errors**2)))
        angle_error = np.linalg.norm(Rotation.from_matrix(predicted_r.T@r).as_rotvec())
        shift_error=float(np.linalg.norm(p-predicted_p))
        self.last_registration=dict(point_rmse_m=self.point_rmse,residual_rmse_m=self.rmse,
            metric='point_to_plane' if self.body_imu else 'point_to_point',
            plane_count=plane_count,inlier_fraction=float(np.mean(inliers)),
            translation_correction_m=shift_error,rotation_correction_rad=float(angle_error))
        if (self.body_imu and min(self.translation_information,default=0.)<.003):
            self.ok,self.reason=False,'translation_degeneracy'
            return False
        if self.rmse > (.08 if self.body_imu else .18) or shift_error > 0.5 or angle_error > 0.35:
            self.ok, self.reason = False, 'registration_rejected'
            return False
        if self.body_imu and self.last_good is not None:
            interval=stamp-self.last_good
            if 0<interval<=.5:
                samples=[(t,q) for t,q in self.accepted_positions if 0<stamp-t<=.3]+[(stamp,p)]
                samples=samples[-3:]
                times=np.asarray([t for t,_ in samples]);centred_times=times-times.mean()
                positions=np.asarray([q for _,q in samples])
                measured_velocity=np.sum(centred_times[:,None]*positions,axis=0)/max(float(centred_times@centred_times),1e-9)
                disagreement_vector=measured_velocity-self.v
                disagreement=float(np.linalg.norm(disagreement_vector))
                previous=self.previous_velocity_disagreement
                coherent=(previous is not None and float(previous@disagreement_vector)>
                          .7*np.linalg.norm(previous)*disagreement)
                self.velocity_disagreement_scans=((self.velocity_disagreement_scans+1 if coherent else 1)
                                                 if disagreement>.3 else 0)
                self.previous_velocity_disagreement=disagreement_vector.copy() if disagreement>.3 else None
                # A contact impulse may fall entirely between IMU samples.
                # Two independently accepted scan displacements can expose it.
                if self.velocity_disagreement_scans>=2:
                    self.imu_shock_until=stamp+.5
                self.last_registration.update(scan_velocity_mps=measured_velocity.tolist(),
                    scan_velocity_window_seconds=float(times[-1]-times[0]),scan_velocity_samples=len(samples),
                    velocity_disagreement_mps=disagreement,velocity_disagreement_scans=self.velocity_disagreement_scans,
                    velocity_disagreement_coherent=bool(coherent))
                if stamp<=self.imu_shock_until and self.accepted_position is not None:
                    # Contact impulses can alias at the IMU sample rate. Use
                    # independent accepted scan displacement to recover velocity;
                    # never assume ground contact or read simulator velocity.
                    self.v=.2*self.v+.8*measured_velocity
                    self.last_registration['velocity_update']='accepted_scan_after_inertial_disagreement'
                else:
                    self.v+=.2*(p-predicted_p)/interval
                    self.last_registration['velocity_update']='scan_prediction_innovation'
                self.last_registration['imu_shock_peak_mps2']=self.imu_shock_peak
        self.accepted_position=p.copy()
        self.accepted_positions.append((stamp,p.copy()))
        self.p, self.r = p, r
        self.covariance = np.eye(6)*max(0.005, self.rmse**2)
        # Keep a bounded recent local map, with spatial culling.
        if self.body_imu:
            if np.linalg.norm(p-self.last_keyframe_position)>=.5:
                self.freeze_keyframe(transformed)
        else:
            combined = np.vstack((self.reference, transformed))
            combined = combined[np.linalg.norm(combined-p, axis=1) < 15.]
            self.reference = voxel_downsample(combined)[-40000:]
            self.rebuild_reference()
        self.ok, self.reason, self.last_good = True, 'tracking', stamp
        return True
