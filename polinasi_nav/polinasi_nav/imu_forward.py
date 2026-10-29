"""Experimental short-horizon body-IMU propagation after accepted ICP correction.

No map, planner, simulator pose or ground-truth input. This is not LIO-SAM,
FAST-LIO or a full error-state filter. The caller enforces correction freshness.
"""
from collections import deque
import numpy as np
from scipy.spatial.transform import Rotation


class ForwardImuOdometry:
    def __init__(self,capacity=500,accel_noise=.01,gyro_noise=.001,velocity_uncertainty=.03):
        self.history=deque(maxlen=capacity)
        self.last_input=-np.inf;self.correction_stamp=None;self.stamp=None
        self.valid=False;self.reason='waiting_for_accepted_lidar_correction';self.revision=0
        self.accel_noise=accel_noise;self.gyro_noise=gyro_noise
        self.velocity_uncertainty=velocity_uncertainty;self.overflows=0

    def push(self,stamp,gyro,acceleration):
        gyro=np.asarray(gyro,float);acceleration=np.asarray(acceleration,float)
        if (gyro.shape!=(3,) or acceleration.shape!=(3,) or
                not np.all(np.isfinite(np.r_[stamp,gyro,acceleration])) or stamp<=self.last_input):return False
        if len(self.history)==self.history.maxlen:self.overflows+=1
        self.history.append((float(stamp),gyro.copy(),acceleration.copy()));self.last_input=stamp
        self.revision+=1
        if self.correction_stamp is not None and self.valid:self._step(stamp,gyro,acceleration)
        return True

    def _step(self,stamp,gyro,acceleration):
        dt=stamp-self.stamp
        if dt<=0:return
        if dt>.1+1e-9:
            self.valid=False;self.reason='imu_prediction_gap';return
        self.omega=gyro-self.gyro_bias
        self.r=self.r@Rotation.from_rotvec(self.omega*dt).as_matrix()
        world_acceleration=self.r@acceleration-[0.,0.,self.gravity]
        self.p+=self.v*dt+.5*world_acceleration*dt*dt
        self.v+=world_acceleration*dt
        self.stamp=stamp

    def correct(self,stamp,position,rotation,velocity,covariance,gyro_bias,gravity,omega=None):
        values=np.r_[stamp,np.asarray(position).ravel(),np.asarray(rotation).ravel(),np.asarray(velocity).ravel(),
                     np.asarray(covariance).ravel(),np.asarray(gyro_bias).ravel(),gravity]
        if not np.all(np.isfinite(values)) or (self.correction_stamp is not None and stamp<=self.correction_stamp):return False
        self.correction_stamp=self.stamp=float(stamp)
        self.p=np.asarray(position,float).copy();self.r=np.asarray(rotation,float).copy()
        self.v=np.asarray(velocity,float).copy();self.base_covariance=np.asarray(covariance,float).copy()
        self.gyro_bias=np.asarray(gyro_bias,float).copy();self.gravity=float(gravity)
        self.omega=np.zeros(3) if omega is None else np.asarray(omega,float).copy()
        self.valid=True;self.reason='forward_body_imu';self.revision+=1
        self.history=deque((row for row in self.history if row[0]>stamp+1e-9),maxlen=self.history.maxlen)
        for t,gyro,acceleration in self.history:
            self._step(t,gyro,acceleration)
            if not self.valid:break
        return self.valid

    def snapshot(self):
        if self.correction_stamp is None or not self.valid:return None
        horizon=max(0.,self.stamp-self.correction_stamp)
        covariance=self.base_covariance.copy()
        # Conservative diagonal growth for a short propagation interval;
        # not a claim of a full Kalman/ESKF covariance update.
        position_variance=((self.velocity_uncertainty*horizon)**2+
            (.5*self.accel_noise*horizon*horizon)**2+
            (self.gravity*self.gyro_noise*horizon**3/6)**2)
        covariance[:3,:3]+=np.eye(3)*position_variance
        covariance[3:,3:]+=np.eye(3)*(self.gyro_noise*horizon)**2
        return dict(stamp=self.stamp,correction_stamp=self.correction_stamp,horizon=horizon,
            position=self.p.copy(),rotation=self.r.copy(),velocity=self.v.copy(),omega=self.omega.copy(),
            covariance=covariance,revision=self.revision)

    def status(self):
        return dict(enabled=True,valid=self.valid,reason=self.reason,correction_stamp=self.correction_stamp,
            imu_stamp=self.stamp,horizon_seconds=(None if self.stamp is None else self.stamp-self.correction_stamp),
            pending_history=len(self.history),history_overflows=self.overflows,
            covariance_policy='base_pose_plus_short_horizon_conservative_diagonal_noise',
            gravity_mps2=getattr(self,'gravity',None),
            accel_noise_mps2=self.accel_noise,gyro_noise_rad_s=self.gyro_noise,
            velocity_uncertainty_mps=self.velocity_uncertainty)
