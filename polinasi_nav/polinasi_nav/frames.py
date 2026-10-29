"""Fixed numerical alignment between map ENU and FCU-local ENU.

ArduPilot can reset local height to zero while map height is above ground.
Calibrate once while disarmed/stationary; NEVER continuously absorb drift.
"""
import math
import numpy as np
from scipy.spatial.transform import Rotation


def pose_at(samples,stamp):
    """Interpolate measured position/yaw; never extrapolate a stale FCU pose."""
    if not samples or stamp<samples[0][0] or stamp>samples[-1][0]:return None
    for i,(t,p,yaw) in enumerate(samples):
        if t>=stamp:
            if t==stamp:return np.asarray(p).copy(),float(yaw)
            previous,q,before=samples[i-1]
            fraction=(stamp-previous)/(t-previous)
            angle=math.atan2(math.sin(yaw-before),math.cos(yaw-before))
            return np.asarray(q)+(np.asarray(p)-q)*fraction,before+angle*fraction
    return None


def common_pose_pair(first,second,maximum_gap=.3):
    """Newest common measured time; never extrapolate either pose stream."""
    if not first or not second:return None
    stamp=min(first[-1][0],second[-1][0])
    for samples in (first,second):
        if stamp<samples[0][0]:return None
        if not any(t==stamp for t,_,_ in samples):
            pair=next(((a,b) for a,b in zip(samples,list(samples)[1:]) if a[0]<stamp<b[0]),None)
            if pair is None or pair[1][0]-pair[0][0]>maximum_gap+1e-9:return None
    a,b=pose_at(first,stamp),pose_at(second,stamp)
    return None if a is None or b is None else (stamp,a,b)

ENU_TO_NED = np.array([[0., 1., 0.], [1., 0., 0.], [0., 0., -1.]])
FLU_TO_FRD = np.diag([1., -1., -1.])


def position_at(samples, stamp, maximum_extrapolation=.03):
    """Match measurement times, not frame labels. Bounded linear interpolation.

    Samples contain (timestamp, ENU position, ENU velocity). Never infer a
    position outside the available history except a <=30 ms forward prediction.
    """
    if not samples or not math.isfinite(stamp) or stamp < samples[0][0]:
        return None
    last = samples[-1]
    if stamp >= last[0]:
        dt = stamp-last[0]
        return last[1]+dt*last[2] if dt <= maximum_extrapolation else None
    for first, second in zip(samples, list(samples)[1:]):
        if first[0] <= stamp <= second[0]:
            fraction = (stamp-first[0])/(second[0]-first[0])
            return first[1]+fraction*(second[1]-first[1])
    return None


def transform_pose(position, rotation, translation, basis):
    return np.asarray(basis)@np.asarray(position)+translation, np.asarray(basis)@rotation


def body_velocity(world_velocity, body_to_world):
    return np.asarray(body_to_world).T@np.asarray(world_velocity)


class FCULocalAlignment:
    def __init__(self, map_position, map_yaw, fcu_position, fcu_yaw):
        self.yaw = math.atan2(math.sin(fcu_yaw-map_yaw), math.cos(fcu_yaw-map_yaw))
        self.rotation = Rotation.from_euler('z', self.yaw).as_matrix()
        self.translation = np.asarray(fcu_position)-self.rotation@np.asarray(map_position)

    def to_fcu(self, position):
        return self.rotation@np.asarray(position)+self.translation

    def to_map(self, position):
        return self.rotation.T@(np.asarray(position)-self.translation)

    @property
    def fcu_origin_in_map(self):
        return -self.rotation.T@self.translation
