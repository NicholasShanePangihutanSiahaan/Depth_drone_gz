"""FLU mount geometry. Positive pitch command points the sensor upward."""
import math
import numpy as np
from scipy.spatial.transform import Rotation

LIMITS = {'lidar_pitch':(-.2,math.radians(70)), 'camera_pitch':(-.5,1.2),
          'camera_yaw':(-math.radians(170),math.radians(170))}

def mount_rotation(yaw=0.,pitch=0.):
    return Rotation.from_euler('z',yaw).as_matrix()@Rotation.from_euler('y',-pitch).as_matrix()

def camera_angles(position,body_rotation,target,camera_mount):
    delta = np.asarray(body_rotation).T@(np.asarray(target)-np.asarray(position))-np.asarray(camera_mount)
    yaw = math.atan2(delta[1],delta[0])
    pitch = math.atan2(delta[2],math.hypot(delta[0],delta[1]))
    return yaw,pitch

def bounded_target(joint,value):
    if not math.isfinite(value):
        raise ValueError('mount target must be finite')
    return float(np.clip(value,*LIMITS[joint]))

def path_heading(velocity,previous):
    velocity = np.asarray(velocity)
    return math.atan2(velocity[1],velocity[0]) if np.linalg.norm(velocity[:2]) > .015 else previous
