"""Bounded pinhole projection. Invalid depth never supplies free rays."""
import math
import numpy as np


def project_depth(data, width, height, step, encoding, bigendian, k,
                  minimum=.3, maximum=8., stride=8, budget=800, sequence=0):
    if encoding not in ('32FC1', '16UC1'):
        raise ValueError('unsupported depth encoding')
    dtype = np.dtype(('>' if bigendian else '<')+('f4' if encoding=='32FC1' else 'u2'))
    if width<1 or height<1 or step<width*dtype.itemsize or len(data)<height*step:
        raise ValueError('invalid depth image layout')
    k = np.asarray(k,float).reshape(3,3)
    if not np.all(np.isfinite(k)) or k[0,0]<=0 or k[1,1]<=0:
        raise ValueError('invalid camera intrinsics')
    if stride<1 or budget<1 or not 0<minimum<maximum:
        raise ValueError('invalid depth sampling limits')
    image = np.ndarray((height,width), dtype=dtype, buffer=data, strides=(step,dtype.itemsize))
    v,u = np.meshgrid(np.arange((sequence//stride)%stride,height,stride),
                      np.arange(sequence%stride,width,stride),indexing='ij')
    z = image[v,u].astype(float).ravel()*(.001 if encoding=='16UC1' else 1.)
    u,v = u.ravel(),v.ravel()
    valid = np.isfinite(z)&(z>=minimum)&(z<maximum-.01)
    rejected = int(np.count_nonzero(~valid))
    z,u,v = z[valid],u[valid],v[valid]
    # Evenly distributed budget, not a prefix concentrated in one image corner.
    if len(z)>budget:
        ids = np.linspace(0,len(z)-1,budget,dtype=int)
        z,u,v = z[ids],u[ids],v[ids]
    points = np.column_stack(((u-k[0,2])*z/k[0,0],(v-k[1,2])*z/k[1,1],z))
    return points, dict(sampled_pixels=int(len(valid)), invalid_or_out_of_range=rejected,
                        projected_rays=int(len(points)), invalid_policy='unknown_not_free',
                        source='simulated_rgbd_not_ZED_stereo')


def approach_camera_target(position, hint, depression_degrees=25.):
    """World target ahead and below; the dynamic mount solves body-relative aim."""
    p,h = np.asarray(position,float),np.asarray(hint,float)
    distance = max(float(np.linalg.norm(h[:2]-p[:2])),1.)
    return np.array([h[0],h[1],p[2]-distance*math.tan(math.radians(depression_degrees))])
