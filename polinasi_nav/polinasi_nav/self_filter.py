"""Geometry-aware self returns; no ray behind a removed body hit becomes free."""
import numpy as np
from scipy.spatial.transform import Rotation


def self_mask(body_points,config):
    points=np.asarray(body_points)
    shapes=config.get('self_filter_shapes')
    if not shapes:
        return np.all(np.abs(points)<=np.asarray(config['drone_dimensions'])/2+config['self_filter_padding'],axis=1)
    result=np.zeros(len(points),dtype=bool)
    padding=config.get('self_filter_shape_padding',.008)
    for shape in shapes:
        local=(points-np.asarray(shape['pose'][:3]))@Rotation.from_euler('xyz',shape['pose'][3:]).as_matrix()
        size=shape['size']
        if shape['kind']=='box':
            inside=np.all(np.abs(local)<=np.asarray(size)/2+padding,axis=1)
        elif shape['kind']=='sphere':
            inside=np.linalg.norm(local,axis=1)<=size[0]+padding
        else:
            inside=(np.linalg.norm(local[:,:2],axis=1)<=size[0]+padding)&(np.abs(local[:,2])<=size[1]/2+padding)
        result|=inside
    return result


def body_shadow(origin,directions,config):
    """Conservative ray intersections for the simulation's close-range blind zone.

    Static body boxes are exact; cylinders use enclosing oriented boxes.
    Camera uses its configured swept envelope. The LiDAR's own envelope
    containing the optical origin is excluded, not the drone body.
    No shadow ray is free-space evidence, regardless of endpoint distance.
    """
    origin=np.asarray(origin,float);directions=np.asarray(directions,float)
    blocked=np.zeros(len(directions),dtype=bool)
    for shape in config.get('self_filter_shapes',[]):
        r=Rotation.from_euler('xyz',shape['pose'][3:]).as_matrix()
        o=(origin-np.asarray(shape['pose'][:3]))@r
        d=directions@r
        if shape['kind']=='sphere':
            radius=shape['size'][0]
            if np.linalg.norm(o)<radius:
                continue  # LiDAR instrument housing at its optical origin.
            b=d@o;disc=b*b-(np.dot(o,o)-radius*radius)
            hit=(disc>=0)&((-b+np.sqrt(np.maximum(0,disc)))>0)
        else:
            size=shape['size'] if shape['kind']=='box' else [2*shape['size'][0],2*shape['size'][0],shape['size'][1]]
            extent=np.asarray(size)/2
            if np.all(np.abs(o)<extent):
                raise ValueError('LiDAR optical origin lies inside body geometry: '+shape.get('name','unnamed'))
            parallel=np.abs(d)<1e-10
            safe_d=np.where(parallel,1.,d)
            a=(-extent-o)/safe_d;b=(extent-o)/safe_d
            lo=np.where(parallel,-np.inf,np.minimum(a,b))
            hi=np.where(parallel,np.inf,np.maximum(a,b))
            outside=np.any(parallel&(np.abs(o)>extent),axis=1)
            hit=(np.min(hi,axis=1)>np.maximum(np.max(lo,axis=1),0.))&~outside
        blocked|=hit
    return blocked
