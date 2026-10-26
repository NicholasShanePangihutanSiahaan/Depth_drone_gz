"""Geometry-aware self returns; no ray behind a removed body hit becomes free."""
import numpy as np
from functools import lru_cache
from scipy.spatial.transform import Rotation


def _shape_key(config):
    return tuple((shape['kind'],tuple(map(float,shape['pose'])),
                  tuple(map(float,shape['size'])),shape.get('name','unnamed'))
                 for shape in config.get('self_filter_shapes',[]))


@lru_cache(maxsize=32)
def _compiled_self_shapes(key,padding):
    shapes=[];lower=[];upper=[]
    for kind,pose,size,name in key:
        centre=np.asarray(pose[:3]);r=Rotation.from_euler('xyz',pose[3:]).as_matrix()
        half=(np.asarray(size)/2+padding if kind=='box' else
              np.full(3,size[0]+padding) if kind=='sphere' else
              np.array([size[0]+padding,size[0]+padding,size[1]/2+padding]))
        extent=np.abs(r)@half
        lower.append(centre-extent);upper.append(centre+extent)
        shapes.append((kind,centre,r,size))
    # Outward numeric margin applies only to candidate selection, never to
    # the exact shape tests or their physical dimensions.
    return tuple(shapes),np.min(lower,axis=0)-1e-9,np.max(upper,axis=0)+1e-9


def self_mask(body_points,config):
    points=np.asarray(body_points)
    key=_shape_key(config)
    if not key:
        return np.all(np.abs(points)<=np.asarray(config['drone_dimensions'])/2+config['self_filter_padding'],axis=1)
    padding=config.get('self_filter_shape_padding',.008)
    shapes,lower,upper=_compiled_self_shapes(key,float(padding))
    result=np.zeros(len(points),dtype=bool)
    ids=np.flatnonzero(np.all((points>=lower)&(points<=upper),axis=1))
    if not len(ids):return result
    candidates=points[ids]
    for kind,centre,r,size in shapes:
        local=(candidates-centre)@r
        if kind=='box':inside=np.all(np.abs(local)<=np.asarray(size)/2+padding,axis=1)
        elif kind=='sphere':inside=np.linalg.norm(local,axis=1)<=size[0]+padding
        else:inside=(np.linalg.norm(local[:,:2],axis=1)<=size[0]+padding)&(np.abs(local[:,2])<=size[1]/2+padding)
        result[ids]|=inside
    return result


def _body_shadow_uncached(origin,directions,config):
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


@lru_cache(maxsize=4)
def _fixed_body_shadow(origin,direction_bytes,key):
    config={'self_filter_shapes':[dict(kind=kind,pose=pose,size=size,name=name)
        for kind,pose,size,name in key]}
    directions=np.frombuffer(direction_bytes,dtype=np.float64).reshape(-1,3)
    return _body_shadow_uncached(np.asarray(origin),directions,config)


def body_shadow(origin,directions,config):
    # The caller still verifies the original scan-time transform. Only this
    # deterministic self-geometry calculation is reused, never a sensor ray.
    if not config.get('lidar_fixed',False):return _body_shadow_uncached(origin,directions,config)
    origin=np.asarray(origin,dtype=float);directions=np.asarray(directions,dtype=float)
    if not np.all(np.isfinite(origin)) or not np.all(np.isfinite(directions)):
        return _body_shadow_uncached(origin,directions,config)
    return _fixed_body_shadow(tuple(origin),directions.tobytes(),_shape_key(config)).copy()


def body_shadow_cache_status():
    info=_fixed_body_shadow.cache_info()
    return dict(hits=info.hits,misses=info.misses,entries=info.currsize,max_entries=info.maxsize,
        policy='exact_fixed_self_geometry_not_environment_or_measurement_cache')
