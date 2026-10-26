"""Broad-phase and fixed shadow reuse must match exact self-return geometry."""
import copy
import numpy as np
import pytest
from scipy.spatial.transform import Rotation
from polinasi_nav import self_filter


def brute(points,c):
    result=np.zeros(len(points),bool);padding=c.get('self_filter_shape_padding',.008)
    for shape in c['self_filter_shapes']:
        local=(points-np.array(shape['pose'][:3]))@Rotation.from_euler('xyz',shape['pose'][3:]).as_matrix()
        size=shape['size']
        if shape['kind']=='box':inside=np.all(np.abs(local)<=np.asarray(size)/2+padding,axis=1)
        elif shape['kind']=='sphere':inside=np.linalg.norm(local,axis=1)<=size[0]+padding
        else:inside=(np.linalg.norm(local[:,:2],axis=1)<=size[0]+padding)&(np.abs(local[:,2])<=size[1]/2+padding)
        result|=inside
    return result


@pytest.mark.parametrize('kind,size',[('box',[.2,.3,.1]),('sphere',[.15]),('cylinder',[.12,.3])])
@pytest.mark.parametrize('rpy',[[0,0,0],[.4,.8,1.2],[1.5,-.6,.2]])
def test_self_broadphase_preserves_near_boundary_and_far_point_classification(kind,size,rpy):
    shape=dict(kind=kind,pose=[.1,-.2,.3,*rpy],size=size)
    c=dict(self_filter_shapes=[shape],self_filter_shape_padding=.008)
    rng=np.random.default_rng(42);points=rng.uniform(-2,2,(4000,3))
    boundary=np.array([[.108,.158,.058],[.158,0,0],[0,0,.158],[-.128,0,.158]])
    local=np.vstack((boundary,boundary+1e-10,boundary-1e-10))
    points=np.vstack((points,local@Rotation.from_euler('xyz',rpy).as_matrix().T+shape['pose'][:3]))
    np.testing.assert_array_equal(self_filter.self_mask(points,c),brute(points,c))
    changed=copy.deepcopy(c);changed['self_filter_shape_padding']=.1;changed['self_filter_shapes'][0]['pose'][0]+=.4
    np.testing.assert_array_equal(self_filter.self_mask(points,changed),brute(points,changed))


def test_fixed_shadow_reuse_is_exact_invalidates_inputs_and_returns_independent_arrays(monkeypatch):
    self_filter._fixed_body_shadow.cache_clear()
    c=dict(lidar_fixed=True,self_filter_shapes=[dict(kind='box',name='cache-fixture',
        pose=[0.,0.,-.3,0.,0.,0.],size=[.2,.2,.2])])
    origin=np.zeros(3);rays=np.array([[0.,0.,-1.],[1.,0.,0.],[0.,1.,0.],[0.,0.,1.]])
    original=self_filter._body_shadow_uncached;calls=[]
    def counted(*args):calls.append(True);return original(*args)
    monkeypatch.setattr(self_filter,'_body_shadow_uncached',counted)
    expected=original(origin,rays,c)
    first=self_filter.body_shadow(origin,rays,c);first[:]=True
    np.testing.assert_array_equal(self_filter.body_shadow(origin,rays,c),expected)
    assert len(calls)==1 and self_filter.body_shadow_cache_status()['hits']==1
    for offset,ray_set,config in (([.4,0,0],rays,c),(origin,rays[::-1],c),
        (origin,rays,dict(c,self_filter_shapes=[dict(c['self_filter_shapes'][0],size=[.4,.2,.2])]))):
        np.testing.assert_array_equal(self_filter.body_shadow(offset,ray_set,config),original(offset,ray_set,config))
    assert len(calls)==4 and self_filter.body_shadow_cache_status()['entries']<=4


def test_simple_self_filter_fallback_and_empty_input_are_preserved():
    c=dict(drone_dimensions=[.4,.4,.2],self_filter_padding=.01)
    assert self_filter.self_mask(np.empty((0,3)),c).shape==(0,)
    np.testing.assert_array_equal(self_filter.self_mask([[0,0,0],[1,0,0]],c),[True,False])
