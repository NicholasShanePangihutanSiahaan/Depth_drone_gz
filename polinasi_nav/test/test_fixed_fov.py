import json
from pathlib import Path
import numpy as np
import pytest
from scipy.spatial.transform import Rotation
from polinasi_nav.config import load_config
from polinasi_nav.mapping import VoxelMap, UNKNOWN, OCCUPIED
from polinasi_nav.coverage_mapping import SparseHistory, navigation_window, RollingMapper, rolling_input
from polinasi_nav.fixed_fov import boundary_profile, classify
from polinasi_nav.map_transport import encode_window,decode_window


def fixture():
    c=load_config()
    c.update(bounds_min=[-5.,-5.,-5.],bounds_max=[5.,5.,7.],resolution=.2,
        navigation_window_size=[10.,10.,12.],fixed_fov_assumptions=True,
        drone_dimensions=[.2,.2,.2],clearance=.05,tracking_margin=.05)
    return c,VoxelMap(c)


def ray(range_=4.,azimuth=0.,elevation=52.):
    az,el=np.deg2rad([azimuth,elevation])
    return range_*np.array([np.cos(el)*np.cos(az),np.cos(el)*np.sin(az),np.sin(el)])


def test_upper_object_projects_to_virtual_ceiling_and_not_horizontal_range():
    c,g=fixture();profile=boundary_profile([ray()],[True])
    assert profile[0]==pytest.approx(4*np.cos(np.deg2rad(38)))
    profile[:]=profile[0]
    # z=2.9 is below the 3.152m virtual ceiling; 3.3 is above it.
    ids,codes=classify(g,[0,0,0],np.eye(3),profile)
    values={tuple(i):code for i,code in zip(ids,codes)}
    assert values[tuple(g.indices([1.3,.1,2.9]))]==1
    assert values[tuple(g.indices([1.3,.1,3.3]))]==2
    assert tuple(g.indices([3.,.1,1.])) not in values  # Inside real FOV remains unknown.


def test_no_return_only_applies_to_its_azimuth_and_invalid_creates_no_upper_free():
    _,g=fixture();profile=boundary_profile([ray(20.,azimuth=np.rad2deg(np.arctan2(.1,1.3))),[np.nan,0,0]],[False,False])
    assert np.isinf(profile[4]) and np.isnan(profile[90])
    ids,codes=classify(g,[0,0,0],np.eye(3),profile)
    values={tuple(i):code for i,code in zip(ids,codes)}
    assert values[tuple(g.indices([1.3,.1,6.5]))]==1
    assert tuple(g.indices([.1,1.3,6.5])) not in values


def test_lower_unknown_unbounded_by_sensor_range_and_ground_leaf_obstacles_survive():
    c,g=fixture();history=SparseHistory(c)
    occupied=g.indices([[3.,3.,-3.],[1.,1.,-1.]])
    g.state[tuple(occupied.T)]=OCCUPIED;g.seen[tuple(occupied.T)]=1.
    history.fov_priors(g,[0,0,0],np.eye(3),np.full(360,np.nan),2.)
    g.rebuild(2.)
    far=tuple(g.indices([-4.,-4.,-4.]));assert g.assumed_free[far]
    assert g.state[far]==UNKNOWN and np.isneginf(g.seen[far])
    assert g.safe([-3.,-3.,-3.])
    assert not np.any(g.assumed_free[tuple(occupied.T)])
    assert np.all(g.state[tuple(occupied.T)]==OCCUPIED)
    assert not g.safe([3.,3.,-3.])


def test_sensor_rotation_is_applied_numerically_without_relabelling_frame():
    _,g=fixture();rotation=Rotation.from_euler('x',np.pi).as_matrix()
    ids,codes=classify(g,[0,0,0],rotation,np.full(360,np.nan))
    values={tuple(i):code for i,code in zip(ids,codes)}
    assert values[tuple(g.indices([0.,0.,4.]))]==1  # Below physical LiDAR after inversion.
    assert tuple(g.indices([0.,0.,-4.])) not in values


def test_prior_survives_dropout_and_window_relocation_without_new_observation_stamp():
    c,g=fixture();history=SparseHistory(c)
    history.fov_priors(g,[0,0,0],np.eye(3),np.full(360,np.nan),2.)
    before=dict(history.assumptions)
    restored=navigation_window(c,[1,0,0]);history.fov_priors(restored)
    restored.rebuild(900.)
    assert restored.safe([-3,-3,-3])
    assert np.isneginf(restored.last_observation_stamp)
    assert before==history.assumptions
    assert all(t==2. for _,t in history.assumptions.values())


def test_new_upper_hit_revokes_old_free_prior_and_measured_obstacle_always_wins():
    c,g=fixture();history=SparseHistory(c)
    history.fov_priors(g,[0,0,0],np.eye(3),boundary_profile([ray(20,azimuth=np.rad2deg(np.arctan2(.1,1.3)))],[False]),1.)
    key=tuple(g.indices([1.3,.1,4.]));assert g.assumed_free[key]
    history.fov_priors(g,[0,0,0],np.eye(3),boundary_profile([ray(4,azimuth=np.rad2deg(np.arctan2(.1,1.3)))],[True]),2.)
    assert not g.assumed_free[key] and g.assumed_occupied[key]
    g.state[key]=OCCUPIED;g.rebuild(2.)
    assert not g.safe(g.centers(key))


def test_worker_delta_transport_and_export_keep_assumptions_separate():
    c,g=fixture();mapper=RollingMapper(c)
    packet=dict(origin=[0,0,0],rotation=np.eye(3),profile=np.full(360,np.nan),stamp=1.)
    updated,stamp,delta=mapper.integrate(rolling_input(g),[0,0,0],np.empty((0,3)),
        np.empty(0,bool),1.,None,1.,[0,0,0],fixed_fov=packet)
    assert updated.assumed_free.any() and np.all(updated.state==UNKNOWN)
    copy=SparseHistory(c);copy.apply_delta(delta)
    assert copy.assumptions==mapper.history.assumptions
    decoded,_=decode_window(encode_window(updated,{}),g)
    np.testing.assert_array_equal(decoded.assumed_free,updated.assumed_free)
    assert not np.shares_memory(decoded.assumed_free,updated.assumed_free)
    data=copy.snapshot('sensor',1.,[],False,{})
    assert data['assumed_free'] and data['free']==[] and data['occupied']==[]
    assert data['assumption_policy']=='persistent_user_FOV_prior_not_measured'
    json.dumps(data,allow_nan=False)


def test_persisting_fov_prior_does_not_disable_dropout_or_pose_health_checks():
    from polinasi_nav.planning import Health
    c,g=fixture();history=SparseHistory(c)
    history.fov_priors(g,[0,0,0],np.eye(3),np.full(360,np.nan),1.)
    g.rebuild(100.)
    assert g.assumed_free.any()
    health=Health(1.,1.,1.,True)
    assert health.reason(100.,c,True)  # Persistent space assumptions are not a fresh sensor.
