import json
import numpy as np
import pytest
from polinasi_nav.config import load_config
from polinasi_nav.mapping import VoxelMap,FREE,OCCUPIED,UNKNOWN
from polinasi_nav.map_trust import LocalMapTrust
from polinasi_nav.map_transport import encode_window,decode_window


def fixture():
    c=load_config()
    c.update(bounds_min=[-3.,-3.,-1.],bounds_max=[3.,3.,5.],resolution=.2,
        map_trust_max_age=1.5,map_trust_ascent_max_age=.8,
        map_trust_stable_seconds=4.,map_trust_revisit_seconds=2.5,
        map_trust_max_speed=.3,tree_ascent_observation_guard=.5)
    g=VoxelMap(c);g.state[:]=FREE;g.observed[:]=True
    t=LocalMapTrust(c)
    return c,g,t


def observe(g,t,stamps=(1.,3.,5.)):
    for stamp in stamps:
        g.last_observation_stamp=stamp
        t.observe(g,[0.,0.,2.])


def assess(g,t,now=5.6,base=.4,**kw):
    return t.assess(g,now,np.array([0.,0.,2.]),np.zeros(3),np.zeros(3),
        np.array([0.,0.,2.]),base,kw.pop('health',True),kw.pop('quality',True),**kw)


def test_stable_measured_corridor_extends_and_expires_with_hard_cap():
    _,g,t=fixture();observe(g,t)
    assert assess(g,t,ascending=True)
    assert t.diagnostics['extended'] and t.diagnostics['stable_fraction']==1.
    assert t.diagnostics['effective_limit_seconds']==.8
    assert not assess(g,t,now=5.81,ascending=True)
    assert assess(g,t,now=6.2,base=1.)
    assert not assess(g,t,now=6.51,base=1.)
    json.dumps(t.diagnostics,allow_nan=False)


def test_unchanged_history_without_ray_reobservations_is_not_stability():
    _,g,t=fixture();g.observed[:]=False;observe(g,t)
    assert not assess(g,t,ascending=True)
    assert t.diagnostics['stable_fraction']==0.
    assert not t.records


@pytest.mark.parametrize('state',[UNKNOWN,OCCUPIED])
def test_new_obstacle_or_unknown_revokes_immediately(state):
    _,g,t=fixture();observe(g,t)
    assert assess(g,t,ascending=True)
    g.state[tuple(g.indices([.4,0.,2.]))]=state
    assert not assess(g,t,ascending=True)
    assert t.diagnostics['reason']=='corridor_unknown_or_occupied'


def test_single_scan_duplicates_and_blind_interval_cannot_build_stability():
    _,g,t=fixture();observe(g,t,(1.,1.,1.,1.))
    assert not assess(g,t,now=1.6,ascending=True)
    observe(g,t,(5.,7.))  # Long unseen interval reset the evidence.
    assert not assess(g,t,now=7.6,ascending=True)


@pytest.mark.parametrize('flag',['health','quality'])
def test_dropout_or_uncertain_pose_clears_trust_not_only_extension(flag):
    _,g,t=fixture();observe(g,t)
    assert not assess(g,t,ascending=True,**{flag:False})
    assert not t.records
    assert not assess(g,t,ascending=True)


def test_revisit_expiry_and_measured_overspeed_disable_extension():
    _,g,t=fixture();observe(g,t)
    assert not assess(g,t,ascending=True,actual_v=[.4,0.,0.])
    assert t.diagnostics['reason']=='speed_limit_for_extension'
    assert not assess(g,t,now=8.,base=1.)


def test_spatial_change_outside_corridor_does_not_revoke_local_stability():
    _,g,t=fixture();observe(g,t)
    g.state[tuple(g.indices([2.5,2.5,2.]))]=OCCUPIED
    assert assess(g,t,ascending=True)


def test_observation_bits_roundtrip_are_not_restamped():
    _,g,_=fixture();g.observed[:]=False;g.observed[2,3,4]=True
    g.last_observation_stamp=5.;g.rebuild(5.)
    restored,_=decode_window(encode_window(g,{}),g)
    np.testing.assert_array_equal(restored.observed,g.observed)
    assert restored.last_observation_stamp==5.


def test_window_shift_uses_world_keys_and_new_area_is_untrusted():
    _,g,t=fixture();observe(g,t)
    previous=t.records.copy()
    g.lo+=np.array([.2,0.,0.]);g.last_observation_stamp=5.3
    t.observe(g,[0.,0.,2.])
    assert any(k in t.records and t.records[k][1]==v[1] for k,v in previous.items())
    assert not t.assess(g,5.9,np.array([1.8,0.,2.]),np.zeros(3),np.zeros(3),
        np.array([1.8,0.,2.]),.4,True,True,True)


@pytest.mark.parametrize('state',[OCCUPIED,UNKNOWN])
def test_noise_or_state_flips_reset_evidence_instead_of_claiming_motion(state):
    _,g,t=fixture();observe(g,t)
    index=tuple(g.indices([.4,0.,2.]))
    g.state[index]=state;observe(g,t,(5.3,))
    g.state[index]=FREE;observe(g,t,(5.6,))
    assert not assess(g,t,now=6.2,ascending=True)
    assert t.diagnostics['reason']=='insufficient_reobservations'


def test_enabled_controller_uses_one_policy_for_survey_and_ascent():
    from polinasi_nav.control import MissionController
    c,g,_=fixture();c.update(adaptive_map_age=True,tree_ascent_max_map_age=.4,map_timeout=1.)
    ctl=MissionController(c,g)
    ctl.command=np.array([0.,0.,2.]);ctl.localisation_quality_ok=True
    ctl.health.localisation_ok=True
    observe(g,ctl.map_trust)
    ctl.health.cloud_stamp=ctl.health.pose_stamp=5.6
    assert ctl.map_age_safe(5.6,ctl.command,np.zeros(3),ascending=True)
    assert ctl.map_trust.diagnostics['extended']
    ctl.health.cloud_stamp=ctl.health.pose_stamp=6.2
    assert ctl.map_age_safe(6.2,ctl.command,np.zeros(3))
    assert ctl.map_trust.diagnostics['extended']
    ctl.health.cloud_stamp=5.
    assert not ctl.map_age_safe(6.2,ctl.command,np.zeros(3))


def test_launch_prior_is_not_ray_evidence():
    from polinasi_nav.coverage_mapping import RollingMapper,rolling_input,navigation_window
    c,_,_=fixture();c.update(navigation_window_size=[6.,6.,6.])
    mapper=RollingMapper(c);g=navigation_window(c,[0.,0.,2.])
    result,_,_=mapper.integrate(rolling_input(g),np.array([0.,0.,.2]),
        np.empty((0,3)),np.empty(0,bool),1.,(np.array([0.,0.,.2]),.2),1.,[0.,0.,2.])
    assert np.any(result.state==FREE)
    assert not np.any(result.observed)
