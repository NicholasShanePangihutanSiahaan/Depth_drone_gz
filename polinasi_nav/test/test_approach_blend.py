from pathlib import Path
import numpy as np
from polinasi_nav.approach_blend import build_approach_blend
from polinasi_nav.config import load_config
from polinasi_nav.mapping import VoxelMap,FREE,OCCUPIED,UNKNOWN
from polinasi_nav.continuous_tree import ContinuousTreeInspectionController


def fixture():
    c=load_config(Path(__file__).resolve().parents[1]/'config/tree_inspection.json')
    c.update(bounds_min=[-6.,-6.,0.],bounds_max=[6.,6.,6.],
             simulation_continuous_roll=True,continuous_tree_scanning=True,
             rolling_canopy_ascent=True,smooth_approach_ascent=True,
             tree_view_count=6,tree_scan_anchors=24,
             approach_blend_distance=1.8,approach_blend_rise=.6,
             approach_blend_terminal_speed=.08,tree_ascent_speed=.12,
             tree_ascent_observation_guard=.2)
    g=VoxelMap(c);g.state[:]=FREE;g.seen[:]=0.
    g.state[tuple(g.indices([2.,0.,4.5]))]=OCCUPIED
    g.rebuild(0.);g.last_observation_stamp=0.
    return c,g


def test_curve_is_level_to_vertical_with_nonzero_C2_handoff():
    c,g=fixture();tree=dict(id=7,xy=[0.,0.],observed_stem_top=5.)
    t,detail=build_approach_blend(g,[3.,0.,2.],[2.,0.,2.],tree,c)
    assert t is not None,detail
    p,v,a=t.sample(0.)
    np.testing.assert_allclose(p,[3.,0.,2.]);np.testing.assert_allclose(v,0);np.testing.assert_allclose(a,0)
    p,v,a=t.sample(t.duration)
    np.testing.assert_allclose(p,[2.,0.,2.6]);np.testing.assert_allclose(v[:2],0,atol=1e-12)
    assert v[2]>0
    np.testing.assert_allclose(a,0,atol=1e-12)
    points=np.array([t.sample(s)[0] for s in np.linspace(0,t.duration,100)])
    assert np.min(np.linalg.norm(points[:,:2],axis=1))>=2.-1e-10
    assert np.min(np.diff(points[:,2]))>=0
    p2,v2,a2=t.sample(t.duration+.02)
    np.testing.assert_allclose(p2,p+v*.02);np.testing.assert_allclose(v2,v)
    np.testing.assert_allclose(a2,a,atol=1e-10)
    assert t.safe(g)


def test_async_snapshot_adopts_raised_goal_and_complete_curve():
    from concurrent.futures import Future
    from polinasi_nav.survey import prepare_survey_snapshot
    c,g=fixture();ctl=ContinuousTreeInspectionController(c,g,asynchronous=False)
    ctl.state='SURVEY';ctl.tree_phase='APPROACH'
    ctl.active_tree=dict(id=7,xy=[0.,0.],observed_stem_top=5.)
    ctl.command=np.array([3.,0.,2.]);ctl.view_target=np.array([2.,0.,2.])
    clone=ctl._planning_clone()
    clone,ready=prepare_survey_snapshot(clone,0.,ctl.command,None,None)
    assert ready and clone.trajectory.approach_ascent_blend
    ctl.pending_plan=Future();ctl.pending_plan.set_result((clone,ready))
    ctl.executor=object()
    assert ctl._prepare_trajectory(0.,ctl.command)
    assert ctl.trajectory.approach_ascent_blend
    np.testing.assert_allclose(ctl.goal,ctl.view_target)
    assert ctl.goal[2]>2.
    assert ctl.approach_transition['accepted']
    ctl.executor=None


def test_observation_buffer_triggers_brake_but_does_not_forbid_certified_stop():
    from types import SimpleNamespace
    from unittest.mock import patch
    c,g=fixture();ctl=ContinuousTreeInspectionController(c,g,asynchronous=False)
    ctl.tree_phase='ASCEND';ctl.command=np.array([2.,0.,2.])
    ctl.command_v=np.array([0.,0.,.1])
    ctl.ascent_reference=SimpleNamespace(stop=None,reason='')
    with patch('polinasi_nav.planning.brake_safe',return_value=False):
        assert not ctl.final_reference_guard(np.zeros(3))
        ctl.ascent_reference=SimpleNamespace(stop=object(),reason='observation_guard_limit')
        assert ctl.final_reference_guard(np.zeros(3))
    assert ctl.final_reference_guard_diagnostics['actual_and_reference_braking_still_required']
    assert not ctl.final_reference_guard_diagnostics['observation_buffer_required']


def test_unknown_and_occupied_curve_not_rounded_through():
    c,g=fixture();tree=dict(xy=[0.,0.],observed_stem_top=5.)
    for state in (UNKNOWN,OCCUPIED):
        g.state[tuple(g.indices([2.5,0.,2.2]))]=state
        g.rebuild(0.)
        t,detail=build_approach_blend(g,[3.,0.,2.],[2.,0.,2.],tree,c)
        assert t is None


def test_blend_requires_measured_canopy_not_just_a_tree_height_guess():
    c,g=fixture();g.state[:]=FREE;g.rebuild(0.)
    t,detail=build_approach_blend(g,[3.,0.,2.],[2.,0.,2.],dict(xy=[0.,0.],observed_stem_top=5.),c)
    assert t is None and detail['reason']=='no_observed_overhead_surface'


def test_controller_handoff_preserves_reference_derivatives_and_skips_rest():
    c,g=fixture();ctl=ContinuousTreeInspectionController(c,g,asynchronous=False)
    ctl.state='SURVEY';ctl.tree_phase='APPROACH'
    ctl.active_tree=dict(id=7,xy=[0.,0.],observed_stem_top=5.)
    ctl.view_target=np.array([2.,0.,2.])
    ctl.trajectory=ctl._trajectory_from_points([[3.,0.,2.],[2.,0.,2.]])
    assert ctl.trajectory.approach_ascent_blend
    ctl.elapsed=ctl.trajectory.duration
    ctl.command,ctl.command_v,ctl.command_a=ctl.trajectory.sample(ctl.elapsed)
    p,v,a=ctl.command.copy(),ctl.command_v.copy(),ctl.command_a.copy()
    ctl._on_motion(1.,p)
    assert ctl.tree_phase=='ASCEND' and ctl.ascent_reference is not None
    assert ctl.trajectory is None and ctl.wait_for_scan is None
    np.testing.assert_allclose(ctl.command,p);np.testing.assert_allclose(ctl.command_v,v)
    np.testing.assert_allclose(ctl.ascent_nominal_acceleration,a)
    assert v[2]>0
    g.last_observation_stamp=1.05
    next_p,next_v,next_a,_=ctl._live_reference(1.05,.05,p,v)
    assert next_p[2]>p[2] and next_v[2]>0
    np.testing.assert_allclose(next_p[:2],p[:2],atol=1e-10)
    assert np.linalg.norm(next_a-a)<=c['jerk']*.05+1e-8
