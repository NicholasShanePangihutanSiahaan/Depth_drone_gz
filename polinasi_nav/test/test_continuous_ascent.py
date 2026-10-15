from pathlib import Path
import json
import numpy as np
from polinasi_nav.ascent import ContinuousAscent
from polinasi_nav.config import load_config
from polinasi_nav.mapping import VoxelMap,FREE,UNKNOWN
from polinasi_nav.planning import brake_safe
from polinasi_nav.tree_mission import TreeInspectionController


def fixture():
    c = load_config(Path(__file__).resolve().parents[1]/'config/tilt_inspection.json')
    c.update(bounds_min=[-5.,-5.,0.],bounds_max=[5.,5.,6.])
    grid = VoxelMap(c)
    grid.state[:] = FREE
    grid.seen[:] = 0.
    grid.rebuild(0.)
    grid.last_observation_stamp = 0.
    return c,grid


def test_live_camera_hint_does_not_search_detours_and_worker_recomputes_goal():
    from unittest.mock import patch
    c,grid = fixture()
    ctl = TreeInspectionController(c,grid,asynchronous=False)
    ctl.state,ctl.tree_phase = 'SURVEY','APPROACH'
    ctl.view_target = np.array([4.,0.,2.])
    ctl.goal = np.array([2.,0.,2.])
    ctl.executor = object()  # Simulate live async control without launching a process.
    with patch('polinasi_nav.local_subgoal.inspection_hint',side_effect=AssertionError('live search')):
        np.testing.assert_allclose(ctl._mission_hint([0.,0.,2.]),[2.,0.,2.])
    clone = ctl._planning_clone()
    assert clone.executor is None and clone.goal is None
    with patch('polinasi_nav.local_subgoal.inspection_hint',return_value=(np.array([1.,1.,2.]),{'policy':'worker_detour'})) as search:
        np.testing.assert_allclose(clone._mission_hint([0.,0.,2.]),[1.,1.,2.])
        search.assert_called_once()
    assert clone.subgoal_diagnostics['policy']=='worker_detour'


def run_ascent(c,grid,height):
    ref = ContinuousAscent(height,c)
    p,v,a = np.array([0.,0.,2.]),np.zeros(3),np.zeros(3)
    history = []
    for i in range(600):
        previous_a = a.copy()
        p,v,a = ref.step(grid,p,v,a,.05)
        json.dumps(ref.diagnostics,allow_nan=False)
        assert brake_safe(grid,p,v,c,a)
        assert np.linalg.norm(a-previous_a)/.05<=c['jerk']+1e-6
        assert grid.safe(p) and p[2]<=height+1e-6
        assert np.linalg.norm(v)<=c['command_speed'] and np.linalg.norm(a)<=c['acceleration']+1e-6
        history.append((p.copy(),v.copy()))
        if ref.finished:
            break
    assert ref.finished
    return ref,history


def test_continuous_ascent_has_no_intermediate_stops_and_brakes_before_height():
    c,grid = fixture()
    ref,h = run_ascent(c,grid,2.9)
    assert ref.reason=='desired_height_braking_limit'
    assert h[-1][0][2]>2.7
    cruise = [v[2] for p,v in h if 2.2<p[2]<2.6]
    assert cruise and min(cruise)>.08
    assert np.linalg.norm(h[-1][1])<1e-6


def test_unknown_ceiling_stops_ascent_without_free_space_assumption():
    c,grid = fixture()
    grid.state[:,:,grid.indices([0.,0.,3.5])[2]:] = UNKNOWN
    grid.rebuild(1.)
    ref,h = run_ascent(c,grid,5.)
    assert ref.reason in ('observed_stopping_corridor_limit','observation_guard_limit')
    assert h[-1][0][2]<3.5


def test_observation_guard_reserves_extra_space_without_weakening_footprint():
    c,grid = fixture()
    grid.state[:,:,grid.indices([0.,0.,4.])[2]:] = UNKNOWN
    grid.rebuild(1.)
    c['tree_ascent_observation_guard'] = 0.
    _,normal = run_ascent(c,grid,5.)
    c['tree_ascent_observation_guard'] = .5
    ref,guarded = run_ascent(c,grid,5.)
    assert ref.reason=='observation_guard_limit'
    assert guarded[-1][0][2]<normal[-1][0][2]-.4
    assert abs(ref.diagnostics['observation_guard_m']-.6)<1e-8  # .5 m reserve rounds up at .2 m voxels


def test_camera_gate_requires_down_pitch_and_new_integrated_depth():
    c,grid = fixture()
    ctl = TreeInspectionController(c,grid,asynchronous=False)
    ctl.state,ctl.tree_phase = 'SURVEY','SCAN'
    ctl.active_tree = dict(id=1,xy=[4.,0.])
    ctl.command = np.array([0.,0.,2.])
    ctl.view_target = np.array([2.,0.,2.])
    ctl.mount_feedback = dict(feedback_fresh=True,measured={'camera_pitch':.6},targets={'camera_pitch':-.4})
    ctl.depth_feedback = {'last_integrated_stamp':2.}
    assert not ctl._camera_observation_ready(2.,ctl.command)
    ctl.mount_feedback['measured']['camera_pitch'] = -.4
    assert not ctl._camera_observation_ready(3.,ctl.command)
    assert not ctl._camera_observation_ready(3.5,ctl.command)
    ctl.depth_feedback['last_integrated_stamp'] = 3.6
    assert ctl._camera_observation_ready(3.7,ctl.command)
    assert ctl._same_altitude_exploration()
    assert not ctl._camera_observation_ready(5.,ctl.command)  # Cached ready is not perpetual permission.
    ctl.depth_feedback['last_integrated_stamp'] = 5.5
    assert ctl._camera_observation_ready(5.6,ctl.command)
    ctl.depth_navigation_paused = True
    assert not ctl._prepare_trajectory(5.7,ctl.command)
    assert ctl.trajectory is None and not ctl.failure


def test_live_ascent_uses_common_control_gates_and_stale_map_brakes():
    c,grid = fixture()
    ctl = TreeInspectionController(c,grid,asynchronous=False)
    ctl.state,ctl.tree_phase = 'SURVEY','ASCEND'
    ctl.active_tree = dict(id=1,xy=[3.,0.],observed_stem_top=4.)
    ctl.command = np.array([0.,0.,2.])
    ctl.health.localisation_ok = True
    for i in range(20):
        now = 1.+i*.05
        ctl.health.cloud_stamp = ctl.health.pose_stamp = grid.last_observation_stamp = now
        ctl.tick(now,.05,ctl.command.copy(),ctl.command_v.copy())
        json.dumps(ctl.inspection_status(),allow_nan=False)
        assert not ctl.failure and ctl.trajectory is None
    assert ctl.command[2]>2. and ctl.command_v[2]>0.
    now = 2.5
    ctl.health.cloud_stamp = ctl.health.pose_stamp = now
    ctl.tick(now,.05,ctl.command.copy(),ctl.command_v.copy())
    assert ctl.ascent_reference.reason=='ascent_map_too_old'
    assert ctl.ascent_reference.stop is not None and not ctl.failure
    json.dumps(ctl.inspection_status(),allow_nan=False)


def test_pid_feedback_is_not_integrated_into_vertical_reference():
    c,grid = fixture()
    ctl = TreeInspectionController(c,grid,asynchronous=False)
    ctl.state,ctl.tree_phase = 'SURVEY','ASCEND'
    ctl.active_tree = dict(id=1,xy=[3.,0.],observed_stem_top=4.)
    ctl.command = np.array([0.,0.,2.])
    ctl.health.localisation_ok = True
    for i in range(100):
        now = 1.+i*.05
        ctl.health.cloud_stamp = ctl.health.pose_stamp = grid.last_observation_stamp = now
        ctl.command_a = np.array([.2,-.2,.1])  # Final feedback, not nominal feedforward.
        ctl.tick(now,.05,ctl.command.copy(),ctl.command_v.copy())
        assert not ctl.failure
        np.testing.assert_allclose(ctl.command[:2],0.,atol=1e-12)
        np.testing.assert_allclose(ctl.command_v[:2],0.,atol=1e-12)
    assert ctl.command[2]>2.4 and ctl.ascent_reference.stop is None


def paused_ascent():
    c,grid = fixture()
    ctl = TreeInspectionController(c,grid,asynchronous=False)
    ctl.state,ctl.tree_phase = 'SURVEY','ASCEND'
    ctl.active_tree = dict(id=1,xy=[3.,0.],observed_stem_top=4.)
    ctl.command = np.array([0.,0.,2.])
    ctl.command_a = np.array([.2,-.2,.1])
    for i in range(200):
        now = 1.+i*.05
        result = ctl._live_reference(now,.05,ctl.command,ctl.command_v)
        if result is not None:
            ctl.command,ctl.command_v,ctl.command_a,_ = result
        if ctl.ascent_reference.finished:
            break
    assert ctl.ascent_reference.finished
    assert ctl.tree_phase=='ASCEND' and not ctl.failure
    np.testing.assert_allclose(ctl.command[:2],0.,atol=1e-12)
    return ctl,grid,now


def test_stale_ascent_holds_then_resumes_only_on_new_fresh_map():
    ctl,grid,now = paused_ascent()
    held = ctl.command.copy()
    result = ctl._live_reference(now+.1,.05,held,np.zeros(3))
    np.testing.assert_allclose(result[0],held)
    assert ctl.ascent_map_resumes==0 and ctl.tree_phase=='ASCEND'
    grid.last_observation_stamp = now+.2
    result = ctl._live_reference(now+.2,.05,held,np.zeros(3))
    assert ctl.ascent_map_resumes==1 and ctl.ascent_reference.stop is None
    assert result[0][2]>held[2]
    json.dumps(ctl.inspection_status(),allow_nan=False)


def test_ascent_mapping_recovery_timeout_is_failure_not_successful_scan():
    ctl,grid,now = paused_ascent()
    ctl._live_reference(ctl.ascent_pause_started+21.,.05,ctl.command,np.zeros(3))
    assert ctl.failure=='ascent_mapping_recovery_timeout'
    assert ctl.tree_phase!='SCAN'


def test_recovered_map_with_unknown_ceiling_does_not_allow_ascent():
    ctl,grid,now = paused_ascent()
    grid.state[:,:,grid.indices([0.,0.,2.9])[2]:] = UNKNOWN
    grid.rebuild(now+.2)
    grid.last_observation_stamp = now+.2
    ctl._live_reference(now+.2,.05,ctl.command,np.zeros(3))
    assert ctl.ascent_map_resumes==1
    assert ctl.ascent_reference.stop is not None
    assert ctl.ascent_reference.reason in ('observation_guard_limit','observed_stopping_corridor_limit')
