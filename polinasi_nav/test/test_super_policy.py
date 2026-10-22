"""Bounded engineering adaptation of SUPER ideas, without simulator or GT input."""
import numpy as np
import pytest
from test_continuous import setup
from polinasi_nav.super_policy import (reference_speed,feedback_brake_safe,route_speed_limits,
    densify,optimise_durations,voxel_corridor,explicit_backup)
from polinasi_nav.planning import Brake,Health,brake_safe
from polinasi_nav.mapping import FREE,UNKNOWN,OCCUPIED
from polinasi_nav.continuous import ContinuousTrajectory
from polinasi_nav.survey import SurveyController
from polinasi_nav.pid_control import PositionPID


def fixture():
    c,g=setup();c.update(command_speed=.4,feedback_speed_envelope=True,
        velocity_uncertainty_mps=.03,pid_enabled=True,pid_correction_acceleration=.15,
        pid_kp=[.6,.6,.7],pid_ki=[.02,.02,.03],pid_kd=[.3,.3,.35],pid_integral_limit=.4,
        explicit_backup_enabled=True,duration_optimisation=True,trajectory_spacing_m=1.,
        local_speed_profile=True,trajectory_geometry_cache=True,
        survey_waypoints=[[4.,0.,2.]],survey_batch_points=24)
    return c,g


def test_recorded_failure_reproduces_without_obstacle_and_planning_envelope_leaves_reserve():
    c,g=fixture()
    v=np.array([.39848042796577526,-.020021331825184407,.0033148175533591756])
    a=np.array([.049919508008312345,-.008899140314001882,.0006690414894653318])
    detail={}
    assert not brake_safe(g,[0.,0.,2.],v,c,a,actual=True,diagnostics=detail)
    assert detail['reason']=='predicted_speed_limit'
    assert detail['predicted_max_speed_mps']==pytest.approx(.40158890817378223)
    limit=reference_speed(c)
    assert .3<limit<.37
    assert feedback_brake_safe(g,[0.,0.,2.],[limit,0.,0.],np.zeros(3),c)
    assert c['speed']==.4 and c['tracking_limit']==.1


def test_curve_duration_optimization_preserves_initial_terminal_and_shared_PVA():
    c,g=fixture();points,_=densify(np.array([[0.,0.,2.],[4.,0.,2.]]),1.)
    tr=ContinuousTrajectory(points,c)
    initial=[tr.sample(t) for t in [0.,*tr.ends]]
    before=tr.duration;optimise_durations(tr,g,c)
    assert tr.duration<=before and tr.safe(g)
    for a,b in zip(initial,[tr.sample(t) for t in [0.,*tr.ends]]):
        for x,y in zip(a,b):np.testing.assert_allclose(x,y,atol=1e-8)
    for t in tr.ends[:-1]:
        for x,y in zip(tr.sample(t-1e-7),tr.sample(t+1e-7)):
            np.testing.assert_allclose(x,y,atol=1e-6)
    for value,limit in zip(tr.bounds(),(reference_speed(c),c['acceleration'],c['jerk'])):
        assert value<=limit+1e-8


def test_cruise_knots_shorten_long_leg_without_raising_speed_limit():
    c,g=fixture();legacy=ContinuousTrajectory([[0.,0.,2.],[4.,0.,2.]],c)
    ctl=SurveyController(c,g,asynchronous=False);ctl.state='SURVEY'
    tr=ctl._trajectory_from_points([[0.,0.,2.],[4.,0.,2.]])
    assert tr.duration<.9*legacy.duration
    assert tr.safe(g) and len(tr.controls)>=4
    assert max(tr.knot_speeds)>.1


@pytest.mark.parametrize('state',[UNKNOWN,OCCUPIED])
def test_convex_voxel_corridor_rejects_unknown_and_occupied_hull(state):
    c,g=fixture();tr=ContinuousTrajectory([[0.,0.,2.],[1.,0.,2.]],c)
    assert voxel_corridor(g,tr.controls[0]) is not None
    g.state[tuple(g.indices([.5,0.,2.]))]=state;g.rebuild(0.)
    assert voxel_corridor(g,tr.controls[0]) is None
    assert not tr.safe(g)


def test_route_profile_slows_before_forward_wall_and_rejects_fully_blocked_route():
    c,g=fixture()
    points=np.array([[0.,0.,2.],[1.,0.,2.],[2.,0.,2.],[2.,2.,2.]])
    normal,_=route_speed_limits(g,points,c,reference_speed(c))
    g.state[tuple(g.indices([2.8,0.,2.]))]=OCCUPIED;g.rebuild(0.)
    reduced,diag=route_speed_limits(g,points,c,reference_speed(c))
    assert reduced[-2]<normal[-2] and diag['rejected_reasons'][-2]
    g.state[tuple(g.indices([1.,0.,2.]))]=OCCUPIED;g.rebuild(0.)
    with pytest.raises(ValueError):route_speed_limits(g,points,c,reference_speed(c))


def test_explicit_backup_has_matching_PVA_bounded_derivatives_and_terminal_stop():
    c,g=fixture();p=np.array([0.,0.,2.]);v=np.array([.3,0.,0.]);a=np.array([.04,.01,0.])
    backup,detail=explicit_backup(g,p,v,a,c,optimise=True)
    assert backup is not None and detail['valid']
    np.testing.assert_allclose(backup.coeff[0],p)
    np.testing.assert_allclose(backup.coeff[1],v)
    np.testing.assert_allclose(2*backup.coeff[2],a)
    backup.step(backup.duration)
    np.testing.assert_allclose(backup.v,0.,atol=1e-8)
    np.testing.assert_allclose(backup.a,0.,atol=1e-8)
    assert detail['max_speed_bound_mps']<=c['speed']+1e-8


@pytest.mark.parametrize('phase',['APPROACH','SCAN','RETURN'])
def test_PID_materializes_backup_and_failure_executes_it_after_live_recheck(phase):
    c,g=fixture();ctl=SurveyController(c,g,asynchronous=False)
    ctl.state='RETURN' if phase=='RETURN' else 'SURVEY';ctl.tree_phase=phase
    ctl.command=np.array([0.,0.,2.]);ctl.command_v=np.array([.2,0.,0.]);ctl.command_a=np.zeros(3)
    pid=PositionPID(c);pid.apply(.05,ctl,ctl.command.copy(),ctl.command_v.copy())
    saved=ctl.certified_backup
    assert saved is not None and ctl.backup_diagnostics['valid']
    ctl.fail('test_abort')
    assert ctl.brake is saved and ctl.backup_diagnostics['executed']
    assert ctl.backup_diagnostics['live_rechecked']
    before=ctl.command.copy();ctl._braking(.05)
    assert np.linalg.norm(ctl.command-before)<.03


def test_invalidated_backup_is_never_claimed_live_safe():
    c,g=fixture();ctl=SurveyController(c,g,asynchronous=False)
    ctl.state='SURVEY';ctl.command=np.array([0.,0.,2.]);ctl.command_v=np.array([.3,0.,0.])
    pid=PositionPID(c);pid.apply(.05,ctl,ctl.command.copy(),ctl.command_v.copy())
    saved=ctl.certified_backup
    assert saved is not None
    end=np.asarray(ctl.backup_diagnostics['end_position'])
    g.state[tuple(g.indices(end))]=OCCUPIED;g.rebuild(0.)
    ctl.fail('test_new_obstacle')
    assert ctl.brake is not saved and not ctl.backup_diagnostics['executed']


def test_PID_controller_replay_with_accepted_velocity_noise_completes_long_leg():
    c,g=fixture();ctl=SurveyController(c,g,asynchronous=False);ctl.state='SURVEY'
    ctl.command=np.array([0.,0.,2.]);pid=PositionPID(c);peak=0.
    for i in range(2000):
        now=i*.05;ctl.health=Health(now,now,now,True);g.last_observation_stamp=now
        p=ctl.command+np.array([.008*np.sin(now*2),.008*np.cos(now*3),0.])
        v=ctl.command_v+np.array([.025*np.sin(now*7),.005*np.cos(now*4),0.])
        ctl.tick(now,.05,p,v);pid.apply(.05,ctl,p,v)
        peak=max(peak,np.linalg.norm(ctl.command_v))
        if ctl.failure or ctl.state!='SURVEY':break
    assert not ctl.failure
    assert ctl.state=='RETURN' and peak>.3
    assert ctl.backup_diagnostics['valid']


def test_actual_guard_records_specific_predicted_speed_failure():
    c,g=fixture();ctl=SurveyController(c,g,asynchronous=False);ctl.state='SURVEY'
    ctl.command=np.array([0.,0.,2.]);ctl.command_v=np.array([.39,0.,0.])
    ctl.command_a=np.array([.05,0.,0.]);ctl.health=Health(1.,1.,1.,True)
    ctl.tick(1.,.05,ctl.command.copy(),np.array([.399,0.,0.]))
    assert ctl.failure=='insufficient_stopping_clearance'
    assert ctl.route_failure_diagnostics['stopping']['reason']=='predicted_speed_limit'


def test_future_switch_is_bounded_and_has_a_stop_certificate():
    from polinasi_nav.super_policy import select_backup_switch
    c,g=fixture();tr=ContinuousTrajectory([[0.,0.,2.],[2.,0.,2.],[4.,0.,2.]],c)
    elapsed=tr.duration*.35
    chosen,detail=select_backup_switch(tr,elapsed,.75,g,c)
    assert elapsed+.75<=chosen<=elapsed+.85+1e-8
    assert detail['stop']['valid'] and detail['stop']['max_speed_bound_mps']<=c['speed']


def test_sensor_dropout_does_not_get_hidden_by_valid_cached_backup():
    c,g=fixture();ctl=SurveyController(c,g,asynchronous=False);ctl.state='SURVEY'
    ctl.command=np.array([0.,0.,2.]);ctl.command_v=np.array([.2,0.,0.])
    pid=PositionPID(c);pid.apply(.05,ctl,ctl.command.copy(),ctl.command_v.copy())
    assert ctl.certified_backup is not None
    ctl.health=Health(0.,10.,10.,True)
    ctl.tick(10.,.05,ctl.command.copy(),ctl.command_v.copy())
    assert ctl.failure=='stale_lidar'
    assert ctl.backup_diagnostics['executed'] and ctl.backup_diagnostics['trigger']=='stale_lidar'


def test_moving_replay_includes_PID_envelope_timing_and_async_backup_switch():
    from concurrent.futures import Future
    class ImmediateExecutor:
        def submit(self,fn,*args):
            f=Future()
            try:f.set_result(fn(*args))
            except Exception as exc:f.set_exception(exc)
            return f
    c,g=fixture();c.update(survey_waypoints=[[6.,0.,2.]],history_route_enabled=True,
        history_route_local_lookahead=4.,history_route_max_age=8.,moving_handoff_enabled=True,
        moving_handoff_lead=.75,moving_result_max_age=1.,survey_prefetch=False)
    ctl=SurveyController(c,g,asynchronous=False);ctl.executor=ImmediateExecutor()
    ctl.state='SURVEY';ctl.command=np.array([0.,0.,2.]);pid=PositionPID(c)
    for i in range(1600):
        now=i*.05;ctl.health=Health(now,now,now,True);g.last_observation_stamp=now
        ctl.global_route=dict(goal=[6.,0.,2.],path=[[0.,0.,2.],[6.,0.,2.]],stamp=now,
                              partial=False,request_id=i)
        p=ctl.command.copy();v=ctl.command_v.copy()
        ctl.tick(now,.05,p,v);pid.apply(.05,ctl,p,v)
        if ctl.state!='SURVEY' or ctl.failure:break
    assert not ctl.failure and ctl.state=='RETURN'
    assert ctl.moving_status['accepted']>=1
    assert max(ctl.moving_status['PVA_jump'])<1e-8
    assert ctl.moving_status['switch_backup']['stop']['valid']
    assert ctl.route_speed_profile['segment_speed_limits_mps']
    assert not ctl.route_speed_profile['whole_curve_certificate']
    # Continue through the previously untested return transition with the
    # same async/PID/profile stack, rather than stopping the test at RETURN.
    for step in range(1600):
        now+=.05;ctl.health=Health(now,now,now,True);g.last_observation_stamp=now
        ctl.global_route=dict(goal=[0.,0.,2.],path=[ctl.command.tolist(),[0.,0.,2.]],
                              stamp=now,partial=False,request_id=2000+step)
        p=ctl.command.copy();v=ctl.command_v.copy()
        ctl.tick(now,.05,p,v);pid.apply(.05,ctl,p,v)
        if ctl.state!='RETURN' or ctl.failure:break
    assert not ctl.failure and ctl.state=='DESCEND'
    assert not ctl.batch_knots and ctl.batch_end_index is None
    assert np.linalg.norm(ctl.command-np.array([0.,0.,2.]))<.12


def test_cached_backup_flag_does_not_suppress_new_occupied_or_unknown():
    c,g=fixture();p=np.array([0.,0.,2.]);v=np.array([.3,0.,0.]);a=np.zeros(3)
    stop,detail=explicit_backup(g,p,v,a,c)
    assert stop is not None
    hit=np.asarray(detail['end_position'])
    g.state[tuple(g.indices(hit))]=UNKNOWN;g.rebuild(1.)
    assert not stop.path_safe(g)


def test_scan_backup_retains_reserved_overhead_buffer():
    from polinasi_nav.super_policy import backup_grid
    c,g=fixture();c.update(scan_adaptive_height=True,tree_ascent_observation_guard=.4)
    g.state[:,:,g.indices([0.,0.,2.65])[2]]=OCCUPIED;g.rebuild(0.)
    ctl=SurveyController(c,g,asynchronous=False);ctl.state='SURVEY';ctl.tree_phase='SCAN'
    p=np.array([0.,0.,2.]);v=np.array([.2,0.,0.]);a=np.zeros(3)
    assert explicit_backup(g,p,v,a,c)[0] is not None
    assert explicit_backup(backup_grid(ctl),p,v,a,c)[0] is None


def test_terminal_stop_does_not_require_cruise_space_past_destination():
    c,g=fixture()
    # Endpoint remains observed free; unknown immediately beyond its body
    # envelope must stay blocked, rather than being relabelled free.
    g.state[g.indices([2.6,0.,2.])[0]:,:,:]=UNKNOWN
    g.rebuild(0.)
    points=np.array([[0.,0.,2.],[1.,0.,2.],[2.,0.,2.]])
    assert g.safe(points[-1])
    assert not feedback_brake_safe(g,points[-1],[reference_speed(c),0.,0.],np.zeros(3),c)
    caps,detail=route_speed_limits(g,points,c,reference_speed(c))
    assert np.all(caps>0) and not detail['whole_curve_certificate']
    assert g.state[tuple(g.indices([3.,0.,2.]))]==UNKNOWN
    ctl=SurveyController(c,g,asynchronous=False);ctl.state='SURVEY'
    tr=ctl._trajectory_from_points(points)
    assert tr.safe(g)
    np.testing.assert_allclose(tr.sample(tr.duration)[1:],0.,atol=1e-8)


def test_survey_worker_failure_retains_exception_reason():
    from concurrent.futures import Future
    c,g=fixture();ctl=SurveyController(c,g,asynchronous=True)
    job=Future();job.set_exception(ValueError('no robust route braking envelope'))
    ctl.pending_plan=job
    assert not ctl._prepare_trajectory(0.,np.array([0.,0.,2.]),None,None)
    assert ctl.failure=='survey_planner_error'
    assert all(ctl.route_failure_diagnostics[k]==v for k,v in dict(gate='survey_worker',
        exception_type='ValueError',message='no robust route braking envelope').items())
    assert 'ValueError: no robust route braking envelope' in ctl.route_failure_diagnostics['exception_traceback']
    ctl.executor.shutdown(wait=True)


def test_speed_refinement_finds_candidate_below_old_scan_minimum(monkeypatch):
    import polinasi_nav.super_policy as policy
    c,g=fixture()
    def check(grid,p,v,a,config,diagnostics=None):
        if diagnostics is not None:diagnostics['reason']='safe' if np.linalg.norm(v)<=.05 else 'braking_box_blocked'
        return np.linalg.norm(v)<=.05
    monkeypatch.setattr(policy,'feedback_brake_safe',check)
    caps,detail=route_speed_limits(g,np.array([[0.,0.,2.],[1.,0.,2.]]),c,.3)
    assert 0<caps[0]<.09
    assert detail['attempts'][0]['check']['reason']=='braking_box_blocked'


def test_chord_preprofile_rejection_still_requires_actual_curve_certificate(monkeypatch):
    import polinasi_nav.super_policy as policy
    def reject(*args,**kwargs):
        raise policy.RouteSpeedEnvelopeError(dict(reason='chord_preprofile_exhausted'))
    monkeypatch.setattr(policy,'route_speed_limits',reject)
    c,g=fixture();ctl=SurveyController(c,g,asynchronous=False)
    ctl.state='SURVEY';ctl.command=np.array([0.,0.,2.])
    assert ctl._prepare_trajectory(0.,ctl.command)
    assert ctl.planner_stages_ms['braking_sample_checks']>0
    assert ctl.trajectory.safe(g) and not ctl.failure
    assert ctl.route_speed_profile['fallback']=='actual_PVA_curve_certification_required'


def test_candidate_retry_keeps_existing_timeout_and_does_not_mask_unsafe_hold():
    from polinasi_nav.planner_recovery import reject_candidate
    c,g=fixture();c['blocked_timeout']=4.
    ctl=SurveyController(c,g,asynchronous=False);ctl.state='SURVEY'
    ctl.command=np.array([0.,0.,2.])
    assert not reject_candidate(ctl,0.,dict(reason='no_safe_braking_trajectory'))
    assert not ctl.failure and ctl.blocked_since==0.
    assert not reject_candidate(ctl,5.,dict(reason='no_safe_braking_trajectory'))
    assert ctl.failure=='blocked_route'
    other=SurveyController(c,g,asynchronous=False);other.state='SURVEY';other.command=ctl.command.copy()
    g.state[tuple(g.indices(other.command))]=UNKNOWN;g.rebuild(0.)
    reject_candidate(other,0.,dict(reason='no_safe_braking_trajectory'))
    assert other.failure=='no_safe_hold_after_planner_rejection'


def test_failure_snapshot_is_exact_numeric_grid_and_bounded(tmp_path):
    import json
    from polinasi_nav.planner_recovery import save_failure_snapshot
    c,g=fixture();c['planner_audit_dir']=str(tmp_path)
    ctl=SurveyController(c,g,asynchronous=False);ctl.state='SURVEY'
    ctl.command=np.array([0.,0.,2.])
    for i in range(10):save_failure_snapshot(ctl,dict(reason='test'),[[0.,0.,2.],[1.,0.,2.]])
    files=list(tmp_path.glob('*.npz'));assert len(files)==8
    with np.load(files[0],allow_pickle=False) as snapshot:
        np.testing.assert_array_equal(snapshot['blocked'],g.blocked)
        np.testing.assert_array_equal(snapshot['state'],g.state)
        assert json.loads(str(snapshot['metadata']))['provenance']=='exact_detached_planning_grid_not_ground_truth'


def test_local_braking_retime_preserves_C2_and_distant_cruise():
    from polinasi_nav.local_braking_retime import retime_local
    c,g=fixture();tr=ContinuousTrajectory([[x*.6,0.,2.] for x in range(11)],c)
    new=retime_local(tr,float(tr.ends[4]-.1),g,c)
    assert new is not None and new.safe(g)
    assert tr.duration<new.duration<1.4*tr.duration
    assert new.durations[0]==tr.durations[0] and new.durations[-1]==tr.durations[-1]
    np.testing.assert_array_equal(new.controls[0],tr.controls[0])
    for a,b in zip(tr.sample(0.),new.sample(0.)):np.testing.assert_allclose(a,b,atol=1e-8)
    np.testing.assert_allclose(new.sample(new.duration)[1:],0.,atol=1e-8)
    for t in new.ends[:-1]:
        for a,b in zip(new.sample(t-1e-7),new.sample(t+1e-7)):np.testing.assert_allclose(a,b,atol=1e-6)
    assert np.all(new.bounds()<=np.array([reference_speed(c),c['acceleration'],c['jerk']])+1e-8)


def test_local_retime_does_not_override_fixed_moving_PVA():
    from polinasi_nav.local_braking_retime import retime_local
    from polinasi_nav.moving_trajectory import BoundaryTrajectory
    c,g=fixture();tr=BoundaryTrajectory([[0.,0.,2.],[2.,0.,2.]],c,[.2,0.,0.],[.01,0.,0.])
    assert retime_local(tr,1.,g,c) is None
    np.testing.assert_allclose(tr.sample(0.)[1],[.2,0.,0.],atol=1e-8)


def test_segment_braking_cache_is_only_reused_on_unchanged_controls_in_job():
    from polinasi_nav.local_braking_retime import check_braking,retime_local
    c,g=fixture();tr=ContinuousTrajectory([[x*.6,0.,2.] for x in range(11)],c)
    cache=set();safe,_,checks,_=check_braking(tr,g,c,cache)
    assert safe and checks>0
    safe,_,checks,hits=check_braking(tr,g,c,cache)
    assert safe and checks==0 and hits==len(tr.controls)
    new=retime_local(tr,float(tr.ends[4]-.1),g,c)
    safe,_,checks,hits=check_braking(new,g,c,cache)
    assert safe and checks>0 and hits>=2
    # A new map always gets a new cache in the planner; unknown must reject.
    g.state[tuple(g.indices([.5,0.,2.]))]=UNKNOWN;g.rebuild(1.)
    safe,detail,checks,_=check_braking(new,g,c,cache)
    assert not safe and checks>0 and detail['check']['reason']


def test_history_endpoint_retreats_from_static_safe_but_unstoppable_window_edge():
    from polinasi_nav.global_route import local_route_goal
    from polinasi_nav.local_braking_retime import terminal_reserve_check
    c,g=fixture();c['history_route_local_lookahead']=20.
    start=np.array([0.,0.,2.])
    centers=[g.centers([i,g.indices(start)[1],g.indices(start)[2]]) for i in range(g.shape[0]-1,0,-1)]
    center=next(p for p in centers if g.safe(p))
    end=center+np.array([g.res*.49,0.,0.]);end[1:]=start[1:]
    assert g.safe(end)
    tr=ContinuousTrajectory([start,end],c)
    safe,detail=terminal_reserve_check(tr,g,c)
    assert not safe and 'outside_window' in detail['check']['blockage']['causes']
    destination=end+np.array([2.,0.,0.])
    route=dict(goal=destination.tolist(),path=[start.tolist(),end.tolist(),destination.tolist()],
        stamp=0.,partial=False,request_id=1)
    point,detail=local_route_goal(g,start,destination,route,0.,c)
    assert point is not None and point[0]<end[0]
    assert feedback_brake_safe(g,point,[1e-6,0.,0.],np.zeros(3),c)
    assert detail['terminal_reserve_rejections']


def test_pending_worker_does_not_extend_existing_blocked_timeout():
    from concurrent.futures import Future
    c,g=fixture();c['blocked_timeout']=2.
    ctl=SurveyController(c,g,asynchronous=False);ctl.state='SURVEY'
    ctl.command=np.array([0.,0.,2.]);ctl.blocked_since=0.;ctl.pending_plan=Future()
    assert not ctl._prepare_trajectory(2.1,ctl.command)
    assert ctl.failure=='blocked_route' and not ctl.pending_plan.done()


def test_backup_failure_keeps_mask_witness_and_critical_snapshot_beyond_budget(tmp_path):
    from polinasi_nav.planner_recovery import save_failure_snapshot
    c,g=fixture();p=np.array([0.,0.,2.]);g.state[tuple(g.indices(p))]=UNKNOWN;g.rebuild(0.)
    stop,detail=explicit_backup(g,p,np.zeros(3),np.zeros(3),c)
    assert stop is None and not detail['start_safe']
    assert detail['rejections'][0]['path_reason']=='braking_box_blocked'
    c['planner_audit_dir']=str(tmp_path);ctl=SurveyController(c,g,asynchronous=False)
    for _ in range(8):save_failure_snapshot(ctl,dict(reason='ordinary'))
    critical=dict(reason='pid_final_safety');save_failure_snapshot(ctl,critical)
    assert (tmp_path/'failure_critical.npz').exists() and len(list(tmp_path.glob('*.npz')))==9



def test_return_clears_scan_batch_indices_before_new_route_densification():
    c,g=fixture();c.update(history_route_enabled=True,history_route_local_lookahead=4.)
    ctl=SurveyController(c,g,asynchronous=False)
    ctl.state='RETURN';ctl.command=np.array([3.,0.,2.])
    ctl.global_route=dict(goal=[0.,0.,2.],path=[[3.,0.,2.],[0.,0.,2.]],stamp=0.,partial=False,request_id=1)
    ctl.batch_end_index=23;ctl.batch_knots={16:3,23:9}
    assert ctl._prepare_trajectory(0.,ctl.command)
    assert not ctl.failure and ctl.trajectory.safe(g)
    assert ctl.batch_knots=={} and ctl.batch_end_index is None
    np.testing.assert_allclose(ctl.goal,c['home'])
    np.testing.assert_allclose(ctl.trajectory.sample(ctl.trajectory.duration)[1:],0.,atol=1e-8)


def test_direct_return_curve_does_not_remap_stale_scan_indices():
    c,g=fixture();ctl=SurveyController(c,g,asynchronous=False)
    ctl.state='RETURN';ctl.batch_knots={23:9};ctl.batch_end_index=23
    tr=ctl._trajectory_from_points([[3.,0.,2.],[0.,0.,2.]])
    assert tr.safe(g) and not ctl.batch_knots and ctl.batch_end_index is None


def test_new_survey_route_preserves_correct_anchor_mapping_after_densification():
    c,g=fixture();c.update(survey_waypoints=[[1.,0.,2.],[2.,0.,2.],[3.,0.,2.]],trajectory_spacing_m=.4)
    ctl=SurveyController(c,g,asynchronous=False);ctl.state='SURVEY';ctl.command=np.array([0.,0.,2.])
    ctl.batch_knots={23:99};ctl.batch_end_index=23
    assert ctl._choose_goal(ctl.command)
    points=ctl._plan_route(ctl.command,ctl.goal)
    tr=ctl._trajectory_from_points(points)
    assert set(ctl.batch_knots)=={0,1,2}
    for anchor,knot in ctl.batch_knots.items():
        np.testing.assert_allclose(tr.points[knot],ctl.waypoints[anchor])


def test_emergency_brake_records_nominal_step_failure_with_actual_stop_still_safe(tmp_path):
    import json
    from polinasi_nav.planning import Brake
    c,g=fixture();c['planner_audit_dir']=str(tmp_path)
    # A wall in the tracking reserve, outside the actual-body stopping mask.
    selected=None
    for y in range(g.indices([0.,0.,0.])[1]+1,g.shape[1]-1):
        g.state[:]=FREE;g.state[:,y,:]=OCCUPIED;g.rebuild(0.)
        stop=Brake([0.,0.,2.],[.2,0.,0.],np.zeros(3),c)
        if not g.safe(stop.p) and stop.path_safe(g,actual=True):
            selected=y;break
    assert selected is not None
    ctl=SurveyController(c,g,asynchronous=False)
    ctl.command=np.array([0.,0.,2.]);ctl.command_v=np.array([.2,0.,0.]);ctl.command_a=np.zeros(3)
    ctl.brake=stop;ctl.state='BRAKE';ctl.failure='route_changed'
    ctl._braking(.05)
    assert ctl.failure=='emergency_unavoidable:route_changed'
    detail=ctl.route_failure_diagnostics['emergency_braking']
    assert detail['gate']=='nominal_next_step'
    assert not ctl.backup_diagnostics['valid'] and ctl.backup_diagnostics['invalidated']
    assert 'occupied' in detail['check']['causes']
    with np.load(tmp_path/'failure_critical.npz',allow_pickle=False) as saved:
        metadata=json.loads(str(saved['metadata']))
        assert metadata['diagnostics']['reason']=='emergency_braking_invalidated'
        assert np.asarray(metadata['diagnostics']['brake_coefficients']).shape==(6,3)
        np.testing.assert_array_equal(saved['blocked_actual'],g.blocked_actual)


def recorded_backup_fixture():
    from polinasi_nav.mapping import VoxelMap
    c,_=fixture()
    c.update(resolution=.2,bounds_min=[-12.2,-7.,-.6],bounds_max=[-2.2,3.,7.4],
        drone_dimensions=[.6,.6,.7],clearance=.2,tracking_margin=.2,
        speed=.4,acceleration=.5,jerk=1.,reaction_time=.35,
        pid_kp=[.6]*3,pid_ki=[.03]*3,pid_kd=[.9]*3,pid_integral_limit=.5,
        scan_adaptive_height=True,tree_ascent_observation_guard=.2,
        explicit_backup_fallback_search=True)
    g=VoxelMap(c);g.state[:]=FREE;g.seen[:]=0.
    g.state[tuple(g.indices([-6.7,-2.9,3.3]))]=OCCUPIED;g.rebuild(0.)
    p=np.array([-7.071925142263441,-1.8692921643804425,2.273857811509911])
    v=np.array([.17283610606784022,.1894690525776391,.08263806543018591])
    a=np.array([.028437756392196777,.04251019618969396,-.1230982091356313])
    return c,g,p,v,a


def test_recorded_backup_fallback_preserves_PVA_buffer_and_derivative_limits():
    from polinasi_nav.canopy_scan import canopy_guard_grid
    c,g,p,v,a=recorded_backup_fixture();view=canopy_guard_grid(g,.2)
    before=g.blocked.copy()
    assert explicit_backup(view,p,v,a,dict(c,explicit_backup_fallback_search=False))[0] is None
    stop,detail=explicit_backup(view,p,v,a,c)
    assert stop is not None and stop.path_safe(view)
    np.testing.assert_allclose(stop.coeff[0],p)
    np.testing.assert_allclose(stop.coeff[1],v)
    np.testing.assert_allclose(2*stop.coeff[2],a)
    assert detail['fallback_search'] and detail['candidate_count']==21
    assert detail['duration_seconds']==pytest.approx(1.5284229188629095)
    for key,limit in [('speed_mps',c['speed']),('acceleration_mps2',c['acceleration']),('jerk_mps3',c['jerk'])]:
        assert detail['derivative_bounds'][key]<=limit+1e-8
    np.testing.assert_array_equal(g.blocked,before)
    stop.step(stop.duration)
    np.testing.assert_allclose(stop.v,0.,atol=1e-10)
    np.testing.assert_allclose(stop.a,0.,atol=1e-10)
    # Cached success never grants permission for a later map's unknown.
    hit=view.indices(detail['end_position']);g.state[tuple(hit)]=UNKNOWN;g.rebuild(.1)
    assert not stop.path_safe(canopy_guard_grid(g,.2))


@pytest.mark.parametrize('enabled',[False,True])
def test_recorded_PID_accepts_only_a_live_certified_backup(enabled):
    c,g,p,v,a=recorded_backup_fixture()
    c.update(explicit_backup_fallback_search=enabled,survey_waypoints=[p.tolist()])
    ctl=SurveyController(c,g,asynchronous=False);ctl.state='SURVEY';ctl.tree_phase='SCAN'
    ctl.command=p;ctl.command_v=v
    ctl.command_a=np.array([-.011741389200687772,.02502311904454475,-.10481500200390763])
    pid=PositionPID(c);pid.state='SURVEY'
    pid.integral=np.array([.5,-.22817341006700476,-.08754935743504393])
    pid.previous_acceleration=np.array([.06315203964528382,.0767327388916404,-.13422188250534997])
    pid.apply(.05,ctl,np.array([-7.134802158437445,-1.916098370616943,2.2666145006815914]),
              np.array([.15873806737813162,.19065431707996397,.11986945724273014]))
    if not enabled:
        assert ctl.failure=='pid_final_safety'
        return
    assert not ctl.failure and pid.last['correction_scale']==.5
    assert ctl.backup_diagnostics['valid'] and ctl.backup_diagnostics['fallback_search']
    saved=ctl.certified_backup
    ctl.fail('test_abort')
    assert ctl.brake is saved and ctl.backup_diagnostics['executed']
    assert ctl.backup_diagnostics['live_rechecked']


def test_planner_certifies_immediate_backup_in_addition_to_reaction_stop():
    from types import SimpleNamespace
    from polinasi_nav.canopy_scan import canopy_guard_grid
    from polinasi_nav.local_braking_retime import check_braking
    c,g,p,v,a=recorded_backup_fixture();view=canopy_guard_grid(g,.2)
    p=p+[.05,.05,0.]
    assert brake_safe(view,p,v,c,a) and feedback_brake_safe(view,p,v,a,c)
    # A single PVA probe isolates the planner's pointwise backup certificate.
    probe=SimpleNamespace(duration=0.,sample=lambda t:(p,v,a))
    old=dict(c,explicit_backup_fallback_search=False)
    ok,reason,_,_=check_braking(probe,view,old,set())
    assert not ok and reason['check']['reason']=='explicit_backup_unavailable'
    ok,_,_,_=check_braking(probe,view,c,set())
    assert ok


def test_backup_search_never_turns_unknown_into_free():
    from polinasi_nav.canopy_scan import canopy_guard_grid
    c,g,p,v,a=recorded_backup_fixture();g.state[:]=UNKNOWN;g.rebuild(0.)
    stop,detail=explicit_backup(canopy_guard_grid(g,.2),p,v,a,c)
    assert stop is None and detail['fallback_search']
    assert detail['candidate_count']==21 and not detail['start_safe']
    assert np.all(g.state==UNKNOWN)


def test_feedback_checks_velocity_uncertainty_without_and_with_PID_addition():
    c,g=fixture();detail={}
    assert feedback_brake_safe(g,[0.,0.,2.],[.2,0.,0.],np.zeros(3),c,detail)
    assert detail['checked_feedback_scenarios']==[
        'velocity_uncertainty_only','velocity_uncertainty_and_forward_PID']
    detail={}
    assert feedback_brake_safe(g,[0.,0.,2.],[.2,0.,0.],np.zeros(3),
                              dict(c,pid_correction_acceleration=0.),detail)
    assert detail['checked_feedback_scenarios']==['velocity_uncertainty_only']


def test_recorded_reference_is_rejected_before_PVA_commit_and_cached_stop_executes():
    from types import SimpleNamespace
    from polinasi_nav.mapping import VoxelMap
    from polinasi_nav.canopy_scan import canopy_guard_grid
    from polinasi_nav.control import MissionController
    c,_=fixture()
    c.update(resolution=.2,bounds_min=[-15.,-4.,-.6],bounds_max=[-5.,6.,7.4],
        drone_dimensions=[.6,.6,.7],clearance=.2,tracking_margin=.2,
        tracking_limit=.2,scan_adaptive_height=True,tree_ascent_observation_guard=.2,
        explicit_backup_fallback_search=True,planner_audit_dir='')
    g=VoxelMap(c);g.state[:]=FREE;g.seen[:]=0.
    g.state[tuple(g.indices([-9.3,.1,2.9]))]=OCCUPIED;g.rebuild(0.)
    p=np.array([-10.022164494765263,1.0844112159175885,2.44306100280372])
    v=np.array([-.04386893585437042,-.210380880915864,-.057385888989584265])
    a=np.array([.043047611904741476,-.004048600901767493,-.002165948795106752])
    assert brake_safe(g,p,v,c,a)
    detail={}
    assert not feedback_brake_safe(g,p,v,a,c,detail)
    assert detail['feedback_scenario']=='velocity_uncertainty_only'
    c['survey_waypoints']=[p.tolist()]
    ctl=SurveyController(c,g,asynchronous=False);ctl.state='SURVEY';ctl.tree_phase='SCAN'
    ctl.command=np.array([-10.0199170691907,1.094919971627822,2.445926133422369])
    ctl.command_v=np.array([-.046031309023801516,-.20986504061401384,-.05719027982448127])
    ctl.command_a=np.array([.02964144596245399,.008620294543862332,-.01660339158945337])
    # Replay one actual proposed PVA sample from the failed flight. The
    # trajectory is already accepted; this test targets the final commit gate.
    ctl.trajectory=SimpleNamespace();ctl.checked_map_version=g.version
    ctl._sample_next=lambda dt,position,velocity:(p,v,a,1.)
    ctl.health=Health(1.,1.,1.,True)
    saved,status=explicit_backup(canopy_guard_grid(g,.2),ctl.command,ctl.command_v,ctl.command_a,c)
    assert saved is not None
    ctl.certified_backup=saved;ctl.backup_diagnostics=status
    MissionController.tick(ctl,1.,.05,
        np.array([-10.015101054693199,1.1016713836894678,2.4557112990515977]),
        np.array([-.05058338008336855,-.22651822697084098,-.06064304112112882]))
    assert ctl.failure=='route_changed' and ctl.state=='BRAKE'
    assert ctl.backup_diagnostics['executed'] and ctl.brake is saved
    assert not np.array_equal(ctl.command,p)
    rejected=ctl.route_failure_diagnostics['rejected_reference']
    assert rejected['reason']=='reference_feedback_reserve_revoked'
    assert rejected['check']['feedback_scenario']=='velocity_uncertainty_only'
    for _ in range(100):
        ctl._braking(.05)
        if not ctl.failure:break
    assert not ctl.failure and ctl.state=='SURVEY'
    np.testing.assert_allclose(ctl.command_v,0.,atol=1e-10)


def test_transit_backup_uses_same_guard_and_retains_it_during_brake():
    from types import SimpleNamespace
    from polinasi_nav.super_policy import backup_grid
    from polinasi_nav.transit_guard import transit_guard_grid
    c,g=fixture();c.update(transit_observation_guard=.2,transit_lateral_observation_guard=.2)
    g.state[tuple(g.indices([0.,1.5,2.]))]=OCCUPIED;g.rebuild(0.)
    expected=transit_guard_grid(g,.2,.2)
    ids=np.argwhere((~g.blocked)&expected.blocked)
    points=g.centers(ids);p=points[np.argmin(np.linalg.norm(points-[0.,1.5,2.],axis=1))]
    assert g.safe(p) and not expected.safe(p)
    ctl=SimpleNamespace(c=c,grid=g,state='SURVEY',tree_phase='APPROACH')
    guarded=backup_grid(ctl)
    np.testing.assert_array_equal(guarded.blocked,expected.blocked)
    assert feedback_brake_safe(g,p,np.zeros(3),np.zeros(3),c)
    assert not feedback_brake_safe(guarded,p,np.zeros(3),np.zeros(3),c)
    ctl.state='BRAKE';ctl.resume_state='SURVEY'
    np.testing.assert_array_equal(backup_grid(ctl).blocked,expected.blocked)
    ctl.tree_phase='SEARCH';ctl.state='RETURN'
    np.testing.assert_array_equal(backup_grid(ctl).blocked,expected.blocked)
    ctl.state='BRAKE';ctl.resume_state='RETURN'
    np.testing.assert_array_equal(backup_grid(ctl).blocked,expected.blocked)
    ctl.state='TAKEOFF'
    assert backup_grid(ctl) is g


def test_evidence_snapshots_preserve_acceptance_and_revocation_separately(tmp_path):
    import json
    from polinasi_nav.planner_recovery import save_failure_snapshot
    from polinasi_nav.transit_guard import transit_guard_grid
    c,g=fixture();c['planner_audit_dir']=str(tmp_path)
    ctl=SurveyController(c,g,asynchronous=False)
    ctl.state='SURVEY';ctl.command=np.array([0.,0.,2.])
    ctl.trajectory=ContinuousTrajectory([[0.,0.,2.],[1.,0.,2.]],c)
    guarded=transit_guard_grid(g,.2,.2);before=guarded.blocked.copy()
    save_failure_snapshot(ctl,dict(reason='certified_candidate'),grid=guarded,evidence_slot='accepted')
    g.state[tuple(g.indices([1.,0.,2.]))]=OCCUPIED;g.rebuild(1.)
    ctl.fail('route_changed')
    assert ctl.trajectory is None
    for _ in range(10):save_failure_snapshot(ctl,dict(reason='ordinary'))
    save_failure_snapshot(ctl,dict(reason='global_route_unavailable'))
    assert len(list(tmp_path.glob('*.npz')))==11
    with np.load(tmp_path/'evidence_accepted.npz') as d:
        np.testing.assert_array_equal(d['blocked'],before)
        assert len(d['controls'])>0
        meta=json.loads(str(d['metadata']))
        assert meta['transit_lateral_guard_cells']==int(np.ceil(.2/g.res-1e-9))
        assert meta['canopy_guard_cells']==int(np.ceil(.2/g.res-1e-9))
    with np.load(tmp_path/'evidence_revoked.npz') as d:
        np.testing.assert_array_equal(d['state'],g.state)
        assert len(d['controls'])>0  # captured before fail clears the trajectory
        assert not np.array_equal(d['state'],np.load(tmp_path/'evidence_accepted.npz')['state'])
    # Latest evidence replaces the fixed slot: no unbounded snapshot backlog.
    save_failure_snapshot(ctl,dict(reason='certified_candidate'),evidence_slot='accepted')
    assert len(list(tmp_path.glob('*.npz')))==11


def test_fast_straight_backup_preserves_PVA_and_all_hard_bounds():
    c,g=fixture();p=np.array([0.,0.,2.]);v=np.array([.33,0.,0.]);a=np.array([.06,0.,0.])
    base,original=explicit_backup(g,p,v,a,c)
    c['explicit_backup_duration_optimisation']=True
    faster,detail=explicit_backup(g,p,v,a,c)
    assert faster is not None and faster.duration<base.duration
    np.testing.assert_allclose(faster.coeff[0],p)
    np.testing.assert_allclose(faster.coeff[1],v)
    np.testing.assert_allclose(2*faster.coeff[2],a)
    t=faster.duration
    velocity=sum(k*t**(k-1)*faster.coeff[k] for k in range(1,6))
    acceleration=sum(k*(k-1)*t**(k-2)*faster.coeff[k] for k in range(2,6))
    np.testing.assert_allclose(velocity,0.,atol=1e-8)
    np.testing.assert_allclose(acceleration,0.,atol=1e-8)
    assert faster.path_safe(g) and detail['candidate_count']==3
    for key,limit in (('speed_mps','speed'),('acceleration_mps2','acceleration'),('jerk_mps3','jerk')):
        assert detail['derivative_bounds'][key]<=c[limit]+1e-8
    assert detail['search_mode']=='bounded_straight_duration_search'


def test_fast_backup_still_rejects_occupied_start_and_keeps_zero_speed_bounded():
    c,g=fixture();c['explicit_backup_duration_optimisation']=True
    p=np.array([0.,0.,2.]);zero=np.zeros(3)
    stop,detail=explicit_backup(g,p,zero,zero,c)
    assert stop is not None and detail['candidate_count']==1
    g.state[tuple(g.indices(p))]=OCCUPIED;g.rebuild(0.)
    stop,detail=explicit_backup(g,p,[.33,0.,0.],zero,c)
    assert stop is None and not detail['start_safe']
