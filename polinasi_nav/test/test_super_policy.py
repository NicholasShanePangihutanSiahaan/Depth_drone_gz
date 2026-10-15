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
    assert ctl.route_failure_diagnostics==dict(gate='survey_worker',
        exception_type='ValueError',message='no robust route braking envelope')
    ctl.executor.shutdown(wait=True)
