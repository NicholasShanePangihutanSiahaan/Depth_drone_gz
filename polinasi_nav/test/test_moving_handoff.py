"""No simulator: explicit PVA, live-map and asynchronous handoff regression."""
from concurrent.futures import Future
import numpy as np
import pytest
from test_continuous import setup
from polinasi_nav.continuous import ContinuousTrajectory
from polinasi_nav.moving_trajectory import BoundaryTrajectory
from polinasi_nav.survey import SurveyController,prepare_survey_snapshot
from polinasi_nav.mapping import OCCUPIED,UNKNOWN


def fixture(phase='APPROACH'):
    c,g=setup()
    c.update(moving_handoff_enabled=True,moving_handoff_lead=.75,moving_result_max_age=1.,
             survey_waypoints=[[6.,0.,2.]],survey_batch_points=24)
    ctl=SurveyController(c,g,asynchronous=False)
    ctl.state='RETURN' if phase=='RETURN' else 'SURVEY'
    ctl.tree_phase=phase
    ctl.trajectory=ContinuousTrajectory([[0.,0.,2.],[2.,0.,2.]],c)
    ctl.goal=np.array([2.,0.,2.]);ctl.goal_yaw=0.
    ctl.elapsed=ctl.trajectory.duration*.45
    p,v,a=ctl.trajectory.sample(ctl.elapsed)
    ctl.command=p;ctl.command_v=v;ctl.command_a=a
    clone=ctl._planning_clone()
    clone.command=p.copy();clone.goal=np.array([4.,0.,2.])
    clone.trajectory=BoundaryTrajectory([p,clone.goal],c,v,a)
    clone.batch_end_index=None;clone.batch_knots={}
    future=Future();future.set_result((clone,True))
    ctl.moving_future=future
    ctl.moving_key=dict(trajectory=ctl.trajectory,state=ctl.state,tree_phase=phase,
                        index=ctl.waypoint_index,elapsed=ctl.elapsed,submitted=10.)
    return ctl,clone


def test_retiming_preserves_nonzero_initial_PVA_and_terminal_stop():
    ctl,clone=fixture();tr=clone.trajectory
    initial=tr.sample(0.)
    tr.slow(1.4)
    for a,b in zip(initial,tr.sample(0.)):np.testing.assert_allclose(a,b,atol=1e-9)
    np.testing.assert_allclose(tr.sample(tr.duration)[1:],0.,atol=1e-9)
    assert tr.safe(ctl.grid)
    v,a,j=tr.bounds()
    assert v<=tr.vmax+1e-8 and a<=tr.amax+1e-8 and j<=tr.jmax+1e-8


@pytest.mark.parametrize('phase',['APPROACH','SCAN','RETURN','WAYPOINT'])
def test_shared_handoff_accepts_continuous_PVA_in_each_navigation_phase(phase):
    ctl,clone=fixture(phase);before=ctl.trajectory.sample(ctl.elapsed)
    ctl._moving_update(10.75)
    assert ctl.trajectory is clone.trajectory
    assert ctl.moving_status['accepted']==1 and ctl.moving_status['speed_mps']>.04
    for a,b in zip(before,ctl.trajectory.sample(0.)):np.testing.assert_allclose(a,b,atol=1e-8)
    assert max(ctl.moving_status['PVA_jump'])<1e-8


@pytest.mark.parametrize('cell',[OCCUPIED,UNKNOWN])
def test_live_obstacle_or_unknown_rejects_candidate_and_keeps_stopping_reference(cell):
    ctl,clone=fixture();old=ctl.trajectory
    ctl.grid.state[tuple(ctl.grid.indices([3.,0.,2.]))]=cell
    ctl.grid.rebuild(10.75)
    ctl._moving_update(10.75)
    assert ctl.trajectory is old and not ctl.failure
    assert ctl.moving_status['reason']=='new_map_invalidated_curve'
    np.testing.assert_allclose(old.sample(old.duration)[1:],0.,atol=1e-9)


@pytest.mark.parametrize('cause,reason',[
    ('stale','stale_result'),('late','handoff_deadline_missed'),
    ('anchors','measured_anchor_progress_mismatch'),('adaptive','reference_or_phase_changed'),
    ('phase','reference_or_phase_changed'),('boundary','PVA_boundary_mismatch'),
    ('exception','planner_exception'),('unfinished','planner_not_ready_at_boundary')])
def test_rejected_async_results_never_replace_reference(cause,reason):
    ctl,clone=fixture();old=ctl.trajectory;now=10.75
    if cause=='stale':now=11.1
    elif cause=='late':ctl.elapsed+=.001
    elif cause=='anchors':ctl.waypoint_index+=1
    elif cause=='adaptive':ctl.phase.rate=.9
    elif cause=='phase':ctl.tree_phase='ASCEND'
    elif cause=='boundary':clone.trajectory.controls[0][0,1]+=.1
    elif cause=='exception':
        ctl.moving_future=Future();ctl.moving_future.set_exception(ValueError('worker error'))
    elif cause=='unfinished':ctl.moving_future=Future()
    ctl._moving_update(now)
    assert ctl.trajectory is old and not ctl.failure
    assert ctl.moving_status['reason']==reason


def test_variable_tick_crosses_exact_boundary_and_advances_remaining_time():
    ctl,clone=fixture();boundary=ctl.elapsed
    ctl.elapsed=boundary-.021
    elapsed=ctl._moving_crossing(boundary+.032,10.75)
    assert ctl.trajectory is clone.trajectory
    assert elapsed==pytest.approx(.032)
    assert max(ctl.moving_status['PVA_jump'])<1e-8


def test_real_snapshot_planner_preserves_future_boundary_and_braking_checks():
    ctl,_=fixture();clone=ctl._planning_clone()
    clone.moving_boundary=(ctl.command_v.copy(),ctl.command_a.copy())
    clone.goal=clone.trajectory=None;clone.waypoints=np.array([[4.,0.,2.]])
    clone.batch_end_index=None;clone.waypoint_index=0
    result,ready=prepare_survey_snapshot(clone,10.75,clone.command,None,None)
    assert ready and not result.failure and isinstance(result.trajectory,BoundaryTrajectory)
    assert result.planner_stages_ms['braking_sample_checks']>0
    for a,b in zip((ctl.command,ctl.command_v,ctl.command_a),result.trajectory.sample(0.)):
        np.testing.assert_allclose(a,b,atol=1e-8)


def test_history_intermediate_endpoint_cannot_count_as_survey_waypoint():
    ctl,_=fixture();ctl.c['history_route_enabled']=True
    ctl.batch_end_index=None;ctl.survey_now=10.75
    ctl._advance()
    assert ctl.waypoint_index==0 and ctl.wait_for_scan is None and ctl.state=='SURVEY'


@pytest.mark.parametrize('state,phase,limit',[
    ('SURVEY','APPROACH',.4),('RETURN','SEARCH',.4),
    ('SURVEY','SCAN',.3),('TAKEOFF','SEARCH',.3)])
def test_phase_speed_profile_raises_transit_only(state,phase,limit):
    c,g=setup();c.update(command_speed=.4,navigation_transit_speed=.4,
                        navigation_scan_speed=.3,navigation_default_speed=.3)
    ctl=SurveyController(c,g,asynchronous=False);ctl.state=state;ctl.tree_phase=phase
    tr=ctl._trajectory_from_points([[0.,0.,2.],[4.,0.,2.]])
    peak=max(np.linalg.norm(tr.sample(t)[1]) for t in np.linspace(0.,tr.duration,101))
    assert peak<=limit+1e-8 and peak>.95*limit
    assert tr.safe(g)


def test_full_control_tick_handoff_with_jitter_preserves_command_safety():
    from polinasi_nav.planning import Health,brake_safe
    ctl,clone=fixture();boundary=ctl.elapsed
    ctl.elapsed=boundary-.021
    ctl.command,ctl.command_v,ctl.command_a=ctl.trajectory.sample(ctl.elapsed)
    previous=ctl.command.copy();ctl.health=Health(10.75,10.75,10.75,True)
    ctl.grid.last_observation_stamp=10.75
    ctl.tick(10.75,.053,ctl.command.copy(),ctl.command_v.copy())
    assert not ctl.failure and ctl.moving_status['accepted']==1
    assert ctl.elapsed==pytest.approx(.032)
    assert ctl.grid.line_safe(previous,ctl.command)
    assert brake_safe(ctl.grid,ctl.command,ctl.command_v,ctl.c,ctl.command_a)
    assert np.linalg.norm(ctl.command_v)>.04
    assert np.linalg.norm(ctl.command_v)<=ctl.c['command_speed']+1e-8
    assert np.linalg.norm(ctl.command_a)<=ctl.c['acceleration']+1e-8
    assert np.linalg.norm(ctl.command_jerk)<=ctl.c['jerk']+1e-8


def test_controller_replay_extends_history_prefix_while_moving_and_reaches_goal():
    from polinasi_nav.planning import Health
    class ImmediateExecutor:
        def submit(self,fn,*args):
            f=Future()
            try:f.set_result(fn(*args))
            except Exception as exc:f.set_exception(exc)
            return f
    c,g=setup();c.update(survey_waypoints=[[6.,0.,2.]],history_route_enabled=True,
        history_route_local_lookahead=4.,history_route_max_age=8.,moving_handoff_enabled=True,
        moving_handoff_lead=.75,moving_result_max_age=1.,survey_prefetch=False,
        navigation_transit_speed=.3)
    ctl=SurveyController(c,g,asynchronous=False);ctl.executor=ImmediateExecutor()
    ctl.state='SURVEY';ctl.command=np.array([0.,0.,2.])
    for i in range(2500):
        now=i*.05;ctl.health=Health(now,now,now,True);g.last_observation_stamp=now
        ctl.global_route=dict(goal=[6.,0.,2.],path=[[0.,0.,2.],[6.,0.,2.]],stamp=now,
                              partial=False,request_id=i)
        ctl.tick(now,.05,ctl.command.copy(),ctl.command_v.copy())
        if ctl.state!='SURVEY' or ctl.failure:break
    assert not ctl.failure
    assert ctl.state=='RETURN' and ctl.waypoint_index==1
    stopped_legs=sum(ContinuousTrajectory([[x,0.,2.],[x+2.,0.,2.]],c).duration for x in (0.,2.,4.))
    assert now<stopped_legs  # Replay only: excludes localisation/FCU/camera latency.
    assert ctl.moving_status['accepted']>=1
    assert ctl.moving_status['speed_mps']>.04
    assert max(ctl.moving_status['PVA_jump'])<1e-8
    assert np.linalg.norm(ctl.command-[6.,0.,2.])<.12
