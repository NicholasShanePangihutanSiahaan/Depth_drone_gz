"""Recovery while physically moving: C2 boundary, live evidence and fallback."""
import copy
from concurrent.futures import Future
import numpy as np
import pytest
from test_continuous import setup
from polinasi_nav.survey import SurveyController
from polinasi_nav.moving_trajectory import BoundaryTrajectory
from polinasi_nav.depth_recovery import adopt, request
from polinasi_nav.depth_health import DepthMotionRecovery
from polinasi_nav.planning import Brake
from polinasi_nav.mapping import OCCUPIED,UNKNOWN


def fixture(moving_reference=True):
    c,g=setup()
    c.update(map_timeout=5.,depth_camera_timeout=5.,depth_moving_recovery=True,
             survey_waypoints=[[3.,0.,2.]],continuous_survey=True)
    ctl=SurveyController(c,g,asynchronous=False)
    ctl.state='BRAKE';ctl.resume_state='SURVEY';ctl.tree_phase='APPROACH'
    ctl.failure='depth_recovering';ctl.command=np.array([0.,0.,2.])
    ctl.command_v=np.array([.1,0.,0.]) if moving_reference else np.zeros(3)
    ctl.goal=np.array([3.,0.,2.]);ctl.command_a=np.zeros(3)
    ctl.health.reason=lambda *args,**kwargs:None
    ctl.grid.rebuild(10.5)
    ctl.grid.last_observation_stamp=10.5
    ctl.depth_recovery_observation=(10.5,dict(last_integrated_stamp=10.4,last_valid_stamp=10.4),5.,10.)
    clone=ctl._planning_clone();clone.state='SURVEY';clone.failure=''
    clone.trajectory=BoundaryTrajectory([ctl.command,ctl.goal],c,ctl.command_v,ctl.command_a)
    ctl.depth_recovery_key=dict(brake=None,elapsed=None,submitted=10.2,episode=10.,
        state='SURVEY',tree_phase='APPROACH',index=0)
    ctl.depth_recovery_future=Future();ctl.depth_recovery_future.set_result((clone,True))
    return ctl,clone


@pytest.mark.parametrize('moving_reference',[True,False])
def test_can_resume_with_nonzero_actual_velocity_without_reference_jump(moving_reference):
    ctl,clone=fixture(moving_reference)
    before=tuple(a.copy() for a in (ctl.command,ctl.command_v,ctl.command_a))
    measured=ctl.command+np.array([.01,0.,0.]);velocity=np.array([.06,0.,0.])
    assert adopt(ctl,10.5,measured,velocity)
    assert not ctl.failure and ctl.state=='SURVEY'
    for a,b in zip(before,ctl.trajectory.sample(0.)):np.testing.assert_allclose(a,b,atol=1e-9)
    assert ctl.depth_recovery_status['actual_speed_mps']>.04
    guard=DepthMotionRecovery();guard.started=10.
    assert guard.update(10.55,dict(last_integrated_stamp=10.4,last_valid_stamp=10.4),ctl,measured,velocity,5.)=='resumed_moving_replan'
    assert guard.started is None


@pytest.mark.parametrize('cause,reason',[
    ('occupied','new_map_invalidated_braking_certificate'),('unknown','new_map_invalidated_braking_certificate'),
    ('stale','stale_result'),('depth','depth_not_fresh_after_brake'),('health','sensor_health'),
    ('tracking','tracking_error'),('phase','phase_or_anchor_changed'),('boundary','PVA_boundary_mismatch'),
    ('window','map_window_changed'),('unready','planner_not_ready_at_boundary'),
    ('map','map_stale'),('actual','actual_body_blocked')])
def test_bad_results_keep_recovery_brake(cause,reason):
    ctl,clone=fixture();p=ctl.command.copy();now=10.5
    if cause in ('occupied','unknown'):
        ctl.grid.state[tuple(ctl.grid.indices([1.,0.,2.]))]=OCCUPIED if cause=='occupied' else UNKNOWN
        ctl.grid.rebuild(now)
    elif cause=='stale':now=11.3
    elif cause=='depth':ctl.depth_recovery_observation=(now,dict(last_integrated_stamp=9.,last_valid_stamp=now),5.,10.)
    elif cause=='health':ctl.health.reason=lambda *a: 'stale_lidar'
    elif cause=='tracking':p+=np.array([.5,0.,0.])
    elif cause=='phase':ctl.tree_phase='SCAN'
    elif cause=='boundary':clone.trajectory.controls[0][0,1]+=.01
    elif cause=='window':ctl.grid.lo=ctl.grid.lo+np.array([.1,0.,0.])
    elif cause=='unready':ctl.depth_recovery_future=Future()
    elif cause=='map':ctl.grid.last_observation_stamp=4.
    elif cause=='actual':
        ctl.grid.state[tuple(ctl.grid.indices(p))]=OCCUPIED;ctl.grid.rebuild(now)
    assert not adopt(ctl,now,p,[.06,0.,0.])
    assert ctl.failure=='depth_recovering' and ctl.trajectory is None
    assert ctl.depth_recovery_status['reason']==reason


def test_variable_tick_switches_at_exact_brake_boundary(monkeypatch):
    ctl,_=fixture()
    ctl.command_v=np.array([.1,0.,0.]);ctl.brake=Brake(ctl.command,ctl.command_v,ctl.command_a,ctl.c)
    ctl.depth_recovery_key.update(brake=ctl.brake,elapsed=.021)
    prediction=copy.deepcopy(ctl.brake);prediction.step(.021)
    clone=ctl._planning_clone();clone.state='SURVEY';clone.failure=''
    clone.trajectory=BoundaryTrajectory([prediction.p,ctl.goal],ctl.c,prediction.v,prediction.a)
    ctl.depth_recovery_future=Future();ctl.depth_recovery_future.set_result((clone,True))
    ctl.survey_now=10.5;ctl.recovery_measured_state=(ctl.command.copy(),np.array([.06,0.,0.]))
    calls=[]
    def rest_tick(now,dt,*state):
        calls.append(dt);return ctl.command.copy(),ctl.command_v.copy()
    monkeypatch.setattr(ctl,'tick',rest_tick)
    ctl._braking(.05)
    assert calls==pytest.approx([.029])
    assert not ctl.failure and max(ctl.depth_recovery_status['PVA_jump'])<1e-8


def test_submission_requires_new_evidence_and_does_not_advance_brake():
    ctl,_=fixture();ctl.depth_recovery_future=ctl.depth_recovery_key=None
    ctl.brake=Brake(ctl.command,ctl.command_v,ctl.command_a,ctl.c)
    captured=[]
    class Executor:
        def submit(self,*args):captured.append(args);return Future()
    ctl.executor=Executor()
    request(ctl,10.5,dict(last_integrated_stamp=9.9,last_valid_stamp=10.5),5.,10.)
    assert not captured
    request(ctl,10.5,dict(last_integrated_stamp=10.4,last_valid_stamp=10.4),5.,10.)
    assert len(captured)==1 and ctl.brake.elapsed==0.
    clone=captured[0][1]
    assert clone.state=='SURVEY' and not clone.failure and hasattr(clone,'moving_boundary')
    assert ctl.depth_recovery_key['elapsed']==pytest.approx(.5)


def test_real_worker_certifies_moving_recovery_candidate():
    from polinasi_nav.survey import prepare_survey_snapshot
    ctl,_=fixture();ctl.depth_recovery_future=ctl.depth_recovery_key=None
    ctl.c['depth_recovery_planning_lead']=.05
    class InlineExecutor:
        def submit(self,fn,*args):
            f=Future()
            try:f.set_result(fn(*args))
            except Exception as exc:f.set_exception(exc)
            return f
    ctl.executor=InlineExecutor();ctl.state='HOLD_ABORT';ctl.brake=None
    request(ctl,10.5,dict(last_integrated_stamp=10.4,last_valid_stamp=10.4),5.,10.)
    clone,ready=ctl.depth_recovery_future.result()
    assert ready and not clone.failure
    assert clone.planner_stages_ms['braking_sample_checks']>0
    assert adopt(ctl,10.5,ctl.command.copy(),[.06,0.,0.])


def test_missing_candidate_keeps_original_bounded_recovery_deadline():
    ctl,_=fixture();ctl.depth_recovery_key=ctl.depth_recovery_future=None
    guard=DepthMotionRecovery();guard.started=10.
    assert guard.update(12.05,dict(last_integrated_stamp=12.,last_valid_stamp=12.),ctl,ctl.command,[.06,0,0],5.)=='fail'
    assert guard.status['failure_gate']=='recovery_replan_deadline'
