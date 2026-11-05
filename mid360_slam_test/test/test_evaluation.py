import json
import numpy as np
import pytest
from scipy.spatial.transform import Rotation
from mid360_slam_test.evaluation import Evaluation
from mid360_slam_test.geometry import pose,pack


def status(t,x=0,y=0,z=0,yaw=0,valid=True):
    p=pack(pose([x,y,z],Rotation.from_euler('z',yaw,degrees=True).as_quat()))
    return dict(stamp=t,local=p,mapped=p,valid=valid,tracking='TRACKING' if valid else 'LOST',
                reason='ok' if valid else 'registration_failed',reset_counter=0)


def ready(engine,start=1.,x=0,yaw=0):
    for i in range(15):
        engine.receive(status(start+i*.1,x=x,yaw=yaw),start+i*.1)
    assert engine.settled(start+1.4)
    return start+1.4


def command(engine,action,t,**kwargs):
    result=engine.command(dict(id=str(t)+action,action=action,**kwargs),t)
    assert result['ok'],result
    return result


def test_no_data_never_can_start():
    e=Evaluation()
    assert not e.view(1)['can_start']
    result=e.command(dict(id='1',action='start',reference_confirmed=True),1)
    assert not result['ok'] and not e.results


def test_stationary_auto_finishes_measures_drift_and_missing_limits_not_pass():
    e=Evaluation(); t=ready(e)
    command(e,'reference',t,duration_s=1.)
    command(e,'start',t,reference_confirmed=True)
    for i in range(12):
        now=t+(i+1)*.1
        e.receive(status(now,x=i*.001),now);e.tick(now)
    assert e.phase=='REVIEW'
    r=e.results[-1]
    assert r['stationary_max_position_change_m']>.005
    assert r['status']=='NOT TESTED'
    json.dumps(e.view(now),allow_nan=False)


def test_translation_reference_relative_to_stage_orientation_and_finish_settle():
    e=Evaluation(dict(max_position_error_m=.05)); e.index=1
    t=ready(e,x=10,yaw=90)
    command(e,'start',t,reference_confirmed=True)
    for i in range(1,11): e.receive(status(t+i*.1,x=10,y=i*.1,yaw=90),t+i*.1)
    command(e,'finish',t+1.)
    for i in range(1,16):
        now=t+1+i*.1
        e.receive(status(now,x=10,y=1.,yaw=90),now);e.tick(now)
    r=e.results[-1]
    assert r['status']=='PASS' and r['position_error_m']<1e-9
    assert r['displacement_start_sensor_axes_m']==pytest.approx([1,0,0])


def test_command_retry_idempotence_and_reference_confirmation():
    e=Evaluation();t=ready(e)
    assert not e.command(dict(id='a',action='start'),t)['ok']
    c=dict(id='b',action='start',reference_confirmed=True)
    first=e.command(c,t)
    assert e.command(c,t+.2)==first
    assert e.active['started_wall']==t


def test_rotation_unwraps_full_turn():
    e=Evaluation(dict(max_angle_error_deg=2.));e.index=7
    t=ready(e); command(e,'start',t,reference_confirmed=True)
    for i in range(1,37):
        e.receive(status(t+i*.1,yaw=i*10),t+i*.1)
    command(e,'finish',t+3.6)
    for i in range(1,16):
        now=t+3.6+i*.1;e.receive(status(now,yaw=360),now);e.tick(now)
    assert e.results[-1]['accumulated_yaw_deg']==pytest.approx(360)
    assert e.results[-1]['status']=='PASS'


def test_loss_invalidates_stage_and_repeat_preserves_failed_attempt():
    e=Evaluation();t=ready(e);command(e,'start',t,reference_confirmed=True)
    e.receive(status(t+.1,valid=False),t+.1);e.tick(t+.1)
    assert e.results[-1]['status']=='FAIL' and e.phase=='REVIEW'
    command(e,'repeat',t+.2)
    assert e.phase=='READY' and len(e.results)==1


def test_skip_and_next_are_explicit_and_keep_estimator_reference():
    e=Evaluation();command(e,'skip',1); command(e,'next',2)
    assert e.index==1 and e.results[0]['status']=='NOT TESTED'
    assert e.phase=='READY' and not e.view(2)['can_start']


def test_proximity_is_not_loop_detection():
    e=Evaluation(dict(loop_wait_s=1.));e.index=8
    t=ready(e);command(e,'start',t,reference_confirmed=True)
    for i in range(1,12):e.receive(status(t+i*.1),t+i*.1)
    command(e,'finish',t+1.1)
    for i in range(1,16):
        now=t+1.1+i*.1;e.receive(status(now),now);e.tick(now)
    assert e.results[-1]['loop_detection_status']=='NOT_DETECTED'
    assert e.results[-1]['status']=='FAIL'


def test_loop_evidence_recorded_and_map_start_uses_same_revision():
    e=Evaluation(dict(loop_wait_s=1.,max_position_error_m=.05,max_angle_error_deg=2.));e.index=8
    t=ready(e);command(e,'start',t,reference_confirmed=True)
    local_start=pack(pose())
    map_start=pack(pose([.3,0,0]))
    event=dict(sequence=1,source='experimental_pose_graph',stamp=t+.5,accepted=True,
        state='ACCEPTED',graph_solution=[dict(stamp=t,local=local_start,mapped=map_start)])
    e.receive_loop(event,t+.5)
    e.receive_loop(event,t+.6) # duplicate must not count twice
    command(e,'finish',t+.5)
    for i in range(1,16):
        now=t+.5+i*.1;s=status(now)
        s['mapped']=map_start;e.receive(s,now);e.tick(now)
    r=e.results[-1]
    assert len(r['loop_events'])==1
    assert r['mapped_endpoint_error_m']==pytest.approx(0)
    assert r['loop_detection_status']=='DETECTED' and r['status']=='PASS'


def test_reset_counter_and_unordered_messages_fail_stage():
    e=Evaluation();t=ready(e);command(e,'start',t,reference_confirmed=True)
    s=status(t+.1);s['reset_counter']=1;e.receive(s,t+.1)
    e.receive(status(t),t+.2)
    e.tick(t+2.)
    assert e.results[-1]['status']=='FAIL'
    assert 'estimator_reset_during_stage' in e.results[-1]['invalid_reasons']
