from polinasi_nav.depth_health import DepthHoldGuard,fresh_depth
from polinasi_nav.depth_health import DepthMotionRecovery
import numpy as np
from pathlib import Path
from polinasi_nav.config import load_config
from polinasi_nav.mapping import VoxelMap, FREE
from polinasi_nav.tree_mission import TreeInspectionController


def moving_controller():
    c = load_config(Path(__file__).resolve().parents[1]/'config/tilt_inspection.json')
    c.update(bounds_min=[-5.,-5.,0.],bounds_max=[5.,5.,6.])
    grid = VoxelMap(c)
    grid.state[:] = FREE
    grid.seen[:] = 0.
    grid.rebuild(0.)
    ctl = TreeInspectionController(c,grid,asynchronous=False)
    ctl.state,ctl.tree_phase = 'SURVEY','SCAN'
    ctl.command = np.array([0.,0.,2.])
    ctl.command_v = np.array([.1,0.,0.])
    return ctl


def test_exact_timeout_is_not_roundoff_fault():
    assert fresh_depth(255.4,254.4,1.)
    assert not fresh_depth(255.45,254.4,1.)


def test_stale_integration_can_only_wait_while_safe_stationary():
    guard = DepthHoldGuard()
    depth = dict(last_integrated_stamp=3.,last_valid_stamp=4.)
    assert guard.assess(4.2,depth,True,1.)=='hold_wait_for_integration'
    assert guard.assess(4.3,depth,False,1.)=='fail'
    depth['last_integrated_stamp'] = 4.2
    assert guard.assess(4.4,depth,True,1.)=='fresh'
    assert guard.wait_since is None


def test_receiving_valid_frames_does_not_bypass_bounded_hold_timeout():
    guard = DepthHoldGuard()
    for now in (4.2,5.,6.):
        assert guard.assess(now,dict(last_integrated_stamp=3.,last_valid_stamp=now),True,1.)=='hold_wait_for_integration'
    assert guard.assess(6.3,dict(last_integrated_stamp=3.,last_valid_stamp=6.3),True,1.)=='fail'
    assert DepthHoldGuard().assess(4.2,dict(last_integrated_stamp=3.,last_valid_stamp=3.),True,1.)=='fail'


def test_depth_motion_recovery_brakes_then_requires_new_map_and_actual_settling():
    ctl = moving_controller()
    guard = DepthMotionRecovery()
    depth = dict(last_integrated_stamp=3.,last_valid_stamp=3.6)
    assert guard.update(3.7,depth,ctl,ctl.command,ctl.command_v,1.)=='certified_brake_started'
    assert ctl.failure=='depth_recovering' and ctl.state=='BRAKE'
    depth.update(last_integrated_stamp=3.8,last_valid_stamp=3.8)
    assert guard.update(3.9,depth,ctl,ctl.command,ctl.command_v,1.)=='braking_or_waiting'
    for _ in range(100):
        ctl._braking(.05)
        if ctl.brake is None:
            break
    assert ctl.state=='HOLD_ABORT'
    assert guard.update(4.,depth,ctl,ctl.command,np.array([.05,0.,0.]),1.)=='braking_or_waiting'
    assert guard.update(4.,depth,ctl,ctl.command,np.zeros(3),1.)=='resumed_replan'
    assert not ctl.failure and ctl.state=='SURVEY' and ctl.trajectory is None


def test_depth_motion_recovery_timeout_and_sensor_loss_are_terminal():
    for now,valid in ((5.8,5.8),(4.8,3.6)):
        ctl = moving_controller()
        guard = DepthMotionRecovery()
        depth = dict(last_integrated_stamp=3.,last_valid_stamp=3.6)
        guard.update(3.7,depth,ctl,ctl.command,ctl.command_v,1.)
        depth['last_valid_stamp'] = valid
        assert guard.update(now,depth,ctl,ctl.command,ctl.command_v,1.)=='fail'
        assert ctl.failure=='stale_depth_camera'


def test_depth_recovery_does_not_override_other_failure_or_unsafe_brake():
    ctl = moving_controller()
    guard = DepthMotionRecovery()
    depth = dict(last_integrated_stamp=3.,last_valid_stamp=3.6)
    guard.update(3.7,depth,ctl,ctl.command,ctl.command_v,1.)
    ctl.failure = 'stale_lidar'
    assert guard.update(3.8,depth,ctl,ctl.command,ctl.command_v,1.)=='superseded_by_failure'
    assert ctl.failure=='stale_lidar'
    ctl = moving_controller()
    ctl.grid.blocked[:] = True
    assert DepthMotionRecovery().update(3.7,depth,ctl,ctl.command,ctl.command_v,1.)=='fail'
    assert ctl.failure=='insufficient_depth_recovery_stopping_clearance'



def test_preventive_depth_threshold_uses_same_roundoff_tolerance_as_freshness():
    ctl=moving_controller();guard=DepthMotionRecovery()
    depth=dict(last_integrated_stamp=32.4,last_valid_stamp=32.4)
    assert 33.-32.4>.6  # Exact decimal threshold expressed in binary floats.
    assert guard.update(33.,depth,ctl,ctl.command,ctl.command_v,1.)=='inactive'
    assert not ctl.failure
    assert guard.update(33.05,depth,ctl,ctl.command,ctl.command_v,1.)=='certified_brake_started'
    assert ctl.failure=='depth_recovering'


def test_fresh_depth_does_not_override_settling_deadline_and_reason_is_explicit():
    ctl=moving_controller();guard=DepthMotionRecovery()
    guard.update(3.7,dict(last_integrated_stamp=3.,last_valid_stamp=3.6),ctl,ctl.command,ctl.command_v,1.)
    depth=dict(last_integrated_stamp=5.6,last_valid_stamp=5.6)
    assert guard.update(5.75,depth,ctl,ctl.command,[.074,0.,0.],1.)=='fail'
    assert guard.status['failure_gate']=='recovery_settling_deadline'
    assert guard.status['integrated_age']<.6 and guard.status['actual_speed_mps']>.04
    assert ctl.failure=='stale_depth_camera'
