import math
from pathlib import Path
import numpy as np
import pytest
from polinasi_nav.config import load_config
from polinasi_nav.mapping import VoxelMap, FREE
from polinasi_nav.continuous_tree import ContinuousTreeInspectionController


def controller():
    c=load_config(Path(__file__).resolve().parents[1]/'config/tree_inspection.json')
    c.update(simulation_continuous_roll=True,continuous_tree_scanning=True,
             camera_mount_yaw=math.pi/2,tree_view_count=6,tree_scan_anchors=24,
             bounds_min=[-6.,-6.,0.],bounds_max=[6.,6.,6.])
    grid=VoxelMap(c)
    grid.state[:]=FREE;grid.seen[:]=0.;grid.rebuild(0.)
    ctl=ContinuousTreeInspectionController(c,grid,asynchronous=False)
    ctl.active_tree=dict(id=7,xy=[0.,0.])
    ctl.state='SURVEY';ctl.tree_phase='SCAN'
    ctl._begin_continuous_scan(np.array([2.,0.,2.]))
    return ctl


def test_circle_is_constant_height_ccw_and_left_camera_faces_tree():
    ctl=controller()
    np.testing.assert_allclose(np.linalg.norm(ctl.waypoints[:,:2],axis=1),2.)
    np.testing.assert_allclose(ctl.waypoints[:,2],2.)
    assert ctl.waypoints[0,1]>0
    assert ctl._camera_heading([2.,0.,2.])==pytest.approx(math.pi/2)
    np.testing.assert_allclose(ctl.waypoints[-1],[2.,0.,2.],atol=1e-10)


def test_pass_requires_new_map_but_does_not_stop_at_sector():
    ctl=controller();ctl.waypoint_index=3;ctl.batch_end_index=23
    ctl.mount_feedback=dict(feedback_fresh=True,measured={'camera_pitch':.5},targets={'camera_pitch':.5})
    p=ctl.waypoints[3];ctl.body_yaw_measured=ctl._camera_heading(p)
    ctl.command_v=np.array([.1,.2,0.])
    ctl.grid.last_observation_stamp=1.
    ctl._on_motion(1.,p)
    assert ctl.waypoint_index==3
    ctl.grid.last_observation_stamp=1.1
    ctl._on_motion(1.1,p)
    assert ctl.waypoint_index==4 and ctl.wait_for_scan is None
    assert np.linalg.norm(ctl.command_v)>0
    assert ctl.scan_sector_records[0]['inspection_valid']


def test_bad_camera_cannot_claim_sector_success():
    ctl=controller();ctl.mount_feedback=dict(feedback_fresh=False)
    ctl._record_pass(3,1.,ctl.waypoints[3])
    assert not ctl.scan_records and not ctl.scan_sector_records[0]['inspection_valid']


def test_completion_resets_for_next_tree_and_requires_all_sectors():
    ctl=controller();ctl.scan_start_aligned=True
    ctl.mount_feedback=dict(feedback_fresh=True,measured={'camera_pitch':.5},targets={'camera_pitch':.5})
    for i in (3,7,11,15,19):
        ctl.body_yaw_measured=ctl._camera_heading(ctl.waypoints[i])
        ctl._record_pass(i,1.,ctl.waypoints[i])
    ctl.waypoint_index=23;ctl.wait_for_scan=1.;ctl.grid.last_observation_stamp=2.
    ctl.body_yaw_measured=ctl._camera_heading(ctl.waypoints[23])
    ctl._prepare_trajectory(2.,ctl.waypoints[23])
    assert ctl.completed_trees==[7] and ctl.tree_phase=='SEARCH'
    assert ctl.active_tree is None and not ctl.scan_orbit_ready


def test_sensor_localisation_is_not_mislabeled_as_validated():
    ctl=controller()
    with pytest.raises(ValueError):
        ContinuousTreeInspectionController(ctl.c,ctl.grid,sensor_mode=True)


def test_rolling_approach_enters_ascent_not_scan_and_never_tilts_lidar():
    ctl=controller();ctl.c['rolling_canopy_ascent']=True;ctl.c['tree_continuous_ascent']=True
    ctl.tree_phase='APPROACH';ctl.view_target=ctl.command.copy()
    ctl.wait_for_scan=1.;ctl.grid.last_observation_stamp=2.
    ctl._prepare_trajectory(2.,ctl.command)
    assert ctl.tree_phase=='ASCEND' and not ctl.scan_start_aligned
    assert ctl.lidar_target()==0.
    ctl._mount_phase('NORMALIZE',3.)
    assert ctl.tree_phase=='SCAN' and ctl.inspection_altitude==ctl.command[2]


def test_rolling_ascent_uses_bounded_live_reference_and_keeps_xy_fixed():
    ctl=controller();ctl.c['tree_continuous_ascent']=True;ctl.c['rolling_canopy_ascent']=True
    ctl.active_tree['observed_stem_top']=4.
    ctl.tree_phase='ASCEND';ctl.command=np.array([2.,0.,2.])
    for i in range(40):
        now=1.+i*.05;ctl.grid.last_observation_stamp=now
        result=ctl._live_reference(now,.05,ctl.command,ctl.command_v)
        assert result is not None
        ctl.command,ctl.command_v,ctl.command_a,ctl.elapsed=result
        np.testing.assert_allclose(ctl.command[:2],[2.,0.])
        assert not ctl.failure and ctl.tree_phase=='ASCEND'
    assert ctl.command[2]>2.1
