import json
from pathlib import Path
import numpy as np
from polinasi_nav.tree_perception import StemDetector
from polinasi_nav.tree_mission import TreeInspectionController
from polinasi_nav.config import load_config
from polinasi_nav.mapping import VoxelMap,FREE


def cylinder(x,y):
    angle,z = np.meshgrid(np.linspace(-1.2,1.2,30),np.linspace(.8,2.7,12))
    stem = np.column_stack((x+.25*np.cos(angle.ravel()),y+.25*np.sin(angle.ravel()),z.ravel()))
    floor = np.column_stack((np.linspace(-5,5,100),np.zeros(100),np.zeros(100)))
    return np.vstack((stem,floor))


def test_measured_stem_confirmation_and_id_without_layout():
    detector = StemDetector()
    for i in range(4):
        result = detector.update(cylinder(3,2),float(i))
    confirmed = [t for t in result['tracks'] if t['confirmed']]
    assert len(confirmed)==1
    np.testing.assert_allclose(confirmed[0]['xy'],[3,2],atol=.1)
    assert not confirmed[0]['stem_top_complete']
    json.dumps(result,allow_nan=False)


def test_ground_alone_not_a_tree():
    detector = StemDetector()
    assert not detector.update(cylinder(3,2)[-100:],1.)['tracks']


def test_close_approach_config_uses_two_metre_radius_and_68_degree_tilt():
    c = load_config(Path(__file__).resolve().parents[1]/'config/tilt_inspection.json')
    assert c['tree_view_distances'] == [2.]
    np.testing.assert_allclose(c['tree_tilt_angles'], [np.deg2rad(68)])
    assert c['tree_view_count']==6 and not c['camera_yaw_enabled']
    c.update(bounds_min=[-5.,-5.,0.],bounds_max=[5.,5.,6.])
    grid = VoxelMap(c)
    grid.state[:] = FREE
    grid.seen[:] = 0.
    grid.rebuild(0.)
    ctl = TreeInspectionController(c,grid,asynchronous=False)
    ctl.state = 'SURVEY'
    ctl.survey_now = 3.
    ctl.receive_trees(dict(stamp=3.,frame='map',source='measured_lidar_geometry',tracks=[
        dict(id=7,xy=[3.,0.],last_seen=3.,confirmed=True,observed_stem_top=5.)]))
    assert ctl._choose_goal([0.,0.,2.])
    assert abs(np.linalg.norm(ctl.view_target[:2]-[3.,0.])-2.) <= np.sqrt(2)*c['resolution']/2
    assert abs(ctl.view_target[2]-2.) <= c['resolution']/2+1e-9


def test_mission_uses_measured_tree_and_no_buzzer():
    c = load_config(Path(__file__).resolve().parents[1]/'config/tree_inspection.json')
    c.update(bounds_min=[-10.,-10.,0.],bounds_max=[10.,10.,8.])
    grid = VoxelMap(c)
    grid.state[:] = FREE
    grid.seen[:] = 0.
    grid.rebuild(0.)
    ctl = TreeInspectionController(c,grid,asynchronous=False)
    ctl.survey_now = 3.
    assert ctl.preflight_hint() is None
    ctl.receive_trees(dict(stamp=3.,frame='map',source='measured_lidar_geometry',tracks=[
        dict(id=7,xy=[6.,0.],last_seen=3.,confirmed=True,observed_stem_top=3.5)]))
    assert ctl.preflight_hint() is not None
    ctl.state = 'SURVEY'
    assert ctl._choose_goal([0.,0.,2.])
    assert ctl.active_tree['id']==7 and np.linalg.norm(ctl.goal-[0,0,2]) <= 2.00001
    assert isinstance(ctl._planning_clone(),TreeInspectionController)
    ctl.flower_event(True)
    assert ctl.buzzer_events==0
    json.dumps(ctl.inspection_status(),allow_nan=False)


def test_inspection_requires_new_scan_and_advances_only_at_view():
    c = load_config(Path(__file__).resolve().parents[1]/'config/tree_inspection.json')
    grid = VoxelMap(c)
    ctl = TreeInspectionController(c,grid,asynchronous=False)
    ctl.state = 'SURVEY'
    ctl.active_tree = dict(id=7,xy=[6.,0.])
    ctl.tree_phase = 'APPROACH'
    ctl.view_target = ctl.command.copy()
    ctl.wait_for_scan = 1.
    grid.last_observation_stamp = 1.
    assert not ctl._prepare_trajectory(1.,ctl.command)
    assert ctl.tree_phase == 'APPROACH'
    grid.last_observation_stamp = 2.
    ctl._prepare_trajectory(2.,ctl.command)
    assert ctl.tree_phase == 'SCAN' and not ctl.scan_records
    for i in range(c['tree_view_count']):
        ctl.view_target = ctl.command.copy()
        ctl.wait_for_scan = 2.+i
        grid.last_observation_stamp = 3.+i
        ctl._prepare_trajectory(3.+i,ctl.command)
    assert ctl.completed_trees == [7]
    assert len(ctl.scan_records) == c['tree_view_count']
    assert ctl.tree_phase == 'SEARCH' and ctl.active_tree is None
    ctl.survey_now = 20.
    ctl.receive_trees(dict(stamp=20.,frame='map',source='measured_lidar_geometry',tracks=[
        dict(id=99,xy=[6.1,0.],last_seen=20.,confirmed=True,observed_stem_top=3.5)]))
    assert not ctl.candidates()  # A recreated detector ID must not repeat an inspected tree.


def test_body_faces_scan_motion_not_tree_with_front_camera():
    c = load_config(Path(__file__).resolve().parents[1]/'config/tree_inspection.json')
    c['scan_body_faces_path'] = True
    ctl = TreeInspectionController(c,VoxelMap(c),asynchronous=False)
    ctl.active_tree = dict(id=1,xy=[-6.,0.])
    ctl.command_v = np.array([0.,.2,0.])
    ctl._on_motion(1.,np.array([0.,0.,2.]))
    assert ctl.goal_yaw == np.pi/2  # Northward path; tree is westward.
    ctl.command_v[:] = 0.
    ctl.yaw = .7
    ctl._on_motion(2.,np.array([0.,0.,2.]))
    assert ctl.goal_yaw == .7


def test_auto_tilt_waits_for_settled_mount_and_new_map_then_blocks_unknown_ascent():
    c = load_config(Path(__file__).resolve().parents[1]/'config/tilt_inspection.json')
    grid = VoxelMap(c)
    ctl = TreeInspectionController(c,grid,asynchronous=False)
    ctl.state = 'SURVEY'
    ctl.active_tree = dict(id=1,xy=[6.,0.],observed_stem_top=5.)
    ctl.command = np.array([0.,0.,2.])
    ctl.tree_phase = 'UPWARD_OBSERVE'
    ctl.mount_phase_started = 1.
    ctl.mount_feedback = dict(feedback_fresh=True,measured={'lidar_pitch':c['tree_tilt_angles'][0]})
    ctl._prepare_trajectory(1.,ctl.command)
    ctl._prepare_trajectory(2.,ctl.command)
    assert ctl.mount_angle_index == 0  # No fresh integrated map.
    grid.last_observation_stamp = 2.
    ctl._prepare_trajectory(2.1,ctl.command)
    assert ctl.tree_phase=='ASCEND'
    ctl._prepare_trajectory(4.1,ctl.command)
    assert ctl.tree_phase=='NORMALIZE' and ctl.ascent_stop_reason=='next_step_not_certified_free'
    assert ctl.lidar_target()==0.
    ctl.mount_feedback.update(navigation_ready=True,measured={'lidar_pitch':0.})
    ctl._prepare_trajectory(5.,ctl.command)
    grid.last_observation_stamp = 6.
    ctl._prepare_trajectory(6.,ctl.command)
    assert ctl.tree_phase=='SCAN' and ctl.inspection_altitude==2.


def test_six_view_scan_turns_only_at_arrival_and_waits_for_pitch_and_fresh_scan():
    c = load_config(Path(__file__).resolve().parents[1]/'config/tilt_inspection.json')
    grid = VoxelMap(c)
    ctl = TreeInspectionController(c,grid,asynchronous=False)
    ctl.state,ctl.tree_phase = 'SURVEY','SCAN'
    ctl.active_tree = dict(id=1,xy=[0.,3.],observed_stem_top=4.)
    ctl.command = np.array([0.,0.,2.])
    ctl.view_target = ctl.command.copy()
    ctl.command_v = np.array([.2,0.,0.])
    ctl._on_motion(1.,ctl.command)
    assert ctl.goal_yaw==0. and not ctl.inspection_hover()
    ctl.wait_for_scan = 1.
    ctl.command_v[:] = 0.
    ctl.mount_feedback = dict(feedback_fresh=True,measured={'camera_pitch':-.4},targets={'camera_pitch':-.4})
    grid.last_observation_stamp = 10.
    ctl.vehicle_speed = .06
    ctl._prepare_trajectory(1.5,ctl.command)
    assert not ctl.scan_records and ctl.wait_for_scan==1.
    ctl.vehicle_speed = 0.
    ctl._prepare_trajectory(2.,ctl.command)
    ctl._prepare_trajectory(3.,ctl.command)
    assert ctl.yaw>0. and not ctl.scan_records
    ctl.yaw = ctl.body_yaw_measured = np.pi/2
    ctl.mount_feedback.update(measured={'camera_pitch':.5},targets={'camera_pitch':.5})
    ctl._prepare_trajectory(4.,ctl.command)
    grid.last_observation_stamp = 4.
    ctl._prepare_trajectory(5.,ctl.command)
    assert not ctl.scan_records  # A scan before settling must not count.
    grid.last_observation_stamp = 5.1
    ctl._prepare_trajectory(5.1,ctl.command)
    assert ctl.inspection_view==1 and len(ctl.scan_records)==1
    assert ctl.view_settled_since is None
