import importlib.util
import json
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
import pytest
from polinasi_nav.depth_mapping import project_depth,approach_camera_target
from polinasi_nav.coverage_mapping import RollingMapper,navigation_window
from polinasi_nav.mapping import FREE,OCCUPIED,UNKNOWN
from polinasi_nav.config import load_config
from polinasi_nav.sensor_mounts import mount_rotation


def test_depth_pinhole_units_invalid_and_budget():
    data = np.array([[2.,np.nan,np.inf],[0.,.2,8.]],dtype='<f4')
    k = [2,0,0,0,2,0,0,0,1]
    points,stats = project_depth(data.tobytes(),3,2,12,'32FC1',False,k,stride=1)
    np.testing.assert_allclose(points,[[0.,0.,2.]])
    assert stats['invalid_or_out_of_range']==5
    mm = np.full((4,4),2000,dtype='>u2')
    points,_ = project_depth(mm.tobytes(),4,4,8,'16UC1',True,k,stride=1,budget=3)
    assert len(points)==3
    np.testing.assert_allclose(points[:,2],2.)
    assert points[-1,0]>points[0,0] and points[-1,1]>points[0,1]


def test_depth_padded_rows_and_reject_intrinsics():
    k = [2,0,0,0,2,0,0,0,1]
    row = np.array([2.,2.,99.],dtype='<f4').tobytes()
    points,_ = project_depth(row*2,2,2,12,'32FC1',False,k,stride=1)
    assert len(points)==4 and np.max(points[:,2])==2.
    with pytest.raises(ValueError):
        project_depth(row,2,2,12,'32FC1',False,k)
    with pytest.raises(ValueError):
        project_depth(row*2,2,2,12,'32FC1',False,[0]*9)


def test_front_down_camera_and_dynamic_optical_rotation():
    target = approach_camera_target([0.,0.,2.],[5.,0.,2.])
    assert target[2]<2.
    optical = np.array([[0,0,1],[-1,0,0],[0,-1,0]])
    # Same measured rotation as dynamic TF: camera centre ray points down.
    ray = mount_rotation(pitch=-np.deg2rad(25))@optical@np.array([0.,0.,2.])
    assert ray[0]>0 and ray[2]<0


def test_two_origins_fuse_preserve_obstacles_and_do_not_refresh_lidar_stamp():
    c = load_config()
    c.update(bounds_min=[-3.,-3.,0.],bounds_max=[5.,3.,5.],navigation_window_size=[8.,6.,5.],
             drone_dimensions=[.2,.2,.2],clearance=.05,tracking_margin=.05)
    worker = RollingMapper(c)
    grid = navigation_window(c,[0.,0.,2.])
    depth = ((np.array([0.,1.,2.]),np.array([[2.,1.,2.]]),.9,None),)
    updated,stamp,_ = worker.integrate(grid,np.array([0.,0.,2.]),np.array([[2.,0.,2.]]),
        [True],1.,None,1.,[0.,0.,2.],auxiliary=depth)
    assert stamp==updated.last_observation_stamp==1.
    assert updated.depth_integrated_stamp==.9
    for p in ([2.,0.,2.],[2.,1.,2.]):
        assert updated.state[tuple(updated.indices(p))]==OCCUPIED
    assert updated.state[tuple(updated.indices([1.,1.,2.]))]==FREE
    assert updated.state[tuple(updated.indices([3.,1.,2.]))]==UNKNOWN
    # A later camera ray across a previously measured obstacle cannot erase it.
    depth = ((np.array([0.,0.,2.]),np.array([[4.,0.,2.]]),1.1,None),)
    updated,_,_ = worker.integrate(updated,np.array([0.,0.,2.]),np.array([[2.,0.,2.]]),
        [True],1.2,None,1.2,[0.,0.,2.],auxiliary=depth)
    assert updated.state[tuple(updated.indices([2.,0.,2.]))]==OCCUPIED


def test_rgbd_generated_only_in_private_world(tmp_path):
    root = Path(__file__).resolve().parents[2]
    spec = importlib.util.spec_from_file_location('depth_fixture',root/'tools/prepare_sim_world.py')
    module = importlib.util.module_from_spec(spec); spec.loader.exec_module(module)
    original = (root/'polinasi_nav/models/iris_lidar/model.sdf').read_bytes()
    cfg = load_config(root/'polinasi_nav/config/tilt_inspection.json')
    path = tmp_path/'config.json'; path.write_text(json.dumps(cfg))
    module.prepare(root/'polinasi_nav/worlds/palm_farm.sdf',tmp_path/'world.sdf',.15,path)
    model = ET.parse(tmp_path/'models/iris_lidar_mapping/model.sdf')
    sensor = model.find(".//sensor[@name='zed_placeholder']")
    assert sensor.get('type')=='rgbd_camera'
    assert sensor.findtext('camera/optical_frame_id')=='zed_camera_optical'
    assert sensor.findtext('topic')=='/zed'
    assert float(sensor.findtext('camera/clip/far'))==cfg['depth_max_range']
    assert model.find(".//joint[@name='camera_yaw']").get('type')=='fixed'
    assert model.find(".//joint[@name='camera_yaw']/axis") is None
    assert not any(p.findtext('joint_name')=='camera_yaw' for p in model.findall('.//plugin'))
    assert (root/'polinasi_nav/models/iris_lidar/model.sdf').read_bytes()==original


def test_long_finite_ground_depth_can_clear_ray_but_invalid_stays_unknown():
    data = np.array([[12.,np.inf,25.]],dtype='<f4')
    k = [2,0,0,0,2,0,0,0,1]
    short,_ = project_depth(data.tobytes(),3,1,12,'32FC1',False,k,stride=1,maximum=8.)
    long,stats = project_depth(data.tobytes(),3,1,12,'32FC1',False,k,stride=1,maximum=20.)
    assert len(short)==0 and len(long)==1
    assert stats['invalid_or_out_of_range']==2
