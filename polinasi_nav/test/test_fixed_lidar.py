"""Fixed sensor fixture regression: geometry and controller policy."""
import sys
from pathlib import Path
import xml.etree.ElementTree as ET
import pytest
from polinasi_nav.config import load_config
from polinasi_nav.continuous_tree import ContinuousTreeInspectionController
from polinasi_nav.mapping import VoxelMap

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'tools'))
from generate_fixed_lidar import main as generate
from prepare_sim_world import prepare


def test_fixed_config_and_world(tmp_path):
    generate()
    path = ROOT / 'reports/fixed_lidar/inspection.json'
    c = load_config(path)
    assert c['lidar_fixed'] and not c['simulation_continuous_roll']
    assert c['fixed_fov_assumptions'] and c['scan_adaptive_height'] and c['scan_target_repair']
    assert c['camera_mount_yaw'] == 0 and not c['camera_yaw_enabled']
    assert c['depth_camera_enabled'] and c['tree_exploration_same_altitude']
    assert c['map_timeout'] == 5. and c['depth_camera_timeout'] == 5.
    hover = load_config(ROOT / 'reports/fixed_lidar/hover.json')
    assert hover['map_timeout'] == 5.
    prepare(ROOT/'polinasi_nav/worlds/palm_farm.sdf',tmp_path/'world.sdf',.15,path)
    model = ET.parse(tmp_path/'models/iris_lidar_mapping/model.sdf').getroot().find('model')
    assert model.find("joint[@name='lidar_pitch']").get('type') == 'fixed'
    assert model.find("joint[@name='lidar_pitch']/axis") is None
    assert model.find("joint[@name='lidar_roll']") is None
    assert model.find("joint[@name='camera_yaw']").get('type') == 'fixed'
    assert model.find("joint[@name='camera_pitch']").get('type') == 'revolute'
    assert model.find(".//sensor[@name='zed_placeholder']").get('type')=='rgbd_camera'
    assert model.find("link[@name='mid360_link']/pose").text == '0.0 0.0 0.28 0.0 0.0 0.0'
    ctl = ContinuousTreeInspectionController(c,VoxelMap(c),sensor_mode=True,asynchronous=False)
    ctl.active_tree={'xy':[0.,0.]}
    assert abs(ctl._camera_heading([2.,0.,2.])) == pytest.approx(3.141592653589793)
    ctl.tree_phase='SCAN'
    assert ctl._same_altitude_exploration()
    assert not ctl.camera_navigation_view()
    ctl.navigation_camera_active=True
    assert ctl.camera_navigation_view()


def test_fixed_rejects_moving_lidar(tmp_path):
    import json
    generate()
    c=json.loads((ROOT/'reports/fixed_lidar/inspection.json').read_text())
    c['automatic_tilt_ascent']=True
    path=tmp_path/'bad.json';path.write_text(json.dumps(c))
    with pytest.raises(ValueError,match='fixed sensor'):
        load_config(path)
