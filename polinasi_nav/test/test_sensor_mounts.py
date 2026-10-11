import importlib.util
import json
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
import pytest
from scipy.spatial.transform import Rotation
from polinasi_nav.sensor_mounts import mount_rotation,camera_angles,path_heading,bounded_target,LIMITS

ROOT = Path(__file__).resolve().parents[2]

def test_pitch_up_and_camera_yaw_are_body_relative():
    ray = mount_rotation(yaw=np.pi/2,pitch=np.pi/4)@np.array([1.,0.,0.])
    np.testing.assert_allclose(ray,[0.,2**-.5,2**-.5],atol=1e-12)
    body = Rotation.from_euler('z',np.pi/2).as_matrix()
    yaw,pitch = camera_angles([0,0,2],body,[3,0,3],[.2,0,.02])
    direction = body@mount_rotation(yaw,pitch)@np.array([1.,0.,0.])
    expected = np.array([3.,-.2,.98]); expected /= np.linalg.norm(expected)
    np.testing.assert_allclose(direction,expected,atol=1e-12)

def test_heading_follows_motion_and_holds_at_hover():
    assert path_heading([0,1,0],0.) == pytest.approx(np.pi/2)
    assert path_heading([0,0,1],.7) == .7
    assert bounded_target('camera_yaw',99.) == pytest.approx(np.deg2rad(170))
    with pytest.raises(ValueError):
        bounded_target('lidar_pitch',float('nan'))

def test_generated_gimbals_preserve_original_sensor_and_forward_mount(tmp_path):
    spec = importlib.util.spec_from_file_location('prepare_mount_world',ROOT/'tools/prepare_sim_world.py')
    module = importlib.util.module_from_spec(spec); spec.loader.exec_module(module)
    original = (ROOT/'polinasi_nav/models/iris_lidar/model.sdf').read_bytes()
    cfg = json.loads((ROOT/'polinasi_nav/config/tree_inspection.json').read_text())
    cfg['sensor_gimbals_enabled'] = True
    path = tmp_path/'config.json'; path.write_text(json.dumps(cfg))
    module.prepare(ROOT/'polinasi_nav/worlds/palm_farm.sdf',tmp_path/'world.sdf',.15,path)
    model = ET.parse(tmp_path/'models/iris_lidar_mapping/model.sdf').getroot().find('model')
    assert model.get('canonical_link') == 'iris_with_standoffs::base_link'
    assert model.find("joint[@name='lidar_pitch']").get('type') == 'revolute'
    assert float(model.findtext("joint[@name='lidar_pitch']/axis/limit/upper")) == pytest.approx(LIMITS['lidar_pitch'][1])
    assert model.findtext("joint[@name='camera_pitch']/parent") == 'camera_yaw_link'
    assert model.findtext("joint[@name='camera_yaw']/axis/xyz") == '0 0 1'
    assert model.findtext("link[@name='camera_yaw_link']/pose") == '0.20 0 0.02 0 0 0'
    assert model.findtext(".//sensor[@name='mid360']/lidar/scan/vertical/min_angle") == '-0.12217304764'
    assert model.findtext(".//sensor[@name='mid360']/lidar/scan/vertical/max_angle") == '0.90757121104'
    assert (ROOT/'polinasi_nav/models/iris_lidar/model.sdf').read_bytes() == original

def test_68_degree_tilt_is_not_clipped_and_front_fov_centres_above():
    pitch = np.deg2rad(68)
    assert bounded_target('lidar_pitch',pitch) == pytest.approx(pitch)
    assert bounded_target('lidar_pitch',np.deg2rad(90)) == pytest.approx(np.deg2rad(70))
    centre = np.deg2rad(22.5)
    ray = mount_rotation(pitch=pitch)@np.array([np.cos(centre),0.,np.sin(centre)])
    np.testing.assert_allclose(ray,[np.cos(np.deg2rad(90.5)),0.,np.sin(np.deg2rad(90.5))],atol=1e-12)

def test_simulated_self_exclusion_does_not_change_collision_geometry(tmp_path):
    spec = importlib.util.spec_from_file_location('prepare_self_world',ROOT/'tools/prepare_sim_world.py')
    module = importlib.util.module_from_spec(spec); spec.loader.exec_module(module)
    tree = ET.parse(ROOT/'polinasi_nav/models/iris_lidar/model.sdf')
    module.exclude_simulation_self_visuals(tree,tmp_path,ROOT)
    assert tree.findtext(".//sensor[@name='mid360']/lidar/visibility_mask") == '1'
    original = ET.parse(ROOT/'.dependencies/ardupilot_gazebo/models/iris_with_standoffs/model.sdf')
    copied = ET.parse(tmp_path/'iris_with_standoffs_lidar_filtered/model.sdf')
    assert [ET.tostring(x) for x in copied.findall('.//collision')] == [ET.tostring(x) for x in original.findall('.//collision')]
    assert all(v.findtext('visibility_flags')=='2' for v in copied.findall('.//visual'))
