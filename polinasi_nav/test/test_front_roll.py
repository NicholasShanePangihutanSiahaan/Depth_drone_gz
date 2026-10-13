import importlib.util
from pathlib import Path
import math
import numpy as np
from scipy.spatial.transform import Rotation
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[2]


def test_nondefault_mount_changes_simulated_geometry_and_preserves_source(tmp_path):
    spec = importlib.util.spec_from_file_location('prepare', ROOT/'tools/prepare_sim_world.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    import json
    source = ROOT/'polinasi_nav/models/iris_lidar/model.sdf'
    before = source.read_bytes()
    c = json.loads((ROOT/'polinasi_nav/config/empty_mount_visible.json').read_text())
    mount = [.65,0.,.45,math.pi/2,0.,0.]
    c.update(sensor_gimbals_enabled=False,lidar_mount=mount)
    config = tmp_path/'config.json'
    config.write_text(json.dumps(c))
    module.prepare(ROOT/'polinasi_nav/worlds/identification.sdf',tmp_path/'world.sdf',.2,config)
    tree = ET.parse(tmp_path/'models/iris_lidar_mapping/model.sdf')
    pose = tree.getroot().findtext("model/link[@name='mid360_link']/pose")
    np.testing.assert_allclose(list(map(float,pose.split())),mount)
    assert source.read_bytes() == before


def test_fixed_roll90_contains_front_vertical_plane_not_entire_sphere():
    r = Rotation.from_euler('x',math.pi/2).as_matrix()
    for body_ray in ([1,0,0],[1,0,1],[1,0,-1],[0,0,1],[0,0,-1]):
        ray = r.T@body_ray
        elevation = math.atan2(ray[2],np.linalg.norm(ray[:2]))
        assert math.radians(-7) <= elevation <= math.radians(52)
    ray = r.T@np.array([0,1,0])
    assert math.atan2(ray[2],np.linalg.norm(ray[:2])) < math.radians(-7)


def test_continuous_roll_uses_velocity_joint_and_preserves_vertical_fov(tmp_path):
    spec = importlib.util.spec_from_file_location('prepare', ROOT/'tools/prepare_sim_world.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    import json
    c = json.loads((ROOT/'polinasi_nav/config/empty_mount_visible.json').read_text())
    c.update(simulation_continuous_roll=True,simulation_roll_rate=math.pi,
             lidar_mount=[.65,0.,.45,0.,0.,0.])
    config = tmp_path/'config.json'
    config.write_text(json.dumps(c))
    module.prepare(ROOT/'polinasi_nav/worlds/palm_farm.sdf',tmp_path/'world.sdf',.15,config)
    model = ET.parse(tmp_path/'models/iris_lidar_mapping/model.sdf').getroot().find('model')
    assert model.findtext("joint[@name='lidar_roll']/axis/xyz") == '1 0 0'
    assert float(model.findtext("joint[@name='lidar_roll']/axis/limit/velocity")) >= math.pi
    assert model.findtext(".//sensor[@name='mid360']/lidar/scan/vertical/max_angle") == '0.90757121104'
    assert not any(p.findtext('joint_name')=='lidar_pitch' for p in model.findall('plugin'))
    assert model.find("plugin[@name='gz::sim::systems::JointController']/topic").text == '/lidar/roll_rate_cmd'


def test_front_payload_balance_is_run_local_and_cancels_added_xy_moment(tmp_path):
    spec = importlib.util.spec_from_file_location('prepare', ROOT/'tools/prepare_sim_world.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    source = ROOT/'.dependencies/ardupilot_gazebo/models/iris_with_standoffs/model.sdf'
    if not source.exists():
        import pytest
        pytest.skip('ArduPilot Gazebo assets not installed')
    before = source.read_bytes()
    tree = ET.parse(ROOT/'polinasi_nav/models/iris_lidar/model.sdf')
    module.balance_front_payload(tree,tmp_path,ROOT,[.65,0,.45,0,0,0])
    base = ET.parse(tmp_path/'iris_with_standoffs_roll_balanced/model.sdf')
    inertial = base.getroot().find("model/link[@name='base_link']/inertial")
    shift = np.asarray(list(map(float,inertial.findtext('pose').split()))[:2])
    np.testing.assert_allclose(float(inertial.findtext('mass'))*shift+.265*np.array([.65,0]),0.,atol=1e-12)
    assert source.read_bytes() == before
    assert tree.getroot().findtext('model/include/uri') == 'model://iris_with_ardupilot_roll_balanced'
