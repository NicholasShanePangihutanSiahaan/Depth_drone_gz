import importlib.util
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
import xml.etree.ElementTree as ET

ROOT=Path(__file__).resolve().parents[2]
spec=importlib.util.spec_from_file_location('drone_visual',ROOT/'tools/generate_drone_visual.py')
module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)


def test_motor_diagonal_span_and_sensor_directions():
    scene=module.geometry()
    motors=np.asarray(scene['motor_centres'])
    for i in (0,1):
        assert abs(np.linalg.norm(motors[i]-motors[i+2])-.5)<1e-12
    camera=scene['groups']['camera']
    r=Rotation.from_euler('xyz',camera['pose'][3:]).as_matrix()
    np.testing.assert_allclose(r@[1,0,0],[0,1,0],atol=1e-12)
    upward=r@Rotation.from_euler('y',-.5).apply([1,0,0])
    assert upward[2]>0 and upward[1]>0
    assert scene['groups']['lidar']['pose'][0]>.13
    assert scene['groups']['lidar']['axis']==[1,0,0]
    assert scene['groups']['lidar']['pose'][2]>.062  # Above the top plate.


def test_exported_gazebo_visuals_match_browser_geometry(tmp_path):
    scene=module.geometry();module.export(scene,tmp_path)
    model=ET.parse(tmp_path/'model.sdf').getroot().find('model')
    assert model.findtext('static')=='true'
    assert len(model.findall('.//visual'))==len(scene['parts'])
    for part in scene['parts']:
        visual=model.find(f"link[@name='{part['group']}']/visual[@name='{part['name']}']")
        assert visual is not None
        assert visual.find('geometry/'+part['kind']) is not None
        np.testing.assert_allclose(list(map(float,visual.findtext('pose').split())),part['pose'])
    assert not model.findall('.//sensor')  # Preview is not a flight sensor model.


def test_self_filter_preserves_close_external_obstacle_in_empty_space():
    from polinasi_nav.self_filter import self_mask
    scene=module.geometry()
    c=dict(self_filter_shapes=[p for p in scene['parts'] if p['group']=='body'],self_filter_shape_padding=.05)
    result=self_mask(np.array([[0,0,0],[0,.26,0],[.1768,.1768,.025],[0,0,.30]]),c)
    assert result.tolist()==[True,False,True,False]
    # Noise perturbs a body-surface return; it must not leave a ghost obstacle.
    assert self_mask(np.array([[.145,0,0]]),c).tolist()==[True]


def test_body_shadow_removes_occluded_ray_even_with_far_endpoint():
    from polinasi_nav.self_filter import body_shadow
    c=dict(self_filter_shapes=[dict(kind='box',size=[.26,.18,.095],pose=[0,0,0,0,0,0])])
    assert body_shadow([.205,0,0],np.array([[-1,0,0],[1,0,0],[0,1,0],[0,0,1]]),c).tolist()==[True,False,False,False]
    assert body_shadow([.205,0,.085],np.array([[-1,0,0],[-1,0,-1],[0,0,1]]),c).tolist()==[False,True,False]


def test_visual50_integrates_sensors_and_preserves_original_assets(tmp_path):
    import json,math
    spec=importlib.util.spec_from_file_location('prepare',ROOT/'tools/prepare_sim_world.py')
    prep=importlib.util.module_from_spec(spec);spec.loader.exec_module(prep)
    source=ROOT/'.dependencies/ardupilot_gazebo/models/iris_with_standoffs/model.sdf'
    if not source.exists():
        import pytest
        pytest.skip('Gazebo dependency absent')
    before=source.read_bytes()
    c=json.loads((ROOT/'polinasi_nav/config/empty_mount_visible.json').read_text())
    c.update(simulation_visual_model='polinasi_visual_50cm',simulation_continuous_roll=True,
             simulation_balance_front_payload=True,simulation_roll_rate=.4,
             lidar_mount=[.205,0,-.035,0.,0.,0.],camera_mount=[0,0,.098],camera_mount_yaw=math.pi/2,
             camera_yaw_enabled=False)
    c['rangefinder_mount']=[.115,0.,-.07,0.,math.pi/2,0.]
    config=tmp_path/'c.json';config.write_text(json.dumps(c))
    prep.prepare(ROOT/'polinasi_nav/worlds/palm_farm.sdf',tmp_path/'world.sdf',.15,config)
    model=ET.parse(tmp_path/'models/iris_lidar_mapping/model.sdf').getroot().find('model')
    assert model.findtext('include/uri')=='model://iris_with_ardupilot_visual50'
    pose=list(map(float,model.findtext("link[@name='zed_camera_link']/pose").split()))
    np.testing.assert_allclose(pose,[0,0,.098,0,0,math.pi/2])
    base=ET.parse(tmp_path/'models/iris_with_standoffs_visual50/model.sdf')
    assert base.getroot().find("model/link[@name='base_link']/visual[@name='body']") is not None
    assert base.getroot().find("model/link[@name='base_link']/collision") is not None
    assert source.read_bytes()==before
