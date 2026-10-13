import math
import numpy as np
from scipy.spatial.transform import Rotation
from polinasi_nav.localisation import LidarImuOdometry


def test_stationary_body_remains_stationary_when_lidar_rolls():
    e=LidarImuOdometry([.205,0,.085,0,0,0],body_imu=True)
    for i in range(50):e.imu([0,0,0],[0,0,9.81],i*.005)
    x,y=np.meshgrid(np.linspace(-5,5,25),np.linspace(-5,5,25))
    x,y=x.ravel(),y.ravel()
    world=np.vstack((np.column_stack((x,y,np.full_like(x,-1.))),
                     np.column_stack((np.full_like(x,5.),x,y+3.)),
                     np.column_stack((x,np.full_like(x,5.),y+3.))))
    assert e.scan((world-e.mount_p)@e.mount_r,.245)
    for k in range(1,10):
        stamp=.245+k*.1
        for j in range(20):e.imu([0,0,0],[0,0,9.81],stamp-.095+j*.005)
        e.mount_r=Rotation.from_euler('x',math.pi*stamp).as_matrix()
        assert e.scan((world-e.mount_p)@e.mount_r,stamp)
        np.testing.assert_allclose(e.p,0.,atol=1e-9)
        np.testing.assert_allclose(e.r,np.eye(3),atol=1e-9)


def test_body_imu_gravity_is_not_rotated_with_lidar_mount():
    e=LidarImuOdometry([.205,0,.085,math.pi/2,0,0],body_imu=True)
    for i in range(50):e.imu([0,0,0],[0,0,9.81],i*.005)
    assert e.initialized
    np.testing.assert_allclose(e.r,np.eye(3),atol=1e-9)


def test_stationary_calibration_does_not_integrate_gravity_magnitude_mismatch():
    e=LidarImuOdometry([0,0,0,0,0,0],body_imu=True)
    for i in range(200):e.imu([.001,0,0],[0,0,9.8],i*.005)
    np.testing.assert_allclose(e.p,0.,atol=1e-9)
    np.testing.assert_allclose(e.v,0.,atol=1e-9)
    np.testing.assert_allclose(e.r,np.eye(3),atol=1e-9)


def test_body_imu_fixture_is_physical_sensor_not_odometry(tmp_path):
    import importlib.util,json,xml.etree.ElementTree as ET
    from pathlib import Path
    root=Path(__file__).resolve().parents[2]
    c=json.loads((root/'polinasi_nav/config/empty_mount_visible.json').read_text())
    c.update(sensor_rolling_localisation=True,simulation_continuous_roll=True,simulation_roll_rate=math.pi,
             camera_yaw_enabled=False)
    config=tmp_path/'config.json';config.write_text(json.dumps(c))
    spec=importlib.util.spec_from_file_location('prepare',root/'tools/prepare_sim_world.py')
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    module.prepare(root/'polinasi_nav/worlds/palm_farm.sdf',tmp_path/'world.sdf',.15,config)
    model=ET.parse(tmp_path/'models/iris_lidar_mapping/model.sdf').getroot().find('model')
    sensor=model.find("link[@name='body_imu_link']/sensor")
    assert sensor.get('type')=='imu' and sensor.findtext('gz_frame_id')=='base_link'
    assert model.findtext("joint[@name='body_imu_fixed']/parent").endswith('base_link')


def test_full_roll_bootstrap_requires_stationary_disarmed_prior():
    e=LidarImuOdometry([0,0,0,0,0,0],body_imu=True,bootstrap_duration=.2)
    for i in range(50):e.imu([0,0,0],[0,0,9.81],i*.005)
    points=np.random.default_rng(2).uniform(-3,3,(500,3))
    assert not e.scan(points,.245)
    assert e.reason=='bootstrap_requires_disarmed_stationary_launch'
    e.bootstrap_stationary=True
    for k in range(3):
        t=.345+.1*k
        for j in range(20):e.imu([0,0,0],[0,0,9.81],t-.095+j*.005)
        result=e.scan(points,t)
    assert result and e.reason=='full_roll_bootstrap'
