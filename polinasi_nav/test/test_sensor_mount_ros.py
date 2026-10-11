"""ROS command/feedback ownership regressions; no flight commands."""
from types import SimpleNamespace
from unittest.mock import Mock
import pytest
pytest.importorskip('rclpy')
pytest.importorskip('mavros_msgs')
from sensor_msgs.msg import JointState
from builtin_interfaces.msg import Time
from polinasi_nav.sensor_mount_node import SensorMountNode

def test_airborne_lidar_tilt_rejected_without_touching_camera():
    fake = SimpleNamespace(armed=True,state_stamp=1.,now=lambda:1.1,
                           c={},targets={'lidar_pitch':0.,'camera_yaw':0.},rejection='')
    SensorMountNode.target(fake,'lidar_pitch',.4)
    assert fake.targets['lidar_pitch']==0.
    assert 'disarmed' in fake.rejection
    SensorMountNode.target(fake,'camera_yaw',1.2)
    assert fake.targets['camera_yaw']==1.2
    fake.armed = False
    SensorMountNode.target(fake,'lidar_pitch',.4)
    assert fake.targets['lidar_pitch']==.4
    fake.state_stamp = -2.
    SensorMountNode.target(fake,'lidar_pitch',.7)
    assert fake.targets['lidar_pitch']==.4 and fake.rejection


def test_airborne_tilt_only_in_fresh_authorized_stationary_phase():
    fake = SimpleNamespace(armed=True,state_stamp=1.,now=lambda:1.1,c={'automatic_tilt_ascent':True},
        targets={'lidar_pitch':0.},rejection='',navigation=dict(simulation_time=1.,flight='MISSION',
        lidar_tilt_fsm_authorized=True,lidar_tilt_change_authorized=True,failure=''))
    SensorMountNode.target(fake,'lidar_pitch',.7)
    assert fake.targets['lidar_pitch']==.7
    fake.navigation['lidar_tilt_change_authorized'] = False
    SensorMountNode.target(fake,'lidar_pitch',.25)
    assert fake.targets['lidar_pitch']==.7
    SensorMountNode.target(fake,'lidar_pitch',.7)  # Hold fixed angle during certified ascent.
    assert not fake.rejection
    fake.navigation['simulation_time'] = 0.
    SensorMountNode.target(fake,'lidar_pitch',.25)
    assert fake.rejection and fake.targets['lidar_pitch']==.7

def test_tf_uses_scoped_measured_joints_not_servo_targets():
    fake = SimpleNamespace(now=lambda:3.05,targets=dict(lidar_pitch=.8,camera_yaw=2.,camera_pitch=1.),
        measured={},stamps={},tf=Mock(),c={'lidar_mount':[0,0,.2,0,0,0],'camera_mount':[.2,0,.02]})
    fake.transform = lambda *args:SensorMountNode.transform(fake,*args)
    msg = JointState(header=__import__('std_msgs.msg',fromlist=['Header']).Header(stamp=Time(sec=3)),
        name=['iris_lidar::lidar_pitch','iris_lidar::camera_yaw','iris_lidar::camera_pitch'],position=[.2,.3,.4])
    SensorMountNode.feedback(fake,msg)
    transforms = fake.tf.sendTransform.call_args.args[0]
    assert {t.child_frame_id for t in transforms} == {'mid360','camera_yaw_link','zed_camera_link'}
    assert all(t.header.stamp==msg.header.stamp for t in transforms)
    assert fake.measured==dict(lidar_pitch=.2,camera_yaw=.3,camera_pitch=.4)
    fake.tf.reset_mock()
    fake.now = lambda:4.
    SensorMountNode.feedback(fake,msg)
    fake.tf.sendTransform.assert_not_called()


def test_pitch_only_feedback_produces_tf_without_nonexistent_yaw_measurement():
    fake = SimpleNamespace(now=lambda:3.05,targets={'camera_pitch':.5},measured={},stamps={},tf=Mock(),
        c={'camera_yaw_enabled':False,'camera_mount':[.2,0,.02]})
    fake.transform = lambda *args:SensorMountNode.transform(fake,*args)
    msg = JointState(header=__import__('std_msgs.msg',fromlist=['Header']).Header(stamp=Time(sec=3)),
        name=['iris_lidar::camera_pitch'],position=[.4])
    SensorMountNode.feedback(fake,msg)
    transforms = fake.tf.sendTransform.call_args.args[0]
    assert {t.child_frame_id for t in transforms}=={'camera_yaw_link','zed_camera_link'}
    assert fake.measured=={'camera_pitch':.4}
    assert transforms[0].transform.rotation.w==1.
