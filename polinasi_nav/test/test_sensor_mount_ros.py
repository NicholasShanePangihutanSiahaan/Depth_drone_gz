"""ROS command/feedback ownership regressions; no flight commands."""
from types import SimpleNamespace
from unittest.mock import Mock
import pytest
pytest.importorskip('rclpy')
pytest.importorskip('mavros_msgs')
from sensor_msgs.msg import JointState
from builtin_interfaces.msg import Time
from polinasi_nav.sensor_mount_node import SensorMountNode


def test_fixed_lidar_mount_ready_requires_only_camera_joint_feedback():
    import json
    fake=SimpleNamespace(now=lambda:1.1,last_tick=1.,navigation={},armed=False,
        state_stamp=1.,c={'lidar_fixed':True},rolling=False,
        targets={'camera_pitch':.5},commands={'camera_pitch':.4},
        measured={'camera_pitch':.4},stamps={'camera_pitch':1.},
        outputs={'camera_pitch':Mock()},status=Mock(),rejection='')
    SensorMountNode.tick(fake)
    status=json.loads(fake.status.publish.call_args.args[0].data)
    assert status['navigation_ready'] and status['feedback_fresh']
    assert 'lidar_pitch' not in fake.targets
    fake.stamps['camera_pitch']=0.
    SensorMountNode.tick(fake)
    assert not json.loads(fake.status.publish.call_args.args[0].data)['navigation_ready']


def test_roll_rate_log_uses_timestamped_measured_angle_not_command():
    import math
    fake=SimpleNamespace(now=lambda:2.05,targets={'lidar_roll':0.},
        measured={'lidar_roll':math.pi},stamps={'lidar_roll':1.},
        tf=Mock(),c={'lidar_mount':[.205,0.,.085,0.,0.,0.]})
    fake.transform=lambda *args:SensorMountNode.transform(fake,*args)
    msg=JointState(header=__import__('std_msgs.msg',fromlist=['Header']).Header(stamp=Time(sec=2)),
                   name=['lidar_roll'],position=[2*math.pi])
    SensorMountNode.feedback(fake,msg)
    assert fake.roll_rate_measured==pytest.approx(math.pi)

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


def test_roll_tf_uses_measured_angle_and_front_mount():
    import math
    fake = SimpleNamespace(now=lambda:3.05,targets={'lidar_roll':0.},measured={},stamps={},tf=Mock(),
        c={'lidar_mount':[.65,0.,.45,0.,0.,0.]})
    fake.transform = lambda *args:SensorMountNode.transform(fake,*args)
    msg = JointState(header=__import__('std_msgs.msg',fromlist=['Header']).Header(stamp=Time(sec=3)),
                    name=['iris_lidar::lidar_roll'],position=[math.pi/2])
    SensorMountNode.feedback(fake,msg)
    t = fake.tf.sendTransform.call_args.args[0][0]
    assert t.child_frame_id == 'mid360' and t.header.stamp == msg.header.stamp
    assert t.transform.translation.x == .65
    assert abs(t.transform.rotation.x-math.sqrt(.5)) < 1e-8


def test_left_camera_tf_pitch_looks_up_in_left_direction():
    import math
    import numpy as np
    from scipy.spatial.transform import Rotation
    fake=SimpleNamespace(now=lambda:3.05,targets={'camera_pitch':.5},measured={},stamps={},tf=Mock(),
        c={'camera_yaw_enabled':False,'camera_mount':[0,0,.098],'camera_mount_yaw':math.pi/2})
    fake.transform=lambda *args:SensorMountNode.transform(fake,*args)
    msg=JointState(header=__import__('std_msgs.msg',fromlist=['Header']).Header(stamp=Time(sec=3)),
                   name=['camera_pitch'],position=[.4])
    SensorMountNode.feedback(fake,msg)
    ts=fake.tf.sendTransform.call_args.args[0]
    def rot(t):
        q=t.transform.rotation
        return Rotation.from_quat([q.x,q.y,q.z,q.w]).as_matrix()
    ray=rot(ts[0])@rot(ts[1])@np.array([1.,0.,0.])
    assert abs(ray[0])<1e-8 and ray[1]>0 and ray[2]>0


def test_relocated_rangefinder_corrects_forward_and_downward_mount_offset():
    import math
    import numpy as np
    from scipy.spatial.transform import Rotation
    from geometry_msgs.msg import TransformStamped
    from sensor_msgs.msg import Range
    from polinasi_nav.ros_nodes import NavigationNode
    tf=TransformStamped()
    r=Rotation.from_euler('y',.2)
    q=r.as_quat()
    tf.transform.rotation.x,tf.transform.rotation.y,tf.transform.rotation.z,tf.transform.rotation.w=q
    from collections import deque
    fake=SimpleNamespace(c={'rangefinder_mount':[.115,0.,-.07,0.,math.pi/2,0.],'range_timeout':.3},
                         buffer=Mock(),range_value=None,range_stamp=None,
                         pending_ranges=deque(maxlen=12),range_tf_waits=0,now=lambda:.1)
    fake.range_pending=lambda:NavigationNode.range_pending(fake)
    fake.buffer.lookup_transform.return_value=tf
    msg=Range(min_range=.05,max_range=8.,range=2.)
    NavigationNode.range_cb(fake,msg)
    expected=2.*r.as_matrix()[2,2]-(r.as_matrix()@np.array([.115,0.,-.07]))[2]
    assert abs(fake.range_value-expected)<1e-9


def test_rangefinder_static_tf_keeps_map_odom_in_same_latched_message():
    from polinasi_nav.ros_nodes import LocalisationNode
    fake=SimpleNamespace(c={'lidar_mount':[.205,0,-.035,0,0,0],
        'sensor_gimbals_enabled':True,'rangefinder_mount':[.115,0,-.07,0,1.57079632679,0]},
        static=Mock(),get_clock=Mock())
    fake.get_clock.return_value.now.return_value.to_msg.return_value=Time(sec=1)
    LocalisationNode.send_static(fake)
    fake.static.sendTransform.assert_called_once()
    transforms=fake.static.sendTransform.call_args.args[0]
    assert {t.child_frame_id for t in transforms}=={'odom','mid360_imu','rangefinder'}
