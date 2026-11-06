"""Synthetic follow-camera geometry and ROS failure handling; no hardware."""
import json
import time
import numpy as np
import pytest

rclpy = pytest.importorskip('rclpy')
from scipy.spatial.transform import Rotation
from rclpy.executors import SingleThreadedExecutor
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header, String
from tf2_msgs.msg import TFMessage
from mid360_slam_test.follow_view import FollowView, follow_transforms
from mid360_slam_test.geometry import pose, transform, pack
from mid360_slam_test.nodes import latched
from mid360_test.nodes import sensor_qos, xyz


@pytest.mark.parametrize('rpy', [[0,0,0],[30,0,0],[0,-35,0],[25,40,120],[-50,60,-170]])
def test_follow_target_does_not_rotate_and_ground_stays_level(rpy):
    body_rotation = Rotation.from_euler('xyz', rpy, degrees=True)
    local = pose([2.,-3.,1.], (body_rotation*Rotation.from_euler('y',20,degrees=True)).as_quat())
    center = np.array([0.,0.,-.16])
    follow, body = follow_transforms(local, center, 20.)
    assert follow[:3,:3] == pytest.approx(np.eye(3))
    assert follow[:3,3] == pytest.approx(transform(local,[center])[0])
    assert body[:3,:3] == pytest.approx(body_rotation.as_matrix())
    # Ground sampled in world; inverse sensor projection mimics a tilted LiDAR.
    ground = np.array([[0.,0.,-1.],[2.,1.,-1.],[-2.,3.,-1.]])
    sensor = (ground-local[:3,3])@local[:3,:3]
    assert transform(local,sensor) == pytest.approx(ground)
    assert np.ptp(transform(local,sensor)[:,2]) < 1e-12


def spin(executor, seconds=.15):
    until=time.monotonic()+seconds
    while time.monotonic()<until:
        executor.spin_once(timeout_sec=.005)


def test_matched_follow_cloud_center_rotation_and_dropout(tmp_path):
    rclpy.init(args=['--ros-args','-p','mode:=replay','-p','use_sim_time:=false',
        '-p','output_dir:='+str(tmp_path),'-p','lidar_mount_pitch_down_deg:=20.0',
        '-p','body_sphere_center_mode:=lidar_coordinates',
        '-p','body_sphere_center_lidar_cm:=[0.0,0.0,-16.0]',
        '-p','body_sphere_filter_enabled:=true','-p','body_sphere_radius_cm:=44.0',
        '-p','estimator_timeout_s:=0.3'])
    executor=SingleThreadedExecutor();node=fixture=None
    clouds=[];states=[];transforms=[]
    try:
        node=FollowView();fixture=rclpy.create_node('follow_view_fixture')
        executor.add_node(node);executor.add_node(fixture)
        source=fixture.create_publisher(PointCloud2,'/slam_test/points',sensor_qos(2))
        status=fixture.create_publisher(String,'/slam_test/estimator_status',10)
        fixture.create_subscription(PointCloud2,'/slam_test/follow_points',clouds.append,sensor_qos(2))
        fixture.create_subscription(String,'/slam_test/follow_status',lambda m:states.append(json.loads(m.data)),latched())
        fixture.create_subscription(TFMessage,'/tf',lambda m:transforms.extend(m.transforms),10)
        spin(executor,.2)
        body_rotation=Rotation.from_euler('xyz',[25.,30.,70.],degrees=True)
        local=pose([2.,-3.,1.],(body_rotation*Rotation.from_euler('y',20.,degrees=True)).as_quat())
        ground=np.array([[1.,1.,-1.],[2.,-2.,-1.],[-1.,-1.,-1.]])
        sensor=(ground-local[:3,3])@local[:3,:3]
        header=Header(frame_id='livox_frame');header.stamp.sec=10
        cloud=point_cloud2.create_cloud_xyz32(header,sensor.tolist())
        original=bytes(cloud.data)
        state=dict(valid=True,stamp=10.,local=pack(local))
        # Mismatched pose must never be applied to this scan.
        status.publish(String(data=json.dumps(dict(state,stamp=11.))))
        source.publish(cloud);spin(executor)
        assert not clouds
        status.publish(String(data=json.dumps(state)));spin(executor)
        assert bytes(cloud.data)==original
        assert clouds[-1].header.frame_id=='slam_odom'
        assert xyz(clouds[-1])[0]==pytest.approx(ground,abs=1e-6)
        target=[t for t in transforms if t.child_frame_id=='slam_test_follow'][-1]
        assert [target.transform.rotation.x,target.transform.rotation.y,target.transform.rotation.z,target.transform.rotation.w]==[0.,0.,0.,1.]
        expected=transform(local,[[0.,0.,-.16]])[0]
        assert [target.transform.translation.x,target.transform.translation.y,target.transform.translation.z]==pytest.approx(expected)
        assert states[-1]['valid']
        # Invalid tracking clears the old live cloud immediately, never fake HOLD.
        status.publish(String(data=json.dumps(dict(valid=False,reason='tracking_lost'))))
        spin(executor)
        assert not states[-1]['valid'] and clouds[-1].width==0
        # New matched data can restore visualization; no synthetic tracking repair.
        header.stamp.sec=12
        source.publish(point_cloud2.create_cloud_xyz32(header,sensor.tolist()))
        status.publish(String(data=json.dumps(dict(state,stamp=12.))))
        spin(executor)
        assert states[-1]['valid']
        spin(executor,.5)
        assert not states[-1]['valid']
        assert states[-1]['reason']=='matched_cloud_or_pose_stale'
        assert clouds[-1].width==0
        assert not json.loads((tmp_path/'follow_view_status.json').read_text())['valid']
        # Malformed input is rejected without crashing.
        status.publish(String(data='[]'));spin(executor)
        assert states[-1]['reason']=='malformed_estimator_status'
    finally:
        for item in (fixture,node):
            if item is not None:
                executor.remove_node(item);item.destroy_node()
        executor.shutdown();rclpy.shutdown()


def test_actual_registration_tracks_tilt_and_preserves_horizontal_ground():
    from mid360_slam_test.estimator import SlamEstimator
    from mid360_slam_test.geometry import inverse, unpack
    backend = SlamEstimator(dict(voxel_m=.03, min_points=50,
        correspondence_m=.4, max_rmse_m=.04, loop_enabled=False))
    world = np.random.default_rng(73).uniform([-3.,-3.,-1.],[3.,3.,2.],(1000,3))
    ground = np.array([[0.,0.,-1.],[2.,1.,-1.],[-1.,-2.,-1.]])
    center = np.array([0.,0.,-.16])
    for i in range(9):
        rbody = Rotation.from_euler('xyz',[i*3.,i*2.,i*4.],degrees=True)
        truth = pose([i*.015,0.,0.],(rbody*Rotation.from_euler('y',20.,degrees=True)).as_quat())
        scan = transform(inverse(truth),world)
        result = backend.scan(scan,1.+i*.2,truth[:3,:3])
        assert result['valid'],result
        estimated = unpack(result['local'])
        assert estimated == pytest.approx(truth,abs=1e-5)
        floor_scan = transform(inverse(truth),ground)
        assert transform(estimated,floor_scan)[:,2] == pytest.approx([-1.,-1.,-1.],abs=1e-5)
        follow,body = follow_transforms(estimated,center,20.)
        assert follow[:3,:3] == pytest.approx(np.eye(3))
        assert body[:3,:3] == pytest.approx(rbody.as_matrix(),abs=1e-5)
