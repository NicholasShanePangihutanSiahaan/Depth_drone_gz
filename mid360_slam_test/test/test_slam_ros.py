"""Synthetic ROS integration in a private domain; no physical sensors."""
import json
import time
import uuid
import numpy as np
import pytest

rclpy=pytest.importorskip('rclpy')
from rclpy.executors import SingleThreadedExecutor
from sensor_msgs.msg import Imu,PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import String
from mid360_test.nodes import CloudAdapter,sensor_qos
from mid360_slam_test.nodes import (CloudPreprocessor,EstimatorNode,EvaluatorNode,MonitorNode,SlamRecorder)
from mid360_slam_test.geometry import inverse,pose,transform
from mid360_slam_test.follow_view import FollowView


def spin(executor,seconds=.03):
    until=time.monotonic()+seconds
    while time.monotonic()<until:
        executor.spin_once(timeout_sec=.005)


def test_full_pipeline_sensor_to_estimator_stage_results_bag_and_map(tmp_path):
    rclpy.init(args=['--ros-args','-p','mode:=replay','-p','use_sim_time:=false',
        '-p','cloud_type:=sensor_msgs/msg/PointCloud2','-p','output_dir:='+str(tmp_path),
        '-p','calibration_seconds:=0.1','-p','calibration_min_samples:=5',
        '-p','imu_receipt_timeout_s:=1.0','-p','max_scan_gap_s:=1.0',
        '-p','settle_window_s:=0.2','-p','minimum_settle_samples:=3',
        '-p','max_stationary_drift_m:=0.02','-p','max_stationary_orientation_deg:=1.0',
        '-p','map_publish_period_s:=0.2','-p','loop_min_separation:=4',
        '-p','loop_min_travel_m:=0.3','-p','loop_min_seconds:=0.3',
        '-p','loop_cooldown_s:=0.2','-p','keyframe_translation_m:=0.04',
        '-p','loop_radius_m:=0.15','-p','voxel_m:=0.04'])
    executor=SingleThreadedExecutor();nodes=[];fixture=None
    try:
        for klass in (CloudAdapter,CloudPreprocessor,EstimatorNode,EvaluatorNode,MonitorNode,SlamRecorder,FollowView):
            node=klass();nodes.append(node);executor.add_node(node)
        fixture=rclpy.create_node('synthetic_slam_fixture');executor.add_node(fixture)
        imus=fixture.create_publisher(Imu,'/livox/imu',sensor_qos(100))
        clouds=fixture.create_publisher(PointCloud2,'/livox/lidar',sensor_qos(10))
        commands=fixture.create_publisher(String,'/slam_test/commands',10)
        spin(executor,.4)
        world=np.random.default_rng(22).uniform([-3,-3,-1],[3,3,2],(700,3))
        count=0
        def publish(x=0.):
            nonlocal count
            # One cloud at 10Hz source time; many IMU samples between scans.
            for _ in range(5):
                count+=1
                stamp=100.+count*.02
                imu=Imu();imu.header.frame_id='livox_frame'
                imu.header.stamp.sec=int(stamp);imu.header.stamp.nanosec=round((stamp-int(stamp))*1e9)
                imu.linear_acceleration.z=1.
                imus.publish(imu);spin(executor,.008)
            cloud=point_cloud2.create_cloud_xyz32(imu.header,transform(inverse(pose([x,0,0])),world).tolist())
            clouds.publish(cloud);spin(executor,.025)
        for _ in range(18):publish()
        estimator,evaluator=nodes[2],nodes[3]
        assert estimator.last_status and estimator.last_status['valid'],estimator.last_status
        assert evaluator.engine.view(time.monotonic())['can_start']
        def command(action,**kwargs):
            identifier=uuid.uuid4().hex
            commands.publish(String(data=json.dumps(dict(id=identifier,action=action,**kwargs))))
            spin(executor,.06)
            assert evaluator.engine.seen_commands[identifier]['ok']
        command('reference',duration_s=.3)
        command('start',reference_confirmed=True)
        for _ in range(10):publish()
        assert evaluator.engine.phase=='REVIEW'
        assert evaluator.engine.results[-1]['status']=='PASS'
        # Genuine registration/graph optimisation loop, separate from stage results.
        for x in [0,.05,.1,.15,.2,.25,.25,.2,.15,.1,.05,0]:publish(x)
        assert estimator.last_status['valid'],estimator.last_status
        assert estimator.last_status['loops_accepted']>0
        spin(executor,.1)
        for node in reversed(nodes):
            executor.remove_node(node);node.destroy_node()
        nodes=[]
        result=json.loads((tmp_path/'test_results.json').read_text())
        assert result['results'][0]['stage']=='stationary'
        assert (tmp_path/'keyframe_poses.npz').exists() and (tmp_path/'map_points.npy').exists()
        assert 'ACCEPTED' in (tmp_path/'loop_events.jsonl').read_text()
        rec=json.loads((tmp_path/'recorder_status.json').read_text())
        assert rec['closed'] and rec['dropped_by_recorder']==0
        assert rec['written_by_topic']['/slam_test/odometry_local']>0
        assert rec['written_by_topic']['/slam_test/loop_events']>0
        assert rec['written_by_topic']['/slam_test/follow_points']>0
        assert rec['written_by_topic']['/slam_test/follow_status']>0
        import rosbag2_py
        reader=rosbag2_py.SequentialReader()
        reader.open(rosbag2_py.StorageOptions(uri=str(tmp_path/'raw_bag'),storage_id='sqlite3'),
                    rosbag2_py.ConverterOptions('',''))
        kinds={t.name:t.type for t in reader.get_all_topics_and_types()}
        assert kinds['/livox/lidar']=='sensor_msgs/msg/PointCloud2'
        assert kinds['/slam_test/loop_events']=='std_msgs/msg/String'
        assert reader.has_next()
    finally:
        for node in reversed(nodes):
            executor.remove_node(node);node.destroy_node()
        if fixture:fixture.destroy_node()
        executor.shutdown();rclpy.shutdown()


def test_preprocessor_box_preserves_near_real_object():
    # Explicit measured box; never an arbitrary all-near-points cutoff.
    from types import SimpleNamespace
    from std_msgs.msg import Header
    published=[]
    from mid360_slam_test.self_filter import BodySphere
    fake=SimpleNamespace(guard_reason='',c=dict(expected_frame='livox_frame',minimum_range_m=.1,
        maximum_range_m=20.,reject_tag_mask=0,preprocess_voxel_m=.01,preprocess_max_points=100),
        boxes=[dict(min=[-.2,-.2,-.1],max=[.2,.2,.1])],
        sphere=BodySphere(),removed_pub=SimpleNamespace(publish=lambda _:None),
        sphere_pub=SimpleNamespace(publish=lambda _:None),
        pub=SimpleNamespace(publish=published.append),stats=SimpleNamespace(publish=lambda _:None))
    msg=point_cloud2.create_cloud_xyz32(Header(frame_id='livox_frame'),[[.15,0,0],[.3,0,0],[2,0,0]])
    CloudPreprocessor.cloud(fake,msg)
    from mid360_test.nodes import xyz
    assert xyz(published[0])[0][:,0]==pytest.approx([.3,2.])


def test_preprocessor_rotated_sphere_removes_body_keeps_near_external_and_raw():
    from types import SimpleNamespace
    from std_msgs.msg import Header
    from mid360_test.nodes import xyz
    from mid360_slam_test.self_filter import BodySphere
    sphere=BodySphere(pitch_down_deg=30.,a_cm=20.,b_cm=10.,radius_cm=15.,enabled=True)
    outputs=[];removed=[];stats=[];markers=[]
    fake=SimpleNamespace(guard_reason='',c=dict(expected_frame='livox_frame',minimum_range_m=.01,
        maximum_range_m=20.,reject_tag_mask=0,preprocess_voxel_m=.001,preprocess_max_points=100),
        boxes=[],sphere=sphere,pub=SimpleNamespace(publish=outputs.append),
        removed_pub=SimpleNamespace(publish=removed.append),
        sphere_pub=SimpleNamespace(publish=markers.append),stats=SimpleNamespace(publish=stats.append))
    body=sphere.center
    outside=sphere.center+np.array([.151,0,0])
    msg=point_cloud2.create_cloud_xyz32(Header(frame_id='livox_frame'),[body.tolist(),outside.tolist(),[1.,0.,0.]])
    original_bytes=bytes(msg.data)
    CloudPreprocessor.cloud(fake,msg)
    assert bytes(msg.data)==original_bytes
    points,_=xyz(outputs[0])
    assert len(points)==2 and any(np.allclose(p,outside) for p in points)
    assert len(xyz(removed[0])[0])==1
    detail=json.loads(stats[0].data)
    assert detail['sphere_removed_points']==1
    assert detail['sphere_filter']['center_lidar_m']==pytest.approx(body)
    assert markers[0].markers[0].scale.x==pytest.approx(.3)


def test_drone_view_tf_levels_ground_without_changing_sensor_coordinates(tmp_path):
    from tf2_msgs.msg import TFMessage
    from scipy.spatial.transform import Rotation
    from mid360_slam_test.nodes import latched
    rclpy.init(args=['--ros-args', '-p', 'mode:=replay', '-p', 'use_sim_time:=false',
        '-p', 'output_dir:='+str(tmp_path), '-p', 'publish_drone_view_tf:=true',
        '-p', 'lidar_mount_pitch_down_deg:=20.0'])
    executor = SingleThreadedExecutor()
    node = fixture = None
    received = []
    try:
        node = CloudPreprocessor()
        fixture = rclpy.create_node('drone_view_tf_fixture')
        fixture.create_subscription(TFMessage, '/tf_static', received.append, latched())
        executor.add_node(node); executor.add_node(fixture)
        spin(executor, .5)
        transforms = [t for m in received for t in m.transforms
                      if t.header.frame_id == 'slam_test_drone_view' and
                      t.child_frame_id == 'livox_frame']
        assert transforms
        tf = transforms[-1].transform
        assert [tf.translation.x, tf.translation.y, tf.translation.z] == [0., 0., 0.]
        rotation = Rotation.from_quat([tf.rotation.x, tf.rotation.y,
                                     tf.rotation.z, tf.rotation.w])
        # A physical horizontal floor, observed by the downward-pitched sensor.
        ground_body = np.array([[1., 0., -1.], [2., 1., -1.], [-1., -1., -1.]])
        ground_lidar = Rotation.from_euler('y', -20., degrees=True).apply(ground_body)
        assert np.ptp(ground_lidar[:, 2]) > .5
        assert rotation.apply(ground_lidar) == pytest.approx(ground_body)
        # Native sphere definition must undergo exactly the same TF in RViz.
        assert rotation.apply([0., 0., -.16]) == pytest.approx([-.05472322, 0., -.15035082])
    finally:
        for item in (fixture, node):
            if item is not None:
                executor.remove_node(item); item.destroy_node()
        executor.shutdown(); rclpy.shutdown()
