"""ROS integration with synthetic replay only. Never connects to real equipment."""
import json
import os
from pathlib import Path
import time
from types import SimpleNamespace

import numpy as np
import pytest

rclpy = pytest.importorskip('rclpy')
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter
from sensor_msgs.msg import PointCloud2, Imu
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from rclpy.serialization import serialize_message, deserialize_message
from mid360_test.nodes import (CloudAdapter, DirectionalDistance, ImuIntegration,
    HealthMonitor, TestLogger as LoggerNode, RawRecorder, BenchNode, sensor_qos, xyz)


def spin(executor, seconds=.15):
    until = time.monotonic()+seconds
    while time.monotonic() < until:
        executor.spin_once(timeout_sec=.005)


def test_end_to_end_replay_records_original_and_logs_sectors(tmp_path):
    # Isolate from simulation/hardware sessions; caller also sets ROS_LOCALHOST_ONLY.
    rclpy.init(args=['--ros-args', '-p', 'mode:=replay', '-p', 'use_sim_time:=false',
        '-p', 'output_dir:='+str(tmp_path), '-p', 'calibration_seconds:=0.1',
        '-p', 'calibration_min_samples:=5', '-p', 'imu_timeout_s:=1.0',
        '-p', 'cloud_timeout_s:=1.0'])
    nodes = []
    executor = SingleThreadedExecutor()
    try:
        nodes = [CloudAdapter(), DirectionalDistance(), ImuIntegration(), HealthMonitor(), LoggerNode(), RawRecorder()]
        for node in nodes:
            executor.add_node(node)
        fixture = rclpy.create_node('synthetic_bench_fixture')
        executor.add_node(fixture)
        clouds = fixture.create_publisher(PointCloud2, '/livox/lidar', sensor_qos())
        imus = fixture.create_publisher(Imu, '/livox/imu', sensor_qos())
        spin(executor, .4)
        original_clouds = []
        for i in range(45):
            stamp = 100.+i*.01
            msg = Imu()
            msg.header.frame_id = 'livox_frame'
            msg.header.stamp.sec = int(stamp)
            msg.header.stamp.nanosec = round((stamp-int(stamp))*1e9)
            msg.linear_acceleration.z = 1.  # upstream Livox g units
            imus.publish(msg)
            cloud = point_cloud2.create_cloud_xyz32(msg.header,
                [[1., 0., 0.], [0., -2., 0.], [0., 3., 0.], [-4., 0., 0.]])
            original_clouds.append(cloud)
            clouds.publish(cloud)
            spin(executor, .015)
        spin(executor, .1)
        logger = nodes[4]
        logger.tick()
        assert logger.cloud['sectors']['front']['distance_cm'] == 100.
        assert logger.latest_estimate['valid']
        assert 'right = 200.0 cm' in (tmp_path/'readings.log').read_text()
        # Health cannot turn an absent source into PASS.
        assert nodes[3].streams['cloud'].count > 0
        executor.remove_node(nodes[-1])
        nodes.pop().destroy_node()
        status = json.loads((tmp_path/'recorder_status.json').read_text())
        assert status['closed'] and status['written_messages'] > 0
        assert status['dropped_by_recorder'] == 0
        import rosbag2_py
        reader = rosbag2_py.SequentialReader()
        reader.open(rosbag2_py.StorageOptions(uri=str(tmp_path/'raw_bag'), storage_id='sqlite3'),
                    rosbag2_py.ConverterOptions('', ''))
        read_clouds = []
        while reader.has_next():
            topic, data, _ = reader.read_next()
            if topic == '/livox/lidar':
                read_clouds.append(data)
        # Middleware CDR alignment padding is not a ROS field and may differ
        # from serialize_message(). Compare every semantic message field.
        assert read_clouds
        for data in read_clouds:
            decoded = deserialize_message(data, PointCloud2)
            assert any(decoded == original for original in original_clouds)
        replay_cloud = deserialize_message(read_clouds[-1], PointCloud2)
        assert xyz(replay_cloud)[0].shape == (4, 3)
        fixture.destroy_node()
    finally:
        for node in reversed(nodes):
            executor.remove_node(node)
            node.destroy_node()
        executor.shutdown()
        rclpy.shutdown()


def test_hardware_refuses_sim_time(tmp_path):
    rclpy.init(args=['--ros-args', '-p', 'mode:=hardware', '-p', 'use_sim_time:=true',
                     '-p', 'output_dir:='+str(tmp_path)])
    try:
        with pytest.raises(ValueError, match='use_sim_time'):
            CloudAdapter()
    finally:
        rclpy.shutdown()


def test_custom_adapter_preserves_tags_offsets_and_invalid_points():
    # No Livox hardware/messages required for this conversion regression.
    published = []
    adapter = SimpleNamespace(c=dict(cloud_type='livox_ros_driver2/msg/CustomMsg',
                                    cloud_topic='/livox/lidar'),
        accepts=lambda topic: True, pub=SimpleNamespace(publish=published.append))
    header = Header(frame_id='livox_frame')
    p = SimpleNamespace(x=1., y=0., z=0., reflectivity=20, tag=16, line=2, offset_time=123)
    invalid = SimpleNamespace(x=0., y=0., z=0., reflectivity=0, tag=0, line=0, offset_time=456)
    CloudAdapter.receive(adapter, SimpleNamespace(header=header, point_num=2, points=[p, invalid]))
    points, tags = xyz(published[0])
    assert points.tolist() == [[1., 0., 0.], [0., 0., 0.]]
    assert tags.tolist() == [16, 0]
    offsets = point_cloud2.read_points(published[0], field_names=('offset_time',))
    assert list(offsets['offset_time']) == [123, 456]


@pytest.mark.parametrize('names,clock,expected', [
    (['livox_lidar_publisher'], False, ''),
    (['simulation_bridge'], False, 'unapproved_source_publisher'),
    (['livox_lidar_publisher', 'livox_lidar_publisher'], False, 'multiple_source_publishers'),
    (['livox_lidar_publisher'], True, 'clock_topic_present'),
])
def test_hardware_graph_guard(names, clock, expected):
    c = dict(mode='hardware', cloud_topic='/livox/lidar', imu_topic='/livox/imu',
             cloud_type='sensor_msgs/msg/PointCloud2', allowed_publishers=['livox_lidar_publisher'])
    fake = SimpleNamespace(c=c, guard_reason='',
        get_parameter=lambda _: SimpleNamespace(value=False),
        get_topic_names_and_types=lambda: [('/clock', ['rosgraph_msgs/msg/Clock'])] if clock else [],
        get_publishers_info_by_topic=lambda topic: [SimpleNamespace(node_name=name,
            node_namespace='/', topic_type='sensor_msgs/msg/PointCloud2' if topic.endswith('lidar')
            else 'sensor_msgs/msg/Imu') for name in names],
        get_logger=lambda: SimpleNamespace(error=lambda _: None))
    BenchNode.guard_update(fake)
    assert (expected in fake.guard_reason) if expected else not fake.guard_reason


def test_recorder_overflow_is_counted_instead_of_blocking():
    import queue
    import threading
    recorder = SimpleNamespace(c=dict(recorder_max_queue_bytes=4), lock=threading.Lock(),
        accepts=lambda _: True, received=0, received_by_topic={'/livox/lidar': 0},
        queued_bytes=0, dropped=0, error='', q=queue.Queue(maxsize=1))
    RawRecorder.receive(recorder, '/livox/lidar', b'too-large')
    assert recorder.dropped == 1 and recorder.q.empty()


def test_logger_does_not_attach_old_valid_estimate_after_imu_loss():
    from collections import deque
    logger = SimpleNamespace(latest_estimate=dict(valid=False, state='INVALID'),
        imu_received=10., c=dict(imu_timeout_s=.1), estimates=deque())
    result = LoggerNode.matched_estimate(logger, 10.01)
    assert not result['valid'] and result['state'] == 'INVALID'
