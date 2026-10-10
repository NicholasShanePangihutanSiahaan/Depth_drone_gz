"""ROS adapter regressions; skipped for the standalone non-ROS test environment."""
import numpy as np
import pytest
import json
from unittest.mock import Mock

rclpy = pytest.importorskip('rclpy')
pytest.importorskip('mavros_msgs')
from std_msgs.msg import Header
from sensor_msgs_py import point_cloud2
from sensor_msgs.msg import Imu, PointField
from builtin_interfaces.msg import Time
from polinasi_nav.ros_nodes import (
    LocalisationNode, ExternalNavNode, NavigationNode, RangeBridge, SensorGate,
    cloud_points,
)


@pytest.fixture(scope='module', autouse=True)
def ros_context():
    rclpy.init(args=['--ros-args', '-p', 'use_sim_time:=true'])
    yield
    rclpy.shutdown()


def test_navigation_initialises_and_publishes_visualisation():
    node = NavigationNode()
    try:
        # Regression: these methods accidentally belonged to SensorGate,
        # causing NavigationNode startup to fail at create_timer().
        node.visualise()
        path = node.path_message([[1., 2., 3.]])
        assert path.header.frame_id == 'map'
        assert path.poses[0].pose.position.z == 3.
        assert node.final_pub.topic_name == '/mavros/setpoint_position/local'
    finally:
        node.destroy_node()


def test_simulation_ray_gate_preserves_raw_cloud_stamp_and_dropout():
    from polinasi_nav.simulated_rays import directions
    node = SensorGate()
    try:
        node.c['simulation_no_return_rays'] = True
        node.pub, node.rays_pub, node.ray_status_pub = Mock(), Mock(), Mock()
        header = Header(frame_id='mid360', stamp=Time(sec=7, nanosec=123))
        xyz = directions()*5.
        xyz[100] = np.copysign(np.inf, directions()[100])
        fields = [PointField(name=n, offset=i*4, datatype=PointField.FLOAT32, count=1)
                  for i, n in enumerate(('x', 'y', 'z'))]
        fields.append(PointField(name='ring', offset=12, datatype=PointField.UINT16, count=1))
        rows = [(*p, int(r)) for p, r in zip(xyz, np.repeat(np.arange(32), 360))]
        msg = point_cloud2.create_cloud(header, fields, rows)
        msg.width, msg.height, msg.row_step = 360, 32, 360*msg.point_step
        node.cloud(msg)
        assert node.pub.publish.call_args.args[0] is msg
        decoded = node.rays_pub.publish.call_args.args[0]
        assert decoded.header == header
        assert json.loads(node.ray_status_pub.publish.call_args.args[0].data)['explicit_no_return'] == 1
        assert len(cloud_points(decoded)) == 11520
        # Invalid dimensions fail closed: no fabricated max-range rays.
        msg.height = 1
        node.cloud(msg)
        assert node.rays_pub.publish.call_args.args[0] is msg
        assert 'error' in json.loads(node.ray_status_pub.publish.call_args.args[0].data)
        node.set_parameters([rclpy.parameter.Parameter('drop_after', value=1.)])
        node.started_at = -2.
        node.pub.publish.reset_mock()
        node.rays_pub.publish.reset_mock()
        node.cloud(msg)
        node.pub.publish.assert_not_called()
        node.rays_pub.publish.assert_not_called()
    finally:
        node.destroy_node()


def test_mapping_roi_observes_first_corridor_before_takeoff():
    from types import SimpleNamespace
    from pathlib import Path
    from polinasi_nav.config import load_config
    from polinasi_nav.corridor_mapping import CorridorRegion, contains
    from polinasi_nav.mapping_io import MappingIONode
    c = load_config(Path(__file__).resolve().parents[1]/'config/predictive_tour_check.json')
    fake = SimpleNamespace(c=c, corridor=CorridorRegion(c), pose=np.array([0., 0., .2]),
        last_status={'mission': 'TAKEOFF', 'survey_waypoint': 0}, planned=[],
        last_integrated_cloud=1., controller=SimpleNamespace(waypoints=np.asarray(c['survey_waypoints'])))
    roi = MappingIONode.mapping_region(fake)
    # Low cells below the future footprint must be retained during ascent,
    # before the LiDAR's narrow downward coverage makes them difficult to see.
    assert contains([[-.9, -2.3, 1.1], [0., 0., 2.9]], roi).all()


def test_identification_partial_log_write_uses_cached_graph_after_shutdown(tmp_path):
    from types import SimpleNamespace
    from collections import deque
    from polinasi_nav.identification_recorder import IdentificationRecorder
    recorder = SimpleNamespace(status={}, armed_seen=False, land_seen=False,
        truncated=False, c={'identification_empty_arena': True},
        commands=[], responses=[], imu_samples=[], events=[], models=deque(),
        directory=tmp_path, single_producer_seen=True, producer_violation=False,
        get_logger=Mock(return_value=Mock()),
        count_publishers=Mock(side_effect=RuntimeError('ROS context is invalid')))
    IdentificationRecorder.write(recorder)
    report = json.loads((tmp_path/'identification.json').read_text())
    assert recorder.written and not report['completed']
    assert not report['lidar_enabled'] and report['one_final_publisher']
    recorder.count_publishers.assert_not_called()


def test_empty_arena_landing_confirmation_is_not_a_rangefinder_bypass_for_mapping():
    from types import SimpleNamespace
    node = SimpleNamespace(identification_empty_arena=True, pose=np.array([0., 0., .2]),
        state=SimpleNamespace(mode='LAND'), velocity=np.zeros(3),
        c=dict(initial_pose=[0., 0., .195], pose_timeout=.3, range_timeout=.3),
        controller=SimpleNamespace(health=SimpleNamespace(pose_stamp=1.)),
        range_value=None, range_stamp=-float('inf'))
    assert NavigationNode.landing_confirmed(node, 1.1)
    assert not NavigationNode.landing_confirmed(node, 2.)
    node.pose[2] = 2.4
    assert not NavigationNode.landing_confirmed(node, 1.1)
    node.pose[2] = .2
    node.identification_empty_arena = False
    assert not NavigationNode.landing_confirmed(node, 1.1)


def test_logged_executed_path_uses_numerical_measurement_time_map_transform(tmp_path):
    from nav_msgs.msg import Odometry
    from geometry_msgs.msg import TransformStamped
    from polinasi_nav.map_log import MapLog
    node = NavigationNode()
    try:
        node.map_log = MapLog(tmp_path)
        transform = TransformStamped()
        transform.transform.translation.x = 10.
        transform.transform.rotation.z = np.sin(np.pi/4)
        transform.transform.rotation.w = np.cos(np.pi/4)
        node.buffer.lookup_transform = Mock(return_value=transform)
        msg = Odometry(header=Header(frame_id='odom', stamp=Time(sec=1)), child_frame_id='base_link')
        msg.pose.pose.position.x, msg.pose.pose.position.z = 1., 2.
        msg.pose.pose.orientation.w = 1.
        node.odom(msg)
        np.testing.assert_allclose(node.map_log.executed_path, [[10., 1., 2.]], atol=1e-9)
        np.testing.assert_allclose(node.pose, [1., 0., 2.])  # Logging never modifies control.
        assert node.map_log.executed_times == [1.]
        assert node.flight_path_snapshot()['frame'] == 'map'
        args = node.buffer.lookup_transform.call_args.args
        assert args[:2] == ('map', 'odom') and args[2].nanoseconds == 1000000000
    finally:
        node.map_log = None  # Node was created without an export worker.
        node.destroy_node()


@pytest.mark.parametrize('node_type', [LocalisationNode, ExternalNavNode, RangeBridge, SensorGate])
def test_other_adapters_initialise(node_type):
    node = node_type()
    node.destroy_node()


def test_real_pointcloud2_decode():
    points = [(1., 2., 3.), (2., 3., 4.)]
    cloud = point_cloud2.create_cloud_xyz32(Header(frame_id='mid360'), points+[(float('inf'), 0., 0.), (float('nan'), 1., 1.)])
    np.testing.assert_allclose(cloud_points(cloud), points)


def test_imu_prediction_never_advances_past_scan_timestamp():
    node = LocalisationNode()
    try:
        node.now = lambda: 1.
        for nanosecond in (980000000, 990000000, 1000000000):
            msg = Imu(header=Header(frame_id='mid360_imu', stamp=Time(sec=nanosecond//1000000000, nanosec=nanosecond%1000000000)))
            msg.linear_acceleration.z = 9.81
            node.imu(msg)
        cloud = point_cloud2.create_cloud_xyz32(Header(frame_id='mid360', stamp=Time(nanosec=990000000)), [(2., 0., 0.)])
        node.scan(cloud)
        assert node.estimator.last_imu == pytest.approx(0.99)
        assert node.pending_imu[0][0] == 1.
    finally:
        node.destroy_node()


def test_startup_without_pose_publishes_wait_status_and_buzzer_off():
    node = NavigationNode()
    try:
        node.now = lambda: 1.
        node.last_tick = 0.95
        node.status_pub, node.buzzer_pub = Mock(), Mock()
        node.tick()
        status = json.loads(node.status_pub.publish.call_args.args[0].data)
        assert status['health_reason'] == 'waiting_for_localisation'
        assert status['flight'] == 'WAIT_EXTERNALNAV'
        assert not node.buzzer_pub.publish.call_args.args[0].data
    finally:
        node.destroy_node()


def test_sparse_fresh_cloud_is_not_dropout_or_free_space_evidence():
    node = NavigationNode()
    try:
        node.now = lambda: 1.
        before = node.grid.state.copy()
        cloud = point_cloud2.create_cloud_xyz32(Header(frame_id='mid360', stamp=Time(nanosec=990000000)), [(2., 0., 0.)])
        node.cloud(cloud)
        assert node.controller.health.cloud_stamp == pytest.approx(.99)
        assert len(node.pending) == 1
        np.testing.assert_array_equal(node.grid.state, before)
        old = point_cloud2.create_cloud_xyz32(Header(frame_id='mid360', stamp=Time(nanosec=100000000)), [(2., 0., 0.)])
        node.cloud(old)
        assert len(node.pending) == 1
    finally:
        node.destroy_node()


def test_voxel_visualisation_uses_fine_cell_size_and_bounded_markers():
    node = NavigationNode()
    try:
        node.map_pub = Mock()
        node.visualise()
        markers = node.map_pub.publish.call_args.args[0].markers
        assert len(markers) == 3
        for marker in markers:
            assert marker.scale.x == pytest.approx(.2)
            assert marker.scale.y == pytest.approx(.2)
            assert marker.scale.z == pytest.approx(.2)
            assert len(marker.points) <= 6000
    finally:
        node.destroy_node()


def test_duplicate_sim_timestamp_does_not_advance_or_abort():
    node = NavigationNode()
    try:
        node.now = lambda: 1.
        node.last_tick = 1.
        node.flight_stage = 'MISSION'
        node.controller.fail = Mock()
        node.final_pub = Mock()
        node.tick()
        node.controller.fail.assert_not_called()
        node.final_pub.publish.assert_not_called()
    finally:
        node.destroy_node()


def test_real_timing_gap_is_still_rejected_and_reported():
    node = NavigationNode()
    try:
        node.now = lambda: 1.
        node.last_tick = .5
        node.flight_stage = 'MISSION'
        node.status_pub = Mock()
        node.tick()
        assert node.controller.failure == 'clock_or_scheduler_gap'
        assert node.last_status['last_timing_gap_seconds'] == .5
    finally:
        node.destroy_node()
