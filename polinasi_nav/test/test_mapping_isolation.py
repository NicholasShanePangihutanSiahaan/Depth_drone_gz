from array import array
from unittest.mock import Mock
import numpy as np
import pytest

rclpy = pytest.importorskip('rclpy')
pytest.importorskip('mavros_msgs')
from std_msgs.msg import UInt8MultiArray
from polinasi_nav import ros_nodes
from polinasi_nav.config import load_config
from polinasi_nav.map_transport import encode_window
from polinasi_nav.mapping_io import MappingIONode


def configuration():
    c = load_config()
    c.update(survey_waypoints=[[.25, 0., 2.]], survey_dwell=.1,
             navigation_window_size=[4., 4., 4.], map_timeout=1.)
    return c


def test_control_process_does_not_own_mapping_visual_export_work(monkeypatch):
    rclpy.init(args=['--ros-args', '-p', 'use_sim_time:=true', '-p', 'separate_mapping:=true'])
    monkeypatch.setattr(ros_nodes, 'load_config', lambda _: configuration())
    node = ros_nodes.MappingNavigationNode()
    try:
        assert node.separate_mapping
        assert node.map_worker is node.visual_worker is node.log_worker is None
        assert node.history is node.map_log is None
        publishers = [p.topic_name for p in node.publishers]
        assert publishers.count('/mavros/setpoint_raw/local') == 1
        assert '/navigation/occupancy' not in publishers
        assert '/mapping/global_cloud' not in publishers
        assert '/navigation/executed_path' not in publishers
        node.now = Mock(return_value=3.)
        node.grid.rebuild(3.)
        node.grid.last_observation_stamp = 2.9
        node.grid.version = 8
        trace = dict(received_ns=1, submitted_ns=2, worker_started_ns=3,
                     worker_finished_ns=4, integrated_ns=5, scan_stamp=2.9)
        message = UInt8MultiArray(data=array('B', encode_window(node.grid,
            {'global_known_voxels': 100, 'pipeline_trace': trace})))
        node.navigation_snapshot(message)
        assert node.last_integrated_cloud == 2.9 and node.map_statistics['global_known_voxels'] == 100
        assert node.pipeline_trace['received_ns'] == 1
        assert node.pipeline_trace['navigation_received_ns'] > 5
        assert 'navigation_received_ns' not in trace
        previous = node.grid
        node.navigation_snapshot(message)
        assert node.grid is previous  # Duplicate snapshots never reset freshness.
        node.now = Mock(return_value=6.)
        node.grid.last_observation_stamp = 3.1
        node.navigation_snapshot(UInt8MultiArray(data=array('B', encode_window(node.grid, {}))))
        assert node.last_integrated_cloud == 2.9
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_mapping_process_has_no_control_or_flight_command_publishers(monkeypatch):
    rclpy.init(args=['--ros-args', '-p', 'use_sim_time:=true'])
    monkeypatch.setattr(ros_nodes, 'load_config', lambda _: configuration())
    node = MappingIONode()
    try:
        topics = [p.topic_name for p in node.publishers]
        assert '/mapping/navigation_snapshot' in topics and '/navigation/occupancy' in topics
        assert not any(t.startswith('/mavros/setpoint') or t.startswith('/flight/cmd') or
                       t == '/control/safe_target_pose' for t in topics)
        assert node.pending.maxlen == 1
        node.state.armed = True
        node.fcu_state(node.state)
        assert node.flight_stage == 'MISSION'
    finally:
        node.destroy_node()
        rclpy.shutdown()
