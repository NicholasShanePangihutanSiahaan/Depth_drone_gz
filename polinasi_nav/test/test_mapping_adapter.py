import pytest
import numpy as np
from unittest.mock import Mock

rclpy = pytest.importorskip('rclpy')
pytest.importorskip('mavros_msgs')
from geometry_msgs.msg import PoseStamped
from polinasi_nav.config import load_config
from polinasi_nav import ros_nodes
from polinasi_nav.mavros_configurator import MavrosConfigurator, PLUGIN_SETTINGS
from concurrent.futures import Future


def test_pid_node_owns_no_mpc_worker_and_only_one_final_publisher(monkeypatch):
    from pathlib import Path
    c = load_config(Path(__file__).resolve().parents[1]/'config/pid_tour_check.json')
    monkeypatch.setattr(ros_nodes, 'load_config', lambda _: c)
    rclpy.init(args=['--ros-args', '-p', 'use_sim_time:=true', '-p', 'separate_mapping:=true'])
    node = ros_nodes.MappingNavigationNode()
    try:
        assert node.pid is not None and node.predictive is None
        assert [p.topic_name for p in node.publishers].count('/mavros/setpoint_raw/local') == 1
        assert node.pid.status()['active_controller'] == 'PID_PVA_feedback'
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_raw_target_rotates_derivatives_without_translation_or_double_ned():
    from builtin_interfaces.msg import Time
    from mavros_msgs.msg import PositionTarget
    from polinasi_nav.frames import FCULocalAlignment
    alignment = FCULocalAlignment([0, 0, 0], 0., [10, 20, 30], np.pi/2)
    msg = ros_nodes.raw_position_target([1, 0, 2], [1, 0, 0], [0, 1, 0], 0., alignment, Time(sec=5))
    np.testing.assert_allclose(ros_nodes.vector(msg.position), [10, 21, 32])
    np.testing.assert_allclose(ros_nodes.vector(msg.velocity), [0, 1, 0], atol=1e-9)
    np.testing.assert_allclose(ros_nodes.vector(msg.acceleration_or_force), [-1, 0, 0], atol=1e-9)
    assert msg.type_mask == PositionTarget.IGNORE_YAW_RATE
    assert not msg.type_mask & PositionTarget.FORCE
    assert msg.coordinate_frame == PositionTarget.FRAME_LOCAL_NED
    assert msg.header.frame_id == 'fcu_local' and msg.header.stamp.sec == 5
    assert msg.yaw == pytest.approx(np.pi/2)
    with pytest.raises(ValueError):
        ros_nodes.raw_position_target([0, 0, 0], [0, 0, 0], [0, 0, 0], 0., None, Time())


def test_mapping_node_is_separate_and_accepts_fcu_frame(monkeypatch):
    rclpy.init(args=['--ros-args', '-p', 'use_sim_time:=true'])
    config = load_config()
    config.update(survey_waypoints=[[.25, 0., 2.]], survey_dwell=.1)
    monkeypatch.setattr(ros_nodes, 'load_config', lambda _: config)
    node = ros_nodes.MappingNavigationNode()
    try:
        assert node.get_name() == 'polinasi_mapping_navigation'
        assert node.controller.airborne_state == 'SURVEY'
        assert node.controller.c['mock_flower_events'] is False
        assert node.final_pub.topic_name == '/mavros/setpoint_raw/local'
        msg = PoseStamped()
        msg.header.frame_id = 'fcu_local'
        msg.pose.orientation.w = 1.
        msg.pose.position.z = .1
        node.fc_pose_cb(msg)
        np.testing.assert_allclose(node.fc_pose, [0., 0., .1])
        node.survey_pub = Mock()
        node.visualise()
        node.visual_futures['local'].result(timeout=30.)
        node.visualise()
        from nav_msgs.msg import Path
        from rclpy.serialization import deserialize_message
        assert deserialize_message(node.survey_pub.publish.call_args.args[0], Path).header.frame_id == 'map'
        node.flight_stage = 'TAKEOFF'
        node.stage_started = 0.
        node.last_flight_cmd = -10.
        node.takeoff_pub = Mock()
        node.pose = np.array([0., 0., .2])
        node.state.armed = True
        node.flight_command(1.)
        node.flight_command(1.4)
        # Mapping home is confirmed at world Z=0, not initial body height.
        assert node.takeoff_pub.publish.call_args.args[0].data == pytest.approx(config['takeoff_altitude'])
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_mavros_parameter_gate_waits_for_all_plugin_acknowledgements():
    rclpy.init()
    node = MavrosConfigurator()
    try:
        node.pub = Mock()
        for client in node.parameter_clients.values():
            client.service_is_ready = Mock(return_value=False)
        node.tick()
        assert not node.pub.publish.call_args.args[0].data
        for name, settings in PLUGIN_SETTINGS.items():
            future = Future()
            future.set_result(Mock(results=[Mock(successful=True) for _ in settings]))
            node.pending[name] = future
        node.tick()
        assert node.pub.publish.call_args.args[0].data
        assert len(node.done) == len(PLUGIN_SETTINGS)
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_control_gap_guard_retained_with_callback_diagnostics():
    rclpy.init(args=['--ros-args', '-p', 'use_sim_time:=true'])
    node = ros_nodes.NavigationNode()
    try:
        node.flight_stage = 'MISSION'
        node.last_tick = 0.
        node.now = Mock(return_value=.204)
        node.controller.fail = Mock()
        node.publish_status = Mock()
        node.callback_timing.record('local_visual_handoff', .01)
        node.tick()
        node.controller.fail.assert_called_once_with('clock_or_scheduler_gap')
        assert node.last_timing_gap == pytest.approx(.204)
        assert node.timing_gap_context['name'] == 'local_visual_handoff'
    finally:
        node.destroy_node()
        rclpy.shutdown()
