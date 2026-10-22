"""Regression for fail-closed actuation and mission/orbit safety handshakes."""
from copy import deepcopy
from types import SimpleNamespace
from unittest.mock import Mock
import time

import numpy as np

from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool

from beehive_drone.depth_avoidance_controller import DepthAvoidanceController
from beehive_drone.position_setpoint_controller import PositionSetpointController
from beehive_drone.mission_state_machine import MissionStateMachine
from beehive_drone.depth_navigation import DepthView, ObstacleMemory


def test_supervisor_missing_depth_holds_during_return_home():
    pose = PoseStamped()
    pose.pose.orientation.w = 1.0
    now = time.monotonic()
    supervisor = SimpleNamespace(
        pose=pose, pose_time=now, depth_time=-float('inf'), latched_fault=None,
        p={'pose_timeout': 0.5, 'depth_timeout': 0.8, 'goal_timeout': 1.0},
        ready_pub=Mock(), state='RETURN_TO_HOME', goals={'fsm': pose},
        goal_times={'fsm': now}, output=Mock())
    DepthAvoidanceController.tick(supervisor)
    supervisor.output.assert_called_once_with(pose, 'DEPTH_STALE', True, True)
    assert not supervisor.ready_pub.publish.call_args.args[0].data


def test_setpoint_watchdog_reanchors_instead_of_slewing_toward_old_command():
    pose = PoseStamped()
    pose.pose.position.x = 1.0
    pose.pose.position.z = 2.0
    target = deepcopy(pose)
    target.pose.position.x = 8.0
    clock_time = SimpleNamespace(to_msg=lambda: pose.header.stamp)
    node = SimpleNamespace(
        pose=pose, enabled=True, require_depth=True, avoidance_hold=False,
        avoidance_time=time.monotonic()-2.0, depth_hold_pose=None,
        commanded_pose=target, target=target, target_time=None,
        get_clock=lambda: SimpleNamespace(now=lambda: clock_time),
        copy_pose=PositionSetpointController.copy_pose, pub=Mock(), output_frame='map')
    PositionSetpointController.loop(node)
    published = node.pub.publish.call_args.args[0]
    assert published.pose.position.x == 1.0
    assert published.pose.position.z == 2.0
    # A later observation drifting forward does not move the latched hold goal.
    node.pose.pose.position.x = 1.2
    PositionSetpointController.loop(node)
    assert node.pub.publish.call_args.args[0].pose.position.x == 1.0


def test_preflight_requires_depth_and_heartbeat_when_enabled():
    node = SimpleNamespace(require_depth_avoidance=True, depth_ready=False,
                           depth_ready_time=None)
    assert not MissionStateMachine.depth_ready_for_start(node)
    node.require_depth_avoidance = False
    assert MissionStateMachine.depth_ready_for_start(node)


def test_hold_message_updates_independent_wallclock_watchdog():
    node = SimpleNamespace(avoidance_hold=False, avoidance_time=-1.0)
    PositionSetpointController.avoidance_cb(node, Bool(data=True))
    assert node.avoidance_hold
    assert time.monotonic()-node.avoidance_time < 0.1


def test_occupied_approach_point_uses_alternative_entry_on_same_circle():
    pose = PoseStamped()
    pose.pose.orientation.w = 1.0
    pose.pose.position.z = 1.5
    goal = deepcopy(pose)
    goal.pose.position.x = 2.4  # trunk at (5, 0), radius 2.6
    now = time.monotonic()
    node = SimpleNamespace(
        pose=pose, pose_time=now, depth_time=now, latched_fault=None,
        p={'pose_timeout': .5, 'depth_timeout': .8, 'goal_timeout': 1.,
           'vehicle_half_height': .3, 'vehicle_radius': .4, 'clearance_margin': .35,
           'grid_resolution': .2, 'planning_radius': 7., 'orbit_radius': 2.6,
           'lookahead': .65, 'yaw_tolerance_degrees': 12., 'blocked_timeout': 30.},
        state='APPROACH_TREE', goals={'fsm': goal}, goal_times={'fsm': now},
        ready_pub=Mock(), output=Mock(), orbit_centre=None, detour_goal=None,
        waypoint=None, blocked_since=None,
        yaw=DepthAvoidanceController.yaw,
        memory=SimpleNamespace(slice=lambda *args: np.array([[2.4, 0.]])),
        view=DepthView(np.full((80,100), 10.), (60.,60.,49.5,39.5),
                       np.array([0.,0.,1.5]), np.eye(3)))
    DepthAvoidanceController.tick(node)
    assert node.detour_goal is not None
    assert abs(np.linalg.norm(node.detour_goal-[5.,0.])-2.6) < 1e-6
    assert node.output.call_args.args[1] in ('DETOUR', 'LOOK_BEFORE_MOVE')
