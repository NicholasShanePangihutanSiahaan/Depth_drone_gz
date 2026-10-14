#!/usr/bin/env python3
"""Connect the proven mission goal interface to the optional PRIEST planner."""

import math
import time
from copy import deepcopy

import rclpy
from geometry_msgs.msg import Pose, PoseStamped
from nav_msgs.msg import Path
from pcl_cstm_msg.msg import (
    AxisAlignedElipsoid,
    AxisAlignedElipsoidArray,
    TrackedCylinderArray,
)
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String


ORBIT_STATES = {'START_ORBIT', 'WAIT_ORBIT', 'ALIGN_TO_LAST_ORBIT'}


def select_lookahead_pose(path, current_pose, lookahead):
    """Return the first path pose at least ``lookahead`` metres from the UAV."""
    if path is None or not path.poses:
        return None
    cx = float(current_pose.position.x)
    cy = float(current_pose.position.y)
    cz = float(current_pose.position.z)
    for stamped in path.poses:
        point = stamped.pose.position
        distance = math.sqrt(
            (float(point.x) - cx) ** 2
            + (float(point.y) - cy) ** 2
            + (float(point.z) - cz) ** 2
        )
        if distance >= lookahead:
            return stamped.pose
    return path.poses[-1].pose


def cylinder_to_ellipsoid(tracked, minimum_radius, minimum_half_height):
    """Convert one tracked cylinder into the obstacle contract used by PRIEST."""
    cylinder = tracked.cylinder
    if not cylinder.is_valid:
        return None
    radius = max(minimum_radius, float(cylinder.radius))
    half_height = max(minimum_half_height, 0.5 * float(cylinder.height))
    values = (
        cylinder.pose.position.x,
        cylinder.pose.position.y,
        cylinder.pose.position.z,
        radius,
        half_height,
    )
    if not all(math.isfinite(float(value)) for value in values):
        return None
    obstacle = AxisAlignedElipsoid()
    obstacle.center = deepcopy(cylinder.pose.position)
    obstacle.radii.x = radius
    obstacle.radii.y = radius
    obstacle.radii.z = half_height
    return obstacle


class PriestAvoidanceController(Node):
    """Feed goals/obstacles to PRIEST and expose one safe local target.

    Orbit remains on the field-tested dynamic target path. For all other
    navigation states the controller fails closed: when no fresh PRIEST path
    exists it publishes the current vehicle pose instead of the unchecked goal.
    """

    def __init__(self):
        super().__init__('priest_avoidance_controller')
        self.declare_parameter('publish_rate', 20.0)
        self.declare_parameter('path_timeout', 2.0)
        self.declare_parameter('waypoint_lookahead', 0.60)
        self.declare_parameter('goal_republish_period', 0.50)
        self.declare_parameter('minimum_obstacle_radius', 0.10)
        self.declare_parameter('minimum_obstacle_half_height', 0.25)
        self.path_timeout = float(self.get_parameter('path_timeout').value)
        self.lookahead = float(self.get_parameter('waypoint_lookahead').value)
        self.goal_period = float(
            self.get_parameter('goal_republish_period').value)
        self.minimum_radius = float(
            self.get_parameter('minimum_obstacle_radius').value)
        self.minimum_half_height = float(
            self.get_parameter('minimum_obstacle_half_height').value)

        self.current_pose = None
        self.fsm_goal = None
        self.orbit_goal = None
        self.fsm_state = 'INIT'
        self.path = None
        self.path_received_at = None
        self.last_goal_sent_at = 0.0
        self.last_goal_xyz = None
        self.holding = False

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.create_subscription(
            PoseStamped, '/mavros/local_position/pose',
            self.pose_callback, sensor_qos)
        self.create_subscription(
            PoseStamped, '/navigation/local_goal',
            self.fsm_goal_callback, 10)
        self.create_subscription(
            PoseStamped, '/control/dynamic_target',
            self.orbit_goal_callback, 10)
        self.create_subscription(
            String, '/mission/fsm_state', self.state_callback, 10)
        self.create_subscription(
            Path, '/navigation/priest/trajectory/out',
            self.path_callback, reliable_qos)
        self.create_subscription(
            TrackedCylinderArray, '/global_cylinders',
            self.obstacles_callback, sensor_qos)

        self.goal_publisher = self.create_publisher(
            Pose, '/navigation/priest/linear_goal', reliable_qos)
        self.obstacle_publisher = self.create_publisher(
            AxisAlignedElipsoidArray, '/ellipsoids', sensor_qos)
        self.safe_target_publisher = self.create_publisher(
            PoseStamped, '/control/safe_target_pose', 10)
        rate = max(5.0, float(self.get_parameter('publish_rate').value))
        self.create_timer(1.0 / rate, self.control_loop)
        self.get_logger().info(
            'PRIEST avoidance bridge aktif; orbit memakai jalur legacy.')

    def pose_callback(self, message):
        self.current_pose = message

    def fsm_goal_callback(self, message):
        self.fsm_goal = message

    def orbit_goal_callback(self, message):
        self.orbit_goal = message

    def state_callback(self, message):
        self.fsm_state = message.data

    def path_callback(self, message):
        self.path = message if message.poses else None
        self.path_received_at = time.monotonic()

    def obstacles_callback(self, message):
        output = AxisAlignedElipsoidArray()
        output.header = message.header
        for tracked in message.cylinders:
            obstacle = cylinder_to_ellipsoid(
                tracked, self.minimum_radius, self.minimum_half_height)
            if obstacle is not None:
                output.elipsoids.append(obstacle)
        self.obstacle_publisher.publish(output)

    def publish_linear_goal(self):
        if self.fsm_goal is None:
            return
        position = self.fsm_goal.pose.position
        xyz = (float(position.x), float(position.y), float(position.z))
        now = time.monotonic()
        changed = self.last_goal_xyz is None or any(
            abs(current - previous) > 1.0e-3
            for current, previous in zip(xyz, self.last_goal_xyz)
        )
        if changed or now - self.last_goal_sent_at >= self.goal_period:
            self.goal_publisher.publish(deepcopy(self.fsm_goal.pose))
            self.last_goal_xyz = xyz
            self.last_goal_sent_at = now

    def publish_target(self, pose):
        output = PoseStamped()
        output.header.stamp = self.get_clock().now().to_msg()
        output.header.frame_id = 'map'
        output.pose = deepcopy(pose)
        self.safe_target_publisher.publish(output)

    def control_loop(self):
        if self.current_pose is None:
            return
        if self.fsm_state in ORBIT_STATES:
            if self.orbit_goal is not None:
                self.publish_target(self.orbit_goal.pose)
            return

        if self.fsm_goal is None:
            return
        self.publish_linear_goal()
        path_fresh = (
            self.path is not None
            and self.path_received_at is not None
            and time.monotonic() - self.path_received_at <= self.path_timeout
        )
        selected = (
            select_lookahead_pose(
                self.path, self.current_pose.pose, self.lookahead)
            if path_fresh else None
        )
        if selected is None:
            selected = deepcopy(self.current_pose.pose)
            selected.orientation = deepcopy(self.fsm_goal.pose.orientation)
            if not self.holding:
                self.get_logger().warning(
                    'Belum ada trajectory PRIEST yang fresh; hold posisi.')
            self.holding = True
        else:
            selected.orientation = deepcopy(self.fsm_goal.pose.orientation)
            if self.holding:
                self.get_logger().info(
                    'Trajectory PRIEST diterima; navigasi dilanjutkan.')
            self.holding = False
        self.publish_target(selected)


def main(args=None):
    rclpy.init(args=args)
    node = PriestAvoidanceController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
