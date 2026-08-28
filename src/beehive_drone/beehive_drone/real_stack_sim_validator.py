#!/usr/bin/env python3
"""Preflight validator for the Gazebo version of the real ROS stack."""

import json
import math

import rclpy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from nav_msgs.msg import Odometry
from pcl_cstm_msg.msg import TrackedCylinderArray
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Range
from std_msgs.msg import Bool, String
from uav_interfaces.msg import TreeArray
from zed_msgs.msg import ObjectsStamped


def position_error(first, second):
    """Return Euclidean position error between pose-like objects."""
    return math.sqrt(
        (first.position.x - second.position.x) ** 2 +
        (first.position.y - second.position.y) ** 2 +
        (first.position.z - second.position.z) ** 2)


def orientation_error_degrees(first, second):
    """Return shortest angular difference between two ROS quaternions."""
    left = (first.x, first.y, first.z, first.w)
    right = (second.x, second.y, second.z, second.w)
    left_norm = math.sqrt(sum(value * value for value in left))
    right_norm = math.sqrt(sum(value * value for value in right))
    if left_norm < 1.0e-12 or right_norm < 1.0e-12:
        return float('inf')
    dot = abs(sum(left[index] * right[index] for index in range(4)) /
              (left_norm * right_norm))
    return math.degrees(2.0 * math.acos(max(-1.0, min(1.0, dot))))


class RealStackSimulationValidator(Node):
    """Check topic freshness, pose forwarding, mapping, and publisher count."""

    def __init__(self):
        super().__init__('real_stack_sim_validator')
        defaults = {
            'expected_tree_x': 7.0,
            'expected_tree_y': 0.0,
            'freshness_timeout': 2.0,
            'maximum_pose_error': 0.20,
            'maximum_orientation_error_degrees': 2.0,
            'maximum_local_pose_error': 0.50,
            'maximum_local_orientation_error_degrees': 10.0,
            'maximum_tree_xy_error': 0.35,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.expected_tree = (
            float(self.get_parameter('expected_tree_x').value),
            float(self.get_parameter('expected_tree_y').value))
        self.timeout = float(self.get_parameter('freshness_timeout').value)
        self.max_pose_error = float(
            self.get_parameter('maximum_pose_error').value)
        self.max_orientation_error = float(
            self.get_parameter('maximum_orientation_error_degrees').value)
        self.max_local_pose_error = float(
            self.get_parameter('maximum_local_pose_error').value)
        self.max_local_orientation_error = float(self.get_parameter(
            'maximum_local_orientation_error_degrees').value)
        self.max_tree_error = float(
            self.get_parameter('maximum_tree_xy_error').value)

        self.latest = {}
        self.received_at = {}
        self.safety_ok = False
        self.connected = False
        self.last_state = None
        self.last_report_signature = None
        self.report_counter = 0

        self.create_subscription(
            Odometry, '/simulation/ground_truth/odom',
            lambda msg: self.store('ground_truth', msg),
            qos_profile_sensor_data)
        self.create_subscription(
            PoseStamped, '/zed/zed_node/pose',
            lambda msg: self.store('zed_pose', msg), qos_profile_sensor_data)
        self.create_subscription(
            PoseStamped, '/mavros/vision_pose/pose',
            lambda msg: self.store('vision_pose', msg),
            qos_profile_sensor_data)
        self.create_subscription(
            PoseStamped, '/mavros/local_position/pose',
            lambda msg: self.store('local_pose', msg),
            qos_profile_sensor_data)
        self.create_subscription(
            ObjectsStamped, '/zed/zed_node/obj_det/objects',
            lambda msg: self.store('objects', msg), 10)
        self.create_subscription(
            TrackedCylinderArray, '/global_cylinders',
            lambda msg: self.store('cylinders', msg), qos_profile_sensor_data)
        self.create_subscription(
            TreeArray, '/map/trees',
            lambda msg: self.store('trees', msg), 10)
        self.create_subscription(
            Range, '/mavros/rangefinder/rangefinder',
            lambda msg: self.store('range', msg), qos_profile_sensor_data)
        self.create_subscription(
            State, '/mavros/state', self.state_callback, 10)
        self.create_subscription(
            Bool, '/mission/safety_ok', self.safety_callback, 10)
        self.create_subscription(
            String, '/mission/fsm_state',
            lambda msg: self.store('fsm', msg), 10)

        self.ready_publisher = self.create_publisher(
            Bool, '/simulation/real_stack/ready', 10)
        self.status_publisher = self.create_publisher(
            String, '/simulation/real_stack/status', 10)
        self.create_timer(1.0, self.evaluate)

    def store(self, name, message):
        self.latest[name] = message
        self.received_at[name] = self.get_clock().now()

    def state_callback(self, message):
        self.store('mavros_state', message)
        self.connected = bool(message.connected)

    def safety_callback(self, message):
        self.store('safety', message)
        self.safety_ok = bool(message.data)

    def fresh(self, name):
        if name not in self.received_at:
            return False
        age = (
            self.get_clock().now() - self.received_at[name]
        ).nanoseconds * 1e-9
        return age <= self.timeout

    def nearest_tree_error(self):
        trees = self.latest.get('trees')
        if trees is None or not trees.trees:
            return float('inf')
        return min(math.hypot(
            tree.x - self.expected_tree[0], tree.y - self.expected_tree[1])
                   for tree in trees.trees)

    def mavros_vision_plugin_present(self):
        """Return true only when the MAVROS vision plugin subscribes."""
        topic = '/mavros/vision_pose/pose'
        for endpoint in self.get_subscriptions_info_by_topic(topic):
            namespace = endpoint.node_namespace.rstrip('/')
            full_name = f'{namespace}/{endpoint.node_name}'
            if full_name == '/mavros/vision_pose':
                return True
        return False

    def evaluate(self):
        streaming_topics = (
            'ground_truth', 'zed_pose', 'vision_pose', 'local_pose', 'objects',
            'cylinders', 'range', 'mavros_state', 'safety')
        failures = [
            f'{name}_stale' for name in streaming_topics
            if not self.fresh(name)]
        # /map/trees is durable state and is intentionally event-driven.  Its
        # geometric check below proves that a map was received; aging it like
        # a live sensor creates a false failure whenever the map is unchanged.
        if 'trees' not in self.latest:
            failures.append('trees_missing')

        ground = self.latest.get('ground_truth')
        zed = self.latest.get('zed_pose')
        vision = self.latest.get('vision_pose')
        local_pose = self.latest.get('local_pose')
        ground_zed_position_error = float('inf')
        ground_zed_orientation_error = float('inf')
        vision_position_error = float('inf')
        vision_orientation_error = float('inf')
        local_position_error = float('inf')
        local_orientation_error = float('inf')
        if ground is not None and zed is not None:
            ground_zed_position_error = position_error(
                ground.pose.pose, zed.pose)
            ground_zed_orientation_error = orientation_error_degrees(
                ground.pose.pose.orientation, zed.pose.orientation)
        if zed is not None and vision is not None:
            vision_position_error = position_error(zed.pose, vision.pose)
            vision_orientation_error = orientation_error_degrees(
                zed.pose.orientation, vision.pose.orientation)
        if ground is not None and local_pose is not None:
            local_position_error = position_error(
                ground.pose.pose, local_pose.pose)
            local_orientation_error = orientation_error_degrees(
                ground.pose.pose.orientation, local_pose.pose.orientation)

        if ground_zed_position_error > self.max_pose_error:
            failures.append('zed_pose_error')
        if ground_zed_orientation_error > self.max_orientation_error:
            failures.append('zed_orientation_error')
        if vision_position_error > self.max_pose_error:
            failures.append('vision_forwarding_error')
        if vision_orientation_error > self.max_orientation_error:
            failures.append('vision_orientation_error')
        if local_position_error > self.max_local_pose_error:
            failures.append('mavros_local_pose_error')
        if local_orientation_error > self.max_local_orientation_error:
            failures.append('mavros_local_orientation_error')

        tree_error = self.nearest_tree_error()
        if tree_error > self.max_tree_error:
            failures.append('tree_global_error')
        if not self.connected:
            failures.append('mavros_disconnected')
        if not self.safety_ok:
            failures.append('mission_safety_not_ok')
        publisher_count = self.count_publishers('/global_cylinders')
        if publisher_count != 1:
            failures.append(f'global_cylinders_publishers_{publisher_count}')
        vision_plugin_present = self.mavros_vision_plugin_present()
        if not vision_plugin_present:
            failures.append('mavros_vision_plugin_missing')
        range_message = self.latest.get('range')
        state_message = self.latest.get('mavros_state')
        # A downward sensor can legitimately report inf while it is inside its
        # minimum range on the ground.  Match the real safety monitor: require
        # a finite measurement once the vehicle is armed, not before takeoff.
        range_valid = range_message is not None and math.isfinite(
            range_message.range)
        if (
                state_message is not None
                and state_message.armed
                and not range_valid):
            failures.append('range_invalid_while_armed')

        failures = sorted(set(failures))
        ready = not failures
        objects = self.latest.get('objects')
        cylinders = self.latest.get('cylinders')
        trees = self.latest.get('trees')
        status = {
            'ready': ready,
            'failures': failures,
            'mavros_connected': self.connected,
            'mavros_vision_plugin_present': vision_plugin_present,
            'global_cylinders_publishers': publisher_count,
            'objects_in_last_message': (
                0 if objects is None else len(objects.objects)),
            'tracked_cylinders': (
                0 if cylinders is None else len(cylinders.cylinders)),
            'mapped_trees': 0 if trees is None else len(trees.trees),
            'range_valid': range_valid,
            'ground_to_zed_position_error_m': ground_zed_position_error,
            'ground_to_zed_orientation_error_deg': (
                ground_zed_orientation_error),
            'zed_to_vision_position_error_m': vision_position_error,
            'zed_to_vision_orientation_error_deg': vision_orientation_error,
            'ground_to_mavros_local_position_error_m': local_position_error,
            'ground_to_mavros_local_orientation_error_deg': (
                local_orientation_error),
            'tree_xy_error_m': tree_error,
            'fsm_state': None if self.latest.get('fsm') is None else
            self.latest['fsm'].data,
        }
        ready_message = Bool()
        ready_message.data = ready
        self.ready_publisher.publish(ready_message)
        status_message = String()
        status_message.data = json.dumps(status, sort_keys=True)
        self.status_publisher.publish(status_message)

        signature = (ready, tuple(failures))
        self.report_counter += 1
        if signature != self.last_report_signature or \
                self.report_counter % 5 == 0:
            # Keep calls on distinct source lines: rclpy uses the call site as
            # the logger identity and rejects changing its severity later.
            if ready:
                self.get_logger().info('REAL-STACK READY')
            else:
                self.get_logger().warning(
                    f'REAL-STACK belum siap: {", ".join(failures)}')
            self.last_report_signature = signature


def main(args=None):
    rclpy.init(args=args)
    node = RealStackSimulationValidator()
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
