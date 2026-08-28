#!/usr/bin/env python3
"""Publish Gazebo ground truth through the interfaces used by a real ZED2i.

This node is simulation-only.  It deliberately publishes the same pose and
``zed_msgs/ObjectsStamped`` topics consumed by ``vision_to_mavros`` and the BB
perception node, while expressing every synthetic detection in the configured
camera frame.  The downstream transform path is therefore identical to real
flight: camera -> base_link -> map.
"""

import math
import random

import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.qos import qos_profile_sensor_data
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from zed_msgs.msg import Object, ObjectsStamped


def normalize_quaternion(quaternion):
    """Return an ``(x, y, z, w)`` unit quaternion."""
    norm = math.sqrt(sum(component * component for component in quaternion))
    if norm < 1.0e-12:
        return (0.0, 0.0, 0.0, 1.0)
    return tuple(component / norm for component in quaternion)


def quaternion_conjugate(quaternion):
    """Return the conjugate of an ``(x, y, z, w)`` quaternion."""
    x, y, z, w = quaternion
    return (-x, -y, -z, w)


def quaternion_multiply(left, right):
    """Multiply two ``(x, y, z, w)`` quaternions."""
    lx, ly, lz, lw = left
    rx, ry, rz, rw = right
    return (
        lw * rx + lx * rw + ly * rz - lz * ry,
        lw * ry - lx * rz + ly * rw + lz * rx,
        lw * rz + lx * ry - ly * rx + lz * rw,
        lw * rw - lx * rx - ly * ry - lz * rz,
    )


def rotate_vector(quaternion, vector):
    """Rotate a three-dimensional vector with a unit quaternion."""
    quaternion = normalize_quaternion(quaternion)
    vector_quaternion = (vector[0], vector[1], vector[2], 0.0)
    rotated = quaternion_multiply(
        quaternion_multiply(quaternion, vector_quaternion),
        quaternion_conjugate(quaternion))
    return rotated[:3]


def quaternion_from_rpy(roll, pitch, yaw):
    """Create an ``(x, y, z, w)`` quaternion from radians."""
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    return normalize_quaternion((
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    ))


def world_to_camera(point_world, base_position, base_orientation,
                    camera_translation, camera_orientation):
    """Transform a world point into the configured camera frame."""
    world_delta = tuple(
        point_world[index] - base_position[index] for index in range(3))
    point_base = rotate_vector(
        quaternion_conjugate(normalize_quaternion(base_orientation)),
        world_delta)
    camera_delta = tuple(
        point_base[index] - camera_translation[index] for index in range(3))
    return rotate_vector(
        quaternion_conjugate(normalize_quaternion(camera_orientation)),
        camera_delta)


class SimulationZedAdapter(Node):
    """Generate deterministic ZED-like pose and tree detections from Gazebo."""

    def __init__(self):
        super().__init__('sim_zed_adapter')
        defaults = {
            'input_odometry_topic': '/simulation/ground_truth/odom',
            'pose_topic': '/zed/zed_node/pose',
            'objects_topic': '/zed/zed_node/obj_det/objects',
            'global_frame_id': 'map',
            'base_frame_id': 'base_link',
            'camera_frame_id': 'zed_left_camera_frame',
            # SDF base_link -> zed2i_link -> left camera optical centre.
            'camera_x': 0.14,
            'camera_y': 0.06,
            'camera_z': 0.02,
            'camera_roll': 0.0,
            'camera_pitch': 0.0,
            'camera_yaw': 0.0,
            'tree_x': 7.0,
            'tree_y': 0.0,
            'tree_ground_z': 0.0,
            'tree_height': 6.24,
            'tree_width': 2.40,
            'tree_depth': 2.40,
            'object_label': 'pohon',
            'object_confidence': 90.0,
            'detection_rate_hz': 10.0,
            'minimum_detection_range': 0.25,
            'maximum_detection_range': 15.0,
            'horizontal_fov_degrees': 80.0,
            'vertical_fov_degrees': 64.0,
            'position_noise_stddev': 0.0,
            'dropout_every_n': 0,
            'random_seed': 23,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

        self.input_topic = str(
            self.get_parameter('input_odometry_topic').value)
        pose_topic = str(self.get_parameter('pose_topic').value)
        objects_topic = str(self.get_parameter('objects_topic').value)
        self.global_frame = str(self.get_parameter('global_frame_id').value)
        self.base_frame = str(self.get_parameter('base_frame_id').value)
        self.camera_frame = str(self.get_parameter('camera_frame_id').value)
        self.camera_translation = tuple(float(
            self.get_parameter(f'camera_{axis}').value) for axis in 'xyz')
        self.camera_orientation = quaternion_from_rpy(*(
            float(self.get_parameter(f'camera_{name}').value)
            for name in ('roll', 'pitch', 'yaw')))
        self.tree_ground = (
            float(self.get_parameter('tree_x').value),
            float(self.get_parameter('tree_y').value),
            float(self.get_parameter('tree_ground_z').value),
        )
        self.tree_height = float(self.get_parameter('tree_height').value)
        self.tree_width = float(self.get_parameter('tree_width').value)
        self.tree_depth = float(self.get_parameter('tree_depth').value)
        self.object_label = str(self.get_parameter('object_label').value)
        self.object_confidence = float(
            self.get_parameter('object_confidence').value)
        rate = max(0.5, float(self.get_parameter('detection_rate_hz').value))
        self.detection_period_ns = int(1.0e9 / rate)
        self.minimum_range = max(
            0.0, float(self.get_parameter('minimum_detection_range').value))
        self.maximum_range = max(
            self.minimum_range,
            float(self.get_parameter('maximum_detection_range').value))
        self.horizontal_half_fov = math.radians(
            float(self.get_parameter('horizontal_fov_degrees').value) * 0.5)
        self.vertical_half_fov = math.radians(
            float(self.get_parameter('vertical_fov_degrees').value) * 0.5)
        self.noise_stddev = max(
            0.0, float(self.get_parameter('position_noise_stddev').value))
        self.dropout_every_n = max(
            0, int(self.get_parameter('dropout_every_n').value))
        self.random = random.Random(
            int(self.get_parameter('random_seed').value))

        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=20)
        self.pose_publisher = self.create_publisher(
            PoseStamped, pose_topic, reliable_qos)
        self.objects_publisher = self.create_publisher(
            ObjectsStamped, objects_topic, reliable_qos)
        self.create_subscription(
            Odometry, self.input_topic, self.odometry_callback,
            qos_profile_sensor_data)

        self.static_broadcaster = StaticTransformBroadcaster(self)
        self.publish_camera_transform()
        self.last_detection_ns = None
        self.detection_cycle = 0
        self.pose_count = 0
        self.object_count = 0
        self.visible_count = 0
        self.create_timer(2.0, self.report)
        self.get_logger().info(
            f'Simulated ZED: {self.input_topic} -> {pose_topic}, '
            f'{objects_topic}; camera={self.camera_frame}, tree='
            f'({self.tree_ground[0]:.2f},{self.tree_ground[1]:.2f})')

    def publish_camera_transform(self):
        """Publish the mounting transform consumed by the real BB node."""
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = self.base_frame
        transform.child_frame_id = self.camera_frame
        transform.transform.translation.x = self.camera_translation[0]
        transform.transform.translation.y = self.camera_translation[1]
        transform.transform.translation.z = self.camera_translation[2]
        transform.transform.rotation.x = self.camera_orientation[0]
        transform.transform.rotation.y = self.camera_orientation[1]
        transform.transform.rotation.z = self.camera_orientation[2]
        transform.transform.rotation.w = self.camera_orientation[3]
        self.static_broadcaster.sendTransform(transform)

    def odometry_callback(self, odometry):
        """Publish ZED pose every sample and objects at the camera rate."""
        stamp = self.get_clock().now().to_msg()
        pose = PoseStamped()
        pose.header.stamp = stamp
        pose.header.frame_id = self.global_frame
        pose.pose = odometry.pose.pose
        self.pose_publisher.publish(pose)
        self.pose_count += 1

        now_ns = self.get_clock().now().nanoseconds
        if self.last_detection_ns is not None and \
                now_ns - self.last_detection_ns < self.detection_period_ns:
            return
        self.last_detection_ns = now_ns
        self.publish_objects(pose)

    def tree_corners_world(self):
        """Return corners ordered like the edges used by the BB processor."""
        half_width = 0.5 * self.tree_width
        half_depth = 0.5 * self.tree_depth
        lower = self.tree_ground[2]
        upper = lower + self.tree_height
        center_x, center_y = self.tree_ground[:2]
        return [
            (center_x - half_width, center_y - half_depth, upper),
            (center_x - half_width, center_y + half_depth, upper),
            (center_x + half_width, center_y + half_depth, upper),
            (center_x + half_width, center_y - half_depth, upper),
            (center_x - half_width, center_y - half_depth, lower),
            (center_x - half_width, center_y + half_depth, lower),
            (center_x + half_width, center_y + half_depth, lower),
            (center_x + half_width, center_y - half_depth, lower),
        ]

    def tree_visible(self, center_camera):
        """Apply a simple range and camera-FOV visibility model."""
        forward, left, up = center_camera
        distance = math.sqrt(forward * forward + left * left + up * up)
        if distance < self.minimum_range or distance > self.maximum_range:
            return False
        if forward <= 0.0:
            return False
        horizontal_angle = abs(math.atan2(left, forward))
        vertical_angle = abs(math.atan2(up, math.hypot(forward, left)))
        return horizontal_angle <= self.horizontal_half_fov and \
            vertical_angle <= self.vertical_half_fov

    def publish_objects(self, pose):
        """Publish one tree bounding box, or an empty heartbeat when unseen."""
        self.detection_cycle += 1
        objects = ObjectsStamped()
        objects.header.stamp = pose.header.stamp
        objects.header.frame_id = self.camera_frame

        base_position = (
            pose.pose.position.x, pose.pose.position.y, pose.pose.position.z)
        base_orientation = (
            pose.pose.orientation.x, pose.pose.orientation.y,
            pose.pose.orientation.z, pose.pose.orientation.w)
        center_world = (
            self.tree_ground[0], self.tree_ground[1],
            self.tree_ground[2] + 0.5 * self.tree_height)
        center_camera = world_to_camera(
            center_world, base_position, base_orientation,
            self.camera_translation, self.camera_orientation)
        dropped = self.dropout_every_n > 0 and \
            self.detection_cycle % self.dropout_every_n == 0

        if self.tree_visible(center_camera) and not dropped:
            detection = Object()
            detection.label = self.object_label
            detection.label_id = 1
            detection.confidence = self.object_confidence
            detection.tracking_available = True
            detection.tracking_state = 1
            noise = tuple(
                self.random.gauss(0.0, self.noise_stddev) for _ in range(3))
            detection.position = [
                float(center_camera[index] + noise[index])
                for index in range(3)]
            detection.position_covariance = [
                float(self.noise_stddev * self.noise_stddev), 0.0, 0.0,
                float(self.noise_stddev * self.noise_stddev), 0.0,
                float(self.noise_stddev * self.noise_stddev)]
            detection.dimensions_3d = [
                self.tree_width, self.tree_height, self.tree_depth]
            for index, corner_world in enumerate(self.tree_corners_world()):
                corner_camera = world_to_camera(
                    corner_world, base_position, base_orientation,
                    self.camera_translation, self.camera_orientation)
                detection.bounding_box_3d.corners[index].kp = [
                    float(corner_camera[axis] + noise[axis])
                    for axis in range(3)]
            objects.objects.append(detection)
            self.visible_count += 1

        self.objects_publisher.publish(objects)
        self.object_count += 1

    def report(self):
        """Report rates without flooding the console."""
        self.get_logger().info(
            f'Simulated ZED active: pose={self.pose_count}, '
            f'objects={self.object_count}, visible={self.visible_count}')


def main(args=None):
    rclpy.init(args=args)
    node = SimulationZedAdapter()
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
