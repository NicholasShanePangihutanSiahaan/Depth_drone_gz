#!/usr/bin/env python3

import math

import rclpy
from geometry_msgs.msg import PointStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener

from beehive_drone.mission_params import MissionConfig


def rotate_vector_by_quaternion(x: float, y: float, z: float, q):
    # Efficient q * v * q^-1 expansion.
    tx = 2.0 * (q.y * z - q.z * y)
    ty = 2.0 * (q.z * x - q.x * z)
    tz = 2.0 * (q.x * y - q.y * x)
    rx = x + q.w * tx + (q.y * tz - q.z * ty)
    ry = y + q.w * ty + (q.z * tx - q.x * tz)
    rz = z + q.w * tz + (q.x * ty - q.y * tx)
    return rx, ry, rz


class TreeLocalizer(Node):
    """Transforms camera optical points into the mission world frame using TF2."""

    def __init__(self) -> None:
        super().__init__("tree_localizer")
        self.declare_parameter("world_frame", MissionConfig.WORLD_FRAME)
        self.declare_parameter("input_topic", "/perception/tree_position_camera")
        self.declare_parameter("output_topic", "/perception/tree_position_world")
        self.world_frame = str(self.get_parameter("world_frame").value)
        input_topic = str(self.get_parameter("input_topic").value)
        output_topic = str(self.get_parameter("output_topic").value)

        self.tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.publisher = self.create_publisher(PointStamped, output_topic, 10)
        self.create_subscription(
            PointStamped, input_topic, self.point_callback, qos_profile_sensor_data
        )
        self.last_warning_time = -1e9
        self.get_logger().info(
            f"Tree Localizer TF aktif: {input_topic} -> {output_topic} ({self.world_frame})."
        )

    def now_sec(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def point_callback(self, msg: PointStamped) -> None:
        if not msg.header.frame_id:
            self.get_logger().warning("Point kamera tanpa frame_id diabaikan.")
            return
        if not all(
            math.isfinite(float(v)) for v in (msg.point.x, msg.point.y, msg.point.z)
        ):
            return

        try:
            transform = self.tf_buffer.lookup_transform(
                self.world_frame,
                msg.header.frame_id,
                Time.from_msg(msg.header.stamp),
                timeout=Duration(seconds=0.08),
            )
        except TransformException as exc:
            now = self.now_sec()
            if now - self.last_warning_time > 1.0:
                self.last_warning_time = now
                self.get_logger().warning(f"TF point kamera gagal: {exc}")
            return

        rotation = transform.transform.rotation
        rx, ry, rz = rotate_vector_by_quaternion(
            msg.point.x, msg.point.y, msg.point.z, rotation
        )
        translation = transform.transform.translation

        output = PointStamped()
        output.header.stamp = msg.header.stamp
        output.header.frame_id = self.world_frame
        output.point.x = float(rx + translation.x)
        output.point.y = float(ry + translation.y)
        output.point.z = float(rz + translation.z)
        self.publisher.publish(output)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TreeLocalizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
