#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped

from tf2_ros import TransformBroadcaster

from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    HistoryPolicy
)


class TFBroadcaster(Node):

    def __init__(self):
        super().__init__("tf_broadcaster")

        qos_sensor = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.tf_broadcaster = TransformBroadcaster(self)

        self.create_subscription(
            Odometry,
            "/localization/odom",
            self.odom_callback,
            qos_sensor
        )

        self.get_logger().info("TF Broadcaster Started")

    def odom_callback(self, msg):

        # Abaikan apabila timestamp belum valid
        if msg.header.stamp.sec == 0 and msg.header.stamp.nanosec == 0:
            return

        transform = TransformStamped()

        transform.header.stamp = msg.header.stamp
        transform.header.frame_id = "odom"

        transform.child_frame_id = "base_link"

        transform.transform.translation.x = msg.pose.pose.position.x
        transform.transform.translation.y = msg.pose.pose.position.y
        transform.transform.translation.z = msg.pose.pose.position.z

        transform.transform.rotation = msg.pose.pose.orientation

        self.tf_broadcaster.sendTransform(transform)


def main(args=None):

    rclpy.init(args=args)

    node = TFBroadcaster()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()