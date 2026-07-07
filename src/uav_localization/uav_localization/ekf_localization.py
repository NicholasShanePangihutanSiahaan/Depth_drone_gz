#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry

from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy


class EKFLocalization(Node):

    def __init__(self):
        super().__init__("ekf_localization")

        qos_sensor = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.counter = 0

        self.pose_sub = self.create_subscription(
            PoseStamped,
            "/mavros/local_position/pose",
            self.pose_callback,
            qos_sensor
        )

        self.odom_pub = self.create_publisher(
            Odometry,
            "/localization/odom",
            10
        )

        self.get_logger().info("EKF Localization Started")

    def pose_callback(self, msg):

        odom = Odometry()

        odom.header = msg.header
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"

        odom.pose.pose = msg.pose

        self.odom_pub.publish(odom)

        self.counter += 1

        if self.counter % 30 == 0:
            self.get_logger().info("Publishing localization odometry")


def main(args=None):
    rclpy.init(args=args)

    node = EKFLocalization()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()