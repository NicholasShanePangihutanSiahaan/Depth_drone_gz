#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry

from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    HistoryPolicy
)


class LocalizationBridge(Node):

    def __init__(self):
        super().__init__("localization_bridge")

        qos_sensor = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.counter = 0

        # ====================================================
        # INPUT
        # Pose hasil estimator autopilot
        # (EKF PX4 / ArduPilot + VIO)
        # ====================================================
        self.pose_sub = self.create_subscription(
            PoseStamped,
            "/mavros/local_position/pose",
            self.pose_callback,
            qos_sensor
        )

        # ====================================================
        # OUTPUT
        # Format standar ROS
        # ====================================================
        self.odom_pub = self.create_publisher(
            Odometry,
            "/localization/odom",
            10
        )

        self.get_logger().info("Localization Bridge Started")

    def pose_callback(self, msg):

        odom = Odometry()

        odom.header.stamp = msg.header.stamp
        odom.header.frame_id = "odom"

        odom.child_frame_id = "base_link"

        odom.pose.pose = msg.pose

        # ----------------------------------------------------
        # Tidak memiliki data velocity
        # Kosongkan covariance agar node lain tahu
        # bahwa informasi ini belum tersedia
        # ----------------------------------------------------
        odom.pose.covariance = [0.0] * 36
        odom.twist.covariance = [-1.0] * 36

        self.odom_pub.publish(odom)

        self.counter += 1

        if self.counter % 30 == 0:
            self.get_logger().info(
                f"Localization: "
                f"x={msg.pose.position.x:.2f} "
                f"y={msg.pose.position.y:.2f} "
                f"z={msg.pose.position.z:.2f}"
            )


def main(args=None):

    rclpy.init(args=args)

    node = LocalizationBridge()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()