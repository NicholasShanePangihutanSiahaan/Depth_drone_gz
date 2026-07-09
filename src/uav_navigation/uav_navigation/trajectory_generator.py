#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import PoseStamped

from nav_msgs.msg import Path
from nav_msgs.msg import Odometry

from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    HistoryPolicy
)


class TrajectoryGenerator(Node):

    def __init__(self):

        super().__init__("trajectory_generator")

        qos_sensor = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.current_target = None

        self.current_x = 0.0
        self.current_y = 0.0
        self.current_z = 3.0

        self.have_pose = False

        self.create_subscription(
            PointStamped,
            "/navigation/target_waypoint",
            self.target_callback,
            10
        )

        self.create_subscription(
            Odometry,
            "/localization/odom",
            self.odom_callback,
            qos_sensor
        )

        self.trajectory_pub = self.create_publisher(
            Path,
            "/navigation/trajectory",
            10
        )

        self.get_logger().info("Trajectory Generator Started")

    ############################################################

    def odom_callback(self, msg):

        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        self.current_z = msg.pose.pose.position.z

        self.have_pose = True

    ############################################################

    def target_callback(self, msg):

        if not self.have_pose:
            return

        self.current_target = msg

        self.generate_trajectory()

    ############################################################

    def generate_trajectory(self):

        tx = self.current_target.point.x
        ty = self.current_target.point.y
        tz = self.current_target.point.z

        sx = self.current_x
        sy = self.current_y
        sz = self.current_z

        path = Path()

        path.header.frame_id = "odom"
        path.header.stamp = self.get_clock().now().to_msg()

        distance = math.hypot(tx - sx, ty - sy)

        step = 0.30

        N = max(5, int(distance / step))

        yaw = math.atan2(
            ty - sy,
            tx - sx
        )

        qz = math.sin(yaw / 2.0)
        qw = math.cos(yaw / 2.0)

        for i in range(N + 1):

            s = i / float(N)

            pose = PoseStamped()

            pose.header.frame_id = "odom"

            pose.pose.position.x = sx + (tx - sx) * s
            pose.pose.position.y = sy + (ty - sy) * s
            pose.pose.position.z = sz + (tz - sz) * s

            pose.pose.orientation.z = qz
            pose.pose.orientation.w = qw

            path.poses.append(pose)

        self.trajectory_pub.publish(path)

        self.get_logger().info(
            f"Generated trajectory : {len(path.poses)} points"
        )


############################################################


def main(args=None):

    rclpy.init(args=args)

    node = TrajectoryGenerator()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == "__main__":
    main()