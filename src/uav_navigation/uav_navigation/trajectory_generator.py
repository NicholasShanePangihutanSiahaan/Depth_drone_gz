#!/usr/bin/env python3

import math

import rclpy

from rclpy.node import Node

from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped


class TrajectoryGenerator(Node):

    def __init__(self):

        super().__init__("trajectory_generator")

        self.current_target = None

        self.create_subscription(
            PointStamped,
            "/navigation/target_waypoint",
            self.target_callback,
            10
        )

        self.trajectory_pub = self.create_publisher(
            Path,
            "/navigation/trajectory",
            10
        )

        self.get_logger().info("Trajectory Generator Started")

    def target_callback(self, msg):

        self.current_target = msg

        self.generate_trajectory()

    def generate_trajectory(self):

        if self.current_target is None:
            return

        tx = self.current_target.point.x
        ty = self.current_target.point.y
        tz = self.current_target.point.z

        path = Path()

        path.header.frame_id = "odom"

        N = 20

        for i in range(N + 1):

            s = i / N

            pose = PoseStamped()

            pose.header.frame_id = "odom"

            pose.pose.position.x = tx * s
            pose.pose.position.y = ty * s
            pose.pose.position.z = tz

            yaw = math.atan2(ty, tx)

            pose.pose.orientation.z = math.sin(yaw / 2.0)
            pose.pose.orientation.w = math.cos(yaw / 2.0)

            path.poses.append(pose)

        self.trajectory_pub.publish(path)

        self.get_logger().info(
            f"Trajectory Generated ({len(path.poses)} points)"
        )


def main(args=None):

    rclpy.init(args=args)

    node = TrajectoryGenerator()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == "__main__":
    main()