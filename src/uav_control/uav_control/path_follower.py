#!/usr/bin/env python3

import math

import rclpy

from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Point

from nav_msgs.msg import Path


class PathFollower(Node):

    def __init__(self):

        super().__init__("path_follower")

        self.current_pose = None

        self.path = []

        self.current_index = 0

        self.goal_threshold = 0.8

        self.create_subscription(
            PoseStamped,
            "/mavros/local_position/pose",
            self.pose_callback,
            10
        )

        self.create_subscription(
            Path,
            "/trajectory/path",
            self.path_callback,
            10
        )

        self.target_pub = self.create_publisher(
            Point,
            "/control/target_point",
            10
        )

        self.timer = self.create_timer(
            0.1,
            self.follow_path
        )

        self.get_logger().info("Path Follower Started")

    ############################################################

    def pose_callback(self, msg):

        self.current_pose = msg.pose

    ############################################################

    def path_callback(self, msg):

        self.path = msg.poses

        self.current_index = 0

        self.get_logger().info(
            f"Received Path ({len(self.path)} waypoints)"
        )

    ############################################################

    def follow_path(self):

        if self.current_pose is None:

            return

        if len(self.path) == 0:

            return

        if self.current_index >= len(self.path):

            self.get_logger().info("Mission Complete")

            return

        target = self.path[self.current_index].pose.position

        dx = target.x - self.current_pose.position.x
        dy = target.y - self.current_pose.position.y
        dz = target.z - self.current_pose.position.z

        distance = math.sqrt(dx*dx + dy*dy + dz*dz)

        if distance < self.goal_threshold:

            self.current_index += 1

            if self.current_index >= len(self.path):

                self.get_logger().info("Last waypoint reached")

                return

            target = self.path[self.current_index].pose.position

        msg = Point()

        msg.x = target.x
        msg.y = target.y
        msg.z = target.z

        self.target_pub.publish(msg)


############################################################

def main(args=None):

    rclpy.init(args=args)

    node = PathFollower()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == "__main__":
    main()