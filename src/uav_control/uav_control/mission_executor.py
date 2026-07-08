#!/usr/bin/env python3

import math

import rclpy

from rclpy.node import Node

from std_msgs.msg import Bool
from geometry_msgs.msg import Point
from geometry_msgs.msg import PoseStamped


class MissionExecutor(Node):

    def __init__(self):

        super().__init__("mission_executor")

        self.current_pose = None
        self.current_goal = None

        self.state = "WAITING"

        self.goal_tolerance = 1.0

        self.pose_sub = self.create_subscription(
            PoseStamped,
            "/mavros/local_position/pose",
            self.pose_callback,
            10
        )

        self.goal_sub = self.create_subscription(
            Point,
            "/navigation/target_point",
            self.goal_callback,
            10
        )

        self.goal_pub = self.create_publisher(
            Point,
            "/mission/current_goal",
            10
        )

        self.land_pub = self.create_publisher(
            Bool,
            "/mission/land",
            10
        )

        self.timer = self.create_timer(
            0.2,
            self.control_loop
        )

        self.get_logger().info("Mission Executor Started")

    def pose_callback(self, msg):

        self.current_pose = msg.pose

    def goal_callback(self, msg):

        self.current_goal = msg

        if self.state == "WAITING":

            self.state = "NAVIGATING"

            self.get_logger().info("Mission Started")

    def distance_to_goal(self):

        if self.current_pose is None:
            return None

        if self.current_goal is None:
            return None

        dx = self.current_goal.x - self.current_pose.position.x
        dy = self.current_goal.y - self.current_pose.position.y

        return math.sqrt(dx * dx + dy * dy)

    def control_loop(self):

        if self.state == "WAITING":
            return

        if self.state == "NAVIGATING":

            self.goal_pub.publish(self.current_goal)

            d = self.distance_to_goal()

            if d is None:
                return

            if d < self.goal_tolerance:

                self.get_logger().info("Waypoint reached")

                self.state = "LAND"

        elif self.state == "LAND":

            msg = Bool()

            msg.data = True

            self.land_pub.publish(msg)

            self.get_logger().info("Mission Complete")

            self.state = "FINISHED"

        elif self.state == "FINISHED":

            pass


def main(args=None):

    rclpy.init(args=args)

    node = MissionExecutor()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == "__main__":

    main()