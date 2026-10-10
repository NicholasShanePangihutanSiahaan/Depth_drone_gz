#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Path
from nav_msgs.msg import Odometry

from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import Point

from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from rclpy.qos import HistoryPolicy


class LocalPlanner(Node):

    def __init__(self):

        super().__init__("local_planner")

        qos_sensor = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        ##################################################

        self.current_x = 0.0
        self.current_y = 0.0
        self.current_z = 0.0

        self.goal_x = None
        self.goal_y = None
        self.goal_z = 3.0

        self.tree_database = []

        ##################################################

        self.create_subscription(
            Point,
            "/navigation/mission_target",
            self.goal_callback,
            10
        )

        self.create_subscription(
            Odometry,
            "/localization/odom",
            self.odom_callback,
            qos_sensor
        )

        self.create_subscription(
            PoseArray,
            "/map/tree_locations",
            self.tree_callback,
            10
        )

        ##################################################

        self.path_pub = self.create_publisher(
            Path,
            "/navigation/local_path",
            10
        )

        ##################################################

        self.timer = self.create_timer(
            0.2,
            self.plan_path
        )

        self.get_logger().info(
            "Local Planner Started"
        )

    ##################################################

    def odom_callback(self, msg):

        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        self.current_z = msg.pose.pose.position.z

    ##################################################

    def goal_callback(self, msg):

        self.goal_x = msg.x
        self.goal_y = msg.y
        self.goal_z = msg.z

        self.get_logger().info(

            f"New Mission Target : ({self.goal_x:.2f}, {self.goal_y:.2f})"

        )

    ##################################################

    def tree_callback(self, msg):

        self.tree_database = []

        for pose in msg.poses:

            self.tree_database.append(

                (
                    pose.position.x,
                    pose.position.y
                )

            )

    ##################################################

    def plan_path(self):

        if self.goal_x is None:
            return

        path = Path()

        path.header.frame_id = "odom"
        path.header.stamp = self.get_clock().now().to_msg()

        ##################################################
        # Straight line interpolation
        ##################################################

        dx = self.goal_x - self.current_x
        dy = self.goal_y - self.current_y

        distance = math.hypot(dx, dy)

        if distance < 0.3:
            return

        step = 0.5

        n = max(2, int(distance / step))

        for i in range(n + 1):

            t = i / n

            x = self.current_x + dx * t
            y = self.current_y + dy * t

            ##################################################
            # Simple tree avoidance
            ##################################################

            offset_y = 0.0

            for tx, ty in self.tree_database:

                d = math.hypot(x - tx, y - ty)

                if d < 2.0:

                    sign = 1.0

                    if y > ty:
                        sign = -1.0

                    offset_y += sign * (2.0 - d) * 0.6

            pose = PoseStamped()

            pose.header = path.header

            pose.pose.position.x = x
            pose.pose.position.y = y + offset_y
            pose.pose.position.z = self.goal_z

            pose.pose.orientation.w = 1.0

            path.poses.append(pose)

        self.path_pub.publish(path)

        self.get_logger().info(

            f"Local Path Published ({len(path.poses)} points)"

        )


######################################################


def main(args=None):

    rclpy.init(args=args)

    node = LocalPlanner()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == "__main__":
    main()