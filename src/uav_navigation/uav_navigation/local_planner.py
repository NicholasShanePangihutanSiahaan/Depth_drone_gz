#!/usr/bin/env python3

import math

import rclpy

from rclpy.node import Node

from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PointStamped

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

        self.path = []

        self.current_index = 0

        self.drone_x = 0.0
        self.drone_y = 0.0
        self.drone_z = 0.0

        self.create_subscription(
            Path,
            "/navigation/global_path",
            self.path_callback,
            10
        )

        self.create_subscription(
            PoseStamped,
            "/mavros/local_position/pose",
            self.pose_callback,
            qos_sensor
        )

        self.target_pub = self.create_publisher(
            PointStamped,
            "/navigation/target_waypoint",
            10
        )

        self.timer = self.create_timer(
            0.2,
            self.publish_target
        )

        self.get_logger().info("Local Planner Started")

    def path_callback(self, msg):

        self.path = msg.poses
        self.current_index = 0

        self.get_logger().info(
            f"Received {len(self.path)} global waypoints"
        )

    def pose_callback(self, msg):

        self.drone_x = msg.pose.position.x
        self.drone_y = msg.pose.position.y
        self.drone_z = msg.pose.position.z

    def publish_target(self):

        if len(self.path) == 0:
            return

        if self.current_index >= len(self.path):
            self.get_logger().info("Mission Finished")
            return

        target_pose = self.path[self.current_index]

        tx = target_pose.pose.position.x
        ty = target_pose.pose.position.y
        tz = target_pose.pose.position.z

        distance = math.sqrt(
            (tx-self.drone_x)**2 +
            (ty-self.drone_y)**2
        )

        if distance < 1.5:

            self.current_index += 1

            if self.current_index >= len(self.path):

                self.get_logger().info("Reached Final Waypoint")
                return

            target_pose = self.path[self.current_index]

            tx = target_pose.pose.position.x
            ty = target_pose.pose.position.y
            tz = target_pose.pose.position.z

        target = PointStamped()

        target.header.frame_id = "odom"

        target.point.x = tx
        target.point.y = ty
        target.point.z = tz

        self.target_pub.publish(target)

        self.get_logger().info(
            f"Target WP {self.current_index+1}/{len(self.path)} : "
            f"({tx:.2f}, {ty:.2f}, {tz:.2f})"
        )


def main(args=None):

    rclpy.init(args=args)

    node = LocalPlanner()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == "__main__":
    main()