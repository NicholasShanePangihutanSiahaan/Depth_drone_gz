#!/usr/bin/env python3

import rclpy

from rclpy.node import Node

from std_msgs.msg import String
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped


class MissionController(Node):

    def __init__(self):

        super().__init__("mission_controller")

        self.tree_map_ready = False
        self.global_path_ready = False
        self.navigation_started = False
        self.mission_finished = False

        self.total_waypoints = 0

        self.current_x = 0.0
        self.current_y = 0.0

        self.create_subscription(
            String,
            "/map/tree_locations",
            self.tree_callback,
            10
        )

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
            10
        )

        self.status_pub = self.create_publisher(
            String,
            "/mission/status",
            10
        )

        self.timer = self.create_timer(
            1.0,
            self.timer_callback
        )

        self.get_logger().info("Mission Controller Started")

    def tree_callback(self, msg):

        if not self.tree_map_ready:

            self.tree_map_ready = True

            self.get_logger().info(
                "Tree map received"
            )

    def path_callback(self, msg):

        self.global_path_ready = True

        self.total_waypoints = len(msg.poses)

    def pose_callback(self, msg):

        self.current_x = msg.pose.position.x
        self.current_y = msg.pose.position.y

    def timer_callback(self):

        status = String()

        if self.mission_finished:

            status.data = "MISSION_FINISHED"

            self.status_pub.publish(status)

            return

        if not self.tree_map_ready:

            status.data = "WAIT_TREE_MAP"

            self.status_pub.publish(status)

            return

        if not self.global_path_ready:

            status.data = "WAIT_GLOBAL_PATH"

            self.status_pub.publish(status)

            return

        if not self.navigation_started:

            self.navigation_started = True

            self.get_logger().info(
                "Mission Started"
            )

        status.data = (
            f"NAVIGATING | "
            f"Waypoints={self.total_waypoints} | "
            f"Drone=({self.current_x:.2f}, {self.current_y:.2f})"
        )

        self.status_pub.publish(status)


def main(args=None):

    rclpy.init(args=args)

    node = MissionController()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == "__main__":
    main()