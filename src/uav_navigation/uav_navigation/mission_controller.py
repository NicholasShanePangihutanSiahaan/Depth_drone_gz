#!/usr/bin/env python3

import rclpy

from rclpy.node import Node

from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    HistoryPolicy
)

from std_msgs.msg import String

from nav_msgs.msg import Path

from geometry_msgs.msg import (
    PoseStamped,
    PoseArray,
    Point
)


class MissionController(Node):

    def __init__(self):

        super().__init__("mission_controller")

        qos_sensor = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        ##################################################
        # Mission State
        ##################################################

        self.tree_map_ready = False
        self.global_path_ready = False
        self.navigation_started = False
        self.mission_finished = False

        ##################################################
        # Data
        ##################################################

        self.tree_count = 0

        self.path = []

        self.total_waypoints = 0

        self.current_waypoint = 0

        self.current_x = 0.0
        self.current_y = 0.0
        self.current_z = 0.0

        ##################################################
        # Subscribers
        ##################################################

        self.create_subscription(
            PoseArray,
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
            qos_sensor
        )

        ##################################################
        # Publishers
        ##################################################

        self.status_pub = self.create_publisher(
            String,
            "/mission/status",
            10
        )

        self.target_pub = self.create_publisher(
            Point,
            "/navigation/target_point",
            10
        )

        ##################################################

        self.timer = self.create_timer(
            0.5,
            self.timer_callback
        )

        self.get_logger().info(
            "Mission Controller Started"
        )

    ###########################################################

    def tree_callback(self, msg):

        self.tree_map_ready = True

        self.tree_count = len(msg.poses)

        self.get_logger().info(
            f"Tree map received ({self.tree_count} trees)"
        )

    ###########################################################

    def path_callback(self, msg):

        self.global_path_ready = True

        self.path = msg.poses

        self.total_waypoints = len(self.path)

        self.get_logger().info(
            f"Global path received ({self.total_waypoints} waypoints)"
        )

    ###########################################################

    def pose_callback(self, msg):

        self.current_x = msg.pose.position.x
        self.current_y = msg.pose.position.y
        self.current_z = msg.pose.position.z

    ###########################################################

    def timer_callback(self):

        status = String()

        ###############################################

        if not self.tree_map_ready:

            status.data = "WAIT_TREE_MAP"

            self.status_pub.publish(status)

            return

        ###############################################

        if not self.global_path_ready:

            status.data = "WAIT_GLOBAL_PATH"

            self.status_pub.publish(status)

            return

        ###############################################

        if self.total_waypoints == 0:

            status.data = "NO_WAYPOINT"

            self.status_pub.publish(status)

            return

        ###############################################

        if not self.navigation_started:

            self.navigation_started = True

            self.get_logger().info(
                "MISSION STARTED"
            )

        ###############################################
        # Publish Current Target
        ###############################################

        if self.current_waypoint >= self.total_waypoints:

            self.current_waypoint = self.total_waypoints - 1

        pose = self.path[self.current_waypoint]

        target = Point()

        target.x = pose.pose.position.x
        target.y = pose.pose.position.y
        target.z = pose.pose.position.z

        self.target_pub.publish(target)

        ###############################################
        # Publish Status
        ###############################################

        status.data = (

            f"NAVIGATING | "
            f"Trees={self.tree_count} | "
            f"Waypoint={self.current_waypoint + 1}/{self.total_waypoints} | "
            f"Target=({target.x:.2f},{target.y:.2f}) | "
            f"UAV=({self.current_x:.2f},{self.current_y:.2f},{self.current_z:.2f})"

        )

        self.status_pub.publish(status)

    ###########################################################


def main(args=None):

    rclpy.init(args=args)

    node = MissionController()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == "__main__":

    main()