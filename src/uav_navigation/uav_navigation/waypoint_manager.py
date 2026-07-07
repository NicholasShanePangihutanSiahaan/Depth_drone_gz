#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node

from visualization_msgs.msg import MarkerArray

from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseArray


class WaypointManager(Node):

    def __init__(self):

        super().__init__("waypoint_manager")

        self.create_subscription(
            MarkerArray,
            "/tree_markers",
            self.tree_callback,
            10
        )

        self.pub = self.create_publisher(
            PoseArray,
            "/navigation/waypoints",
            10
        )

        self.get_logger().info("Waypoint Manager Started")

    def tree_callback(self, msg):

        if len(msg.markers) == 0:
            return

        trees = []

        for marker in msg.markers:

            x = marker.pose.position.x
            y = marker.pose.position.y

            trees.append((x, y))

        #
        # Urutkan dari kiri ke kanan
        #

        trees.sort(key=lambda p: (p[0], p[1]))

        pose_array = PoseArray()

        pose_array.header.frame_id = "odom"
        pose_array.header.stamp = self.get_clock().now().to_msg()

        for x, y in trees:

            pose = Pose()

            #
            # waypoint dibuat 2 meter sebelum pohon
            #

            pose.position.x = x - 2.0
            pose.position.y = y
            pose.position.z = 3.0

            pose.orientation.w = 1.0

            pose_array.poses.append(pose)

        self.pub.publish(pose_array)

        self.get_logger().info(
            f"Published {len(pose_array.poses)} waypoints"
        )


def main(args=None):

    rclpy.init(args=args)

    node = WaypointManager()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == "__main__":
    main()