#!/usr/bin/env python3

import math

import rclpy

from rclpy.node import Node

from std_msgs.msg import String

from nav_msgs.msg import Path

from geometry_msgs.msg import PoseStamped

from visualization_msgs.msg import Marker

from geometry_msgs.msg import Point


class GlobalPlanner(Node):

    def __init__(self):

        super().__init__("global_planner")

        self.create_subscription(
            String,
            "/map/tree_locations",
            self.tree_callback,
            10
        )

        self.path_pub = self.create_publisher(
            Path,
            "/navigation/global_path",
            10
        )

        self.marker_pub = self.create_publisher(
            Marker,
            "/navigation/global_path_marker",
            10
        )

        self.get_logger().info("Global Planner Started")

    def tree_callback(self, msg):

        trees = []

        lines = msg.data.strip().split("\n")

        for line in lines:

            if line == "":
                continue

            data = line.split(",")

            if len(data) != 3:
                continue

            _, x, y = data

            trees.append((float(x), float(y)))

        if len(trees) < 2:
            return

        trees.sort(key=lambda p: p[0])

        path = Path()

        path.header.frame_id = "odom"

        marker = Marker()

        marker.header.frame_id = "odom"

        marker.type = Marker.LINE_STRIP

        marker.action = Marker.ADD

        marker.scale.x = 0.25

        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 1.0
        marker.color.a = 1.0

        for i in range(len(trees)-1):

            x1, y1 = trees[i]
            x2, y2 = trees[i+1]

            mx = (x1 + x2) / 2.0
            my = (y1 + y2) / 2.0

            pose = PoseStamped()

            pose.header.frame_id = "odom"

            pose.pose.position.x = mx
            pose.pose.position.y = my
            pose.pose.position.z = 3.0

            dx = x2 - x1
            dy = y2 - y1

            yaw = math.atan2(dy, dx)

            pose.pose.orientation.z = math.sin(yaw / 2.0)
            pose.pose.orientation.w = math.cos(yaw / 2.0)

            path.poses.append(pose)

            p = Point()

            p.x = mx
            p.y = my
            p.z = 3.0

            marker.points.append(p)

        self.path_pub.publish(path)

        self.marker_pub.publish(marker)

        self.get_logger().info(
            f"Published Global Path ({len(path.poses)} waypoints)"
        )


def main(args=None):

    rclpy.init(args=args)

    node = GlobalPlanner()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == "__main__":
    main()