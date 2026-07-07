#!/usr/bin/env python3

import numpy as np

import rclpy
from rclpy.node import Node

from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import Point


class RowDetector(Node):

    def __init__(self):

        super().__init__("row_detector")

        self.create_subscription(
            MarkerArray,
            "/tree_markers",
            self.marker_callback,
            10
        )

        self.row_pub = self.create_publisher(
            Marker,
            "/row_center",
            10
        )

        self.get_logger().info("Row Detector Started")

    def marker_callback(self, msg):

        if len(msg.markers) < 5:
            return

        xs = []
        ys = []

        for marker in msg.markers:

            xs.append(marker.pose.position.x)
            ys.append(marker.pose.position.y)

        xs = np.array(xs)
        ys = np.array(ys)

        center_y = float(np.mean(ys))

        line = Marker()

        line.header.frame_id = "odom"
        line.header.stamp = self.get_clock().now().to_msg()

        line.ns = "row"

        line.id = 0

        line.type = Marker.LINE_STRIP
        line.action = Marker.ADD

        line.scale.x = 0.25

        line.color.r = 1.0
        line.color.g = 0.0
        line.color.b = 0.0
        line.color.a = 1.0

        xmin = np.min(xs) - 5.0
        xmax = np.max(xs) + 5.0

        for x in np.linspace(xmin, xmax, 40):

            p = Point()

            p.x = float(x)
            p.y = center_y
            p.z = 0.0

            line.points.append(p)

        self.row_pub.publish(line)

        self.get_logger().info(
            f"Detected row center at y = {center_y:.2f}"
        )


def main(args=None):

    rclpy.init(args=args)

    node = RowDetector()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == "__main__":
    main()