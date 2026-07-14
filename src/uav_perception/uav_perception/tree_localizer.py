#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Point


class TreeLocalizer(Node):

    def __init__(self):

        super().__init__("tree_localizer")

        ##########################################
        # Camera Intrinsic
        ##########################################

        self.fx = 381.3611502479812
        self.fy = 381.3611502479812

        self.cx = 320.0
        self.cy = 240.0

        ##########################################

        self.create_subscription(
            Point,
            "/perception/tree_pixel",
            self.pixel_callback,
            10
        )

        self.pub = self.create_publisher(
            Point,
            "/perception/tree_position_camera",
            10
        )

        self.get_logger().info(
            "Tree Localizer Started"
        )

    ##################################################

    def pixel_callback(self, msg):

        u = msg.x
        v = msg.y
        Z = msg.z

        ##################################################
        # Invalid depth
        ##################################################

        if Z <= 0.0:
            return

        ##################################################
        # Camera coordinate
        ##################################################

        X = (u - self.cx) * Z / self.fx
        Y = (v - self.cy) * Z / self.fy

        ##################################################

        point = Point()

        point.x = X
        point.y = Y
        point.z = Z

        self.pub.publish(point)

        ##################################################

        self.get_logger().info(

            f"Tree Camera Position : "
            f"({X:.2f}, {Y:.2f}, {Z:.2f})"

        )


##########################################################


def main(args=None):

    rclpy.init(args=args)

    node = TreeLocalizer()

    try:

        rclpy.spin(node)

    except KeyboardInterrupt:

        pass

    node.destroy_node()

    rclpy.shutdown()


if __name__ == "__main__":

    main()