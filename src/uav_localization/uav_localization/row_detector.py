#!/usr/bin/env python3

import numpy as np

import rclpy

from rclpy.node import Node

from visualization_msgs.msg import Marker

from geometry_msgs.msg import Point


class RowDetector(Node):

    def __init__(self):

        super().__init__("row_detector")

        self.create_subscription(
            Marker,
            "/tree_markers",
            self.callback,
            10
        )

        self.pub=self.create_publisher(
            Marker,
            "/row_center",
            10
        )

    def callback(self,msg):

        if len(msg.points)<5:
            return

        ys=[p.y for p in msg.points]

        center=np.mean(ys)

        line=Marker()

        line.header.frame_id="odom"

        line.type=Marker.LINE_STRIP

        line.scale.x=0.2

        line.color.r=1.0

        line.color.a=1.0

        for x in np.linspace(-100,100,30):

            p=Point()

            p.x=float(x)

            p.y=float(center)

            line.points.append(p)

        self.pub.publish(line)


def main():

    rclpy.init()

    node=RowDetector()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__=="__main__":
    main()