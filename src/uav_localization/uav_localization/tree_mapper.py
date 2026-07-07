#!/usr/bin/env python3

import math

import rclpy

from rclpy.node import Node

from geometry_msgs.msg import Point

from visualization_msgs.msg import Marker


class TreeMapper(Node):

    def __init__(self):

        super().__init__("tree_mapper")

        self.tree_list=[]

        self.create_subscription(
            Point,
            "/perception/tree_target",
            self.callback,
            10
        )

        self.marker_pub=self.create_publisher(
            Marker,
            "/tree_markers",
            10
        )

    def callback(self,msg):

        for p in self.tree_list:

            if math.hypot(msg.x-p[0],msg.y-p[1])<1.5:

                return

        self.tree_list.append((msg.x,msg.y))

        marker=Marker()

        marker.header.frame_id="odom"

        marker.type=Marker.SPHERE_LIST

        marker.action=Marker.ADD

        marker.scale.x=0.5
        marker.scale.y=0.5
        marker.scale.z=0.5

        marker.color.g=1.0
        marker.color.a=1.0

        for x,y in self.tree_list:

            pt=Point()

            pt.x=x
            pt.y=y

            marker.points.append(pt)

        self.marker_pub.publish(marker)

        self.get_logger().info(
            f"Total tree : {len(self.tree_list)}"
        )


def main():

    rclpy.init()

    node=TreeMapper()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__=="__main__":
    main()