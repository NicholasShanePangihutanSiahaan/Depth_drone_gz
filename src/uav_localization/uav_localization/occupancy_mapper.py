#!/usr/bin/env python3

import numpy as np

import rclpy

from rclpy.node import Node

from visualization_msgs.msg import Marker

from nav_msgs.msg import OccupancyGrid


class OccupancyMapper(Node):

    def __init__(self):

        super().__init__("occupancy_mapper")

        self.grid=np.zeros((200,200),dtype=np.int8)

        self.pub=self.create_publisher(
            OccupancyGrid,
            "/occupancy_map",
            10
        )

        self.create_subscription(
            Marker,
            "/tree_markers",
            self.callback,
            10
        )

    def callback(self,msg):

        self.grid.fill(0)

        for p in msg.points:

            ix=int(p.x+100)

            iy=int(p.y+100)

            if 0<=ix<200 and 0<=iy<200:

                self.grid[iy,ix]=100

        grid=OccupancyGrid()

        grid.header.frame_id="odom"

        grid.info.width=200

        grid.info.height=200

        grid.info.resolution=1.0

        grid.data=self.grid.flatten().tolist()

        self.pub.publish(grid)


def main():

    rclpy.init()

    node=OccupancyMapper()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__=="__main__":
    main()