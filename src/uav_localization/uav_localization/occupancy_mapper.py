#!/usr/bin/env python3

import numpy as np

import rclpy
from rclpy.node import Node

from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import MapMetaData
from geometry_msgs.msg import Pose
from visualization_msgs.msg import MarkerArray


class OccupancyMapper(Node):

    def __init__(self):

        super().__init__("occupancy_mapper")

        self.width = 200
        self.height = 200
        self.resolution = 0.5     # 0.5 meter / cell

        self.origin_x = -50.0
        self.origin_y = -50.0

        self.grid = np.zeros(
            (self.height, self.width),
            dtype=np.int8
        )

        self.map_pub = self.create_publisher(
            OccupancyGrid,
            "/occupancy_map",
            10
        )

        self.create_subscription(
            MarkerArray,
            "/tree_markers",
            self.marker_callback,
            10
        )

        self.get_logger().info("Occupancy Mapper Started")

    def marker_callback(self, msg):

        self.grid.fill(0)

        for marker in msg.markers:

            x = marker.pose.position.x
            y = marker.pose.position.y

            ix = int((x - self.origin_x) / self.resolution)
            iy = int((y - self.origin_y) / self.resolution)

            if 0 <= ix < self.width and 0 <= iy < self.height:

                self.grid[iy][ix] = 100

        occ = OccupancyGrid()

        occ.header.stamp = self.get_clock().now().to_msg()
        occ.header.frame_id = "odom"

        info = MapMetaData()

        info.resolution = self.resolution
        info.width = self.width
        info.height = self.height

        origin = Pose()

        origin.position.x = self.origin_x
        origin.position.y = self.origin_y
        origin.position.z = 0.0

        origin.orientation.w = 1.0

        info.origin = origin

        occ.info = info

        occ.data = self.grid.flatten().tolist()

        self.map_pub.publish(occ)

        self.get_logger().info(
            f"Occupancy map updated ({len(msg.markers)} trees)"
        )


def main(args=None):

    rclpy.init(args=args)

    node = OccupancyMapper()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == "__main__":
    main()