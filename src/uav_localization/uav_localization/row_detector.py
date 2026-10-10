#!/usr/bin/env python3

import math
import numpy as np

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point

from nav_msgs.msg import Odometry

from visualization_msgs.msg import Marker


class RowDetector(Node):

    def __init__(self):

        super().__init__("row_detector")

        self.tree_list = []

        self.drone_x = 0.0
        self.drone_y = 0.0

        self.search_radius = 15.0

        self.create_subscription(
            PoseArray,
            "/map/tree_locations",
            self.tree_callback,
            10
        )

        self.create_subscription(
            Odometry,
            "/localization/odom",
            self.odom_callback,
            10
        )

        self.row_marker_pub = self.create_publisher(
            Marker,
            "/row_center",
            10
        )

        self.row_pose_pub = self.create_publisher(
            PoseArray,
            "/navigation/current_row",
            10
        )

        self.create_timer(
            0.5,
            self.process_row
        )

        self.get_logger().info("Row Detector Started")

    ############################################################

    def tree_callback(self,msg):

        self.tree_list=[]

        for pose in msg.poses:

            self.tree_list.append(
                (
                    pose.position.x,
                    pose.position.y
                )
            )

    ############################################################

    def odom_callback(self,msg):

        self.drone_x=msg.pose.pose.position.x
        self.drone_y=msg.pose.pose.position.y

    ############################################################

    def process_row(self):

        if len(self.tree_list)<3:
            return

        nearby=[]

        for tx,ty in self.tree_list:

            d=math.hypot(
                tx-self.drone_x,
                ty-self.drone_y
            )

            if d<self.search_radius:
                nearby.append((tx,ty))

        if len(nearby)<3:
            return

        ys=np.array([p[1] for p in nearby])

        center_y=float(np.mean(ys))

        xs=np.array([p[0] for p in nearby])

        xmin=float(np.min(xs))-3.0
        xmax=float(np.max(xs))+3.0

        ########################################################

        marker=Marker()

        marker.header.frame_id="odom"
        marker.header.stamp=self.get_clock().now().to_msg()

        marker.ns="row_center"
        marker.id=0

        marker.type=Marker.LINE_STRIP
        marker.action=Marker.ADD

        marker.scale.x=0.20

        marker.color.r=1.0
        marker.color.g=0.0
        marker.color.b=0.0
        marker.color.a=1.0

        ########################################################

        path=PoseArray()

        path.header.frame_id="odom"
        path.header.stamp=self.get_clock().now().to_msg()

        for x in np.linspace(xmin,xmax,30):

            p=Point()

            p.x=float(x)
            p.y=center_y
            p.z=0.0

            marker.points.append(p)

            pose=Pose()

            pose.position.x=float(x)
            pose.position.y=center_y
            pose.position.z=0.0

            pose.orientation.w=1.0

            path.poses.append(pose)

        self.row_marker_pub.publish(marker)

        self.row_pose_pub.publish(path)

    ############################################################


def main(args=None):

    rclpy.init(args=args)

    node=RowDetector()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__=="__main__":
    main()