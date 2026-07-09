#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import PoseStamped

from nav_msgs.msg import Path

from visualization_msgs.msg import Marker

from geometry_msgs.msg import Point


class GlobalPlanner(Node):

    def __init__(self):

        super().__init__("global_planner")

        self.tree_map = []

        self.last_tree_count = 0

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

        self.create_subscription(
            PoseArray,
            "/map/tree_locations",
            self.tree_callback,
            10
        )

        self.get_logger().info(
            "Global Planner Started"
        )

    ########################################################

    def tree_callback(self,msg):

        if len(msg.poses)==self.last_tree_count:
            return

        self.last_tree_count=len(msg.poses)

        self.tree_map=[]

        for pose in msg.poses:

            self.tree_map.append(

                (
                    pose.position.x,
                    pose.position.y
                )

            )

        if len(self.tree_map)<2:
            return

        self.generate_path()

    ########################################################

    def generate_path(self):

        #
        # Sort sepanjang arah X
        #

        trees=sorted(
            self.tree_map,
            key=lambda p:p[0]
        )

        path=Path()

        path.header.frame_id="odom"

        path.header.stamp=self.get_clock().now().to_msg()

        marker=Marker()

        marker.header=path.header

        marker.ns="global_path"

        marker.id=0

        marker.type=Marker.LINE_STRIP

        marker.action=Marker.ADD

        marker.scale.x=0.15

        marker.color.r=0.0
        marker.color.g=0.0
        marker.color.b=1.0
        marker.color.a=1.0

        waypoint_index=0

        for tree in trees:

            pose=PoseStamped()

            pose.header=path.header

            pose.pose.position.x=tree[0]

            pose.pose.position.y=tree[1]

            pose.pose.position.z=3.0

            pose.pose.orientation.w=1.0

            path.poses.append(pose)

            p=Point()

            p.x=tree[0]

            p.y=tree[1]

            p.z=3.0

            marker.points.append(p)

            waypoint_index+=1

        self.path_pub.publish(path)

        self.marker_pub.publish(marker)

        self.get_logger().info(

            f"Published {waypoint_index} global waypoints"

        )


#############################################################

def main(args=None):

    rclpy.init(args=args)

    node=GlobalPlanner()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__=="__main__":
    main()