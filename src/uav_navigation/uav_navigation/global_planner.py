#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Point

from nav_msgs.msg import Path

from visualization_msgs.msg import Marker


class GlobalPlanner(Node):

    def __init__(self):

        super().__init__("global_planner")

        self.trees=[]

        self.create_subscription(
            PoseArray,
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


        self.get_logger().info(
            "Global Planner Started"
        )


    def tree_callback(self,msg):

        self.trees=[]

        for pose in msg.poses:

            self.trees.append(
                (
                    pose.position.x,
                    pose.position.y
                )
            )


        self.get_logger().info(
            f"Received {len(self.trees)} trees"
        )


        if len(self.trees) < 2:
            return


        self.generate_path()



    def generate_path(self):

        # urutkan berdasarkan posisi x
        self.trees.sort(
            key=lambda p:p[0]
        )


        path = Path()

        path.header.frame_id="odom"


        marker = Marker()

        marker.header.frame_id="odom"

        marker.type=Marker.LINE_STRIP

        marker.action=Marker.ADD

        marker.scale.x=0.2

        marker.color.b=1.0
        marker.color.a=1.0



        for i in range(len(self.trees)-1):

            x1,y1=self.trees[i]

            x2,y2=self.trees[i+1]


            # titik tengah antar pohon
            mx=(x1+x2)/2
            my=(y1+y2)/2



            pose=PoseStamped()

            pose.header.frame_id="odom"

            pose.pose.position.x=mx

            pose.pose.position.y=my

            pose.pose.position.z=3.0



            yaw=math.atan2(
                y2-y1,
                x2-x1
            )


            pose.pose.orientation.z=math.sin(yaw/2)

            pose.pose.orientation.w=math.cos(yaw/2)



            path.poses.append(
                pose
            )


            point=Point()

            point.x=mx

            point.y=my

            point.z=3.0


            marker.points.append(
                point
            )



        self.path_pub.publish(
            path
        )

        self.marker_pub.publish(
            marker
        )


        self.get_logger().info(
            f"Published Global Path {len(path.poses)} waypoints"
        )



def main(args=None):

    rclpy.init(args=args)

    node=GlobalPlanner()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()



if __name__=="__main__":
    main()