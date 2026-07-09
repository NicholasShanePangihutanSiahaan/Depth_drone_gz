#!/usr/bin/env python3

import math
import time

import rclpy
from rclpy.node import Node


from geometry_msgs.msg import Point

from uav_interfaces.msg import Tree
from uav_interfaces.msg import TreeArray

from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray



class TreeMapper(Node):

    def __init__(self):

        super().__init__(
            "tree_mapper"
        )


        ##################################################
        # Parameter
        ##################################################

        self.merge_distance = 1.5

        self.max_confidence = 1.0

        self.confidence_increment = 0.15

        self.confidence_decay = 0.01



        ##################################################
        # Database
        ##################################################

        self.tree_database = []

        self.next_tree_id = 1



        ##################################################
        # Subscriber
        ##################################################

        self.create_subscription(
            Point,
            "/perception/tree_target",
            self.tree_callback,
            10
        )


        ##################################################
        # Publisher
        ##################################################

        self.tree_pub = self.create_publisher(
            TreeArray,
            "/map/trees",
            10
        )


        self.marker_pub = self.create_publisher(
            MarkerArray,
            "/tree_markers",
            10
        )


        ##################################################

        self.timer = self.create_timer(
            5.0,
            self.update_confidence
        )


        self.get_logger().info(
            "Tree Mapper Started"
        )



    ##################################################
    # Detection callback
    ##################################################

    def tree_callback(self,msg):


        x = msg.x
        y = msg.y
        z = msg.z


        nearest = None

        min_distance = float("inf")



        for tree in self.tree_database:


            d = math.sqrt(

                (x-tree["x"])**2 +

                (y-tree["y"])**2

            )


            if d < min_distance:

                min_distance = d

                nearest = tree



        ##################################################
        # Existing tree
        ##################################################

        if nearest and min_distance < self.merge_distance:


            nearest["count"] += 1


            alpha = 1.0 / nearest["count"]


            nearest["x"] += alpha * (
                x-nearest["x"]
            )


            nearest["y"] += alpha * (
                y-nearest["y"]
            )


            nearest["z"] += alpha * (
                z-nearest["z"]
            )


            nearest["confidence"] = min(

                nearest["confidence"]
                +
                self.confidence_increment,

                self.max_confidence

            )


            nearest["last_seen"] = time.time()



        ##################################################
        # New tree
        ##################################################

        else:


            tree = {


                "id":
                self.next_tree_id,


                "x":x,

                "y":y,

                "z":z,


                "count":1,


                "confidence":0.2,


                "inspected":False,


                "last_seen":
                time.time()

            }



            self.tree_database.append(tree)


            self.next_tree_id +=1



            self.get_logger().info(

                f"New Tree {tree['id']} "
                f"({x:.2f},{y:.2f},{z:.2f})"

            )



        self.publish_tree()

        self.publish_marker()



    ##################################################
    # Confidence aging
    ##################################################

    def update_confidence(self):


        now=time.time()


        for tree in self.tree_database:


            dt = now-tree["last_seen"]


            if dt > 30:


                tree["confidence"] -= (
                    self.confidence_decay
                )



                if tree["confidence"] <0:

                    tree["confidence"]=0



        self.publish_tree()



    ##################################################
    # Publish TreeArray
    ##################################################

    def publish_tree(self):


        msg=TreeArray()



        for tree in self.tree_database:


            t=Tree()


            t.id=tree["id"]

            t.x=tree["x"]

            t.y=tree["y"]

            t.z=tree["z"]


            t.confidence=tree["confidence"]

            t.inspected=tree["inspected"]


            msg.trees.append(t)



        self.tree_pub.publish(msg)



    ##################################################
    # RVIZ visualization
    ##################################################

    def publish_marker(self):


        markers=MarkerArray()


        sphere=Marker()


        sphere.header.frame_id="odom"

        sphere.header.stamp=self.get_clock().now().to_msg()


        sphere.ns="trees"

        sphere.id=0


        sphere.type=Marker.SPHERE_LIST

        sphere.action=Marker.ADD


        sphere.scale.x=0.5

        sphere.scale.y=0.5

        sphere.scale.z=0.5


        sphere.pose.orientation.w=1.0



        for tree in self.tree_database:


            p=Point()

            p.x=tree["x"]

            p.y=tree["y"]

            p.z=tree["z"]


            sphere.points.append(p)



        markers.markers.append(sphere)



        self.marker_pub.publish(
            markers
        )



def main(args=None):


    rclpy.init(args=args)


    node=TreeMapper()


    try:

        rclpy.spin(node)


    except KeyboardInterrupt:

        pass


    node.destroy_node()


    rclpy.shutdown()



if __name__=="__main__":

    main()