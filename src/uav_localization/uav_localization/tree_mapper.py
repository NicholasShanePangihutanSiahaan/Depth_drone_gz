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
        # Parameters
        ##################################################

        self.frame_id = "odom"


        # jarak maksimum pohon dianggap sama
        self.merge_distance = 1.5


        # confidence
        self.max_confidence = 1.0

        self.new_tree_confidence = 0.2

        self.confidence_increment = 0.15

        self.confidence_decay = 0.01


        # lama tidak terlihat
        self.timeout = 30.0



        ##################################################
        # Database
        ##################################################

        # format:
        #
        # {
        #   id:
        #   x:
        #   y:
        #   z:
        #   confidence:
        #   inspected:
        #   count:
        #   last_seen:
        # }

        self.tree_database = {}


        self.next_tree_id = 1



        ##################################################
        # Subscribers
        ##################################################

        # hasil deteksi perception

        self.create_subscription(

            Point,

            "/perception/tree_target",

            self.tree_callback,

            10

        )


        # hasil inspeksi

        self.create_subscription(

            Tree,

            "/map/tree_update",

            self.tree_update_callback,

            10

        )



        ##################################################
        # Publishers
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
        # Timer
        ##################################################

        self.timer = self.create_timer(

            5.0,

            self.update_confidence

        )



        self.get_logger().info(
            "Tree Mapper Started"
        )



    ##################################################
    # New tree detection
    ##################################################

    def tree_callback(self,msg):


        x = msg.x
        y = msg.y
        z = msg.z



        nearest_id = None

        nearest_distance = float("inf")



        ##################################################
        # Search existing tree
        ##################################################

        for tree_id,tree in self.tree_database.items():


            d = math.sqrt(

                (x-tree["x"])**2 +

                (y-tree["y"])**2

            )


            if d < nearest_distance:

                nearest_distance = d

                nearest_id = tree_id



        ##################################################
        # Update existing tree
        ##################################################

        if (

            nearest_id is not None

            and

            nearest_distance < self.merge_distance

        ):


            tree = self.tree_database[nearest_id]


            tree["count"] += 1


            alpha = 1.0 / tree["count"]


            tree["x"] += alpha * (
                x-tree["x"]
            )


            tree["y"] += alpha * (
                y-tree["y"]
            )


            tree["z"] += alpha * (
                z-tree["z"]
            )



            tree["confidence"] = min(

                tree["confidence"]

                +

                self.confidence_increment,

                self.max_confidence

            )


            tree["last_seen"] = time.time()



            self.get_logger().debug(

                f"Tree {nearest_id} updated"

            )



        ##################################################
        # New tree
        ##################################################

        else:


            tree_id = self.next_tree_id


            self.tree_database[tree_id] = {


                "id":tree_id,


                "x":x,

                "y":y,

                "z":z,


                "confidence":
                self.new_tree_confidence,


                "count":1,


                "inspected":False,


                "last_seen":
                time.time()

            }


            self.next_tree_id += 1



            self.get_logger().info(

                f"New tree {tree_id} "
                f"({x:.2f},{y:.2f},{z:.2f})"

            )



        self.publish_tree()

        self.publish_marker()



    ##################################################
    # Receive inspection result
    ##################################################

    def tree_update_callback(self,msg):


        if msg.id not in self.tree_database:

            return



        tree = self.tree_database[msg.id]



        tree["inspected"] = msg.inspected


        tree["confidence"] = msg.confidence



        self.get_logger().info(

            f"Tree {msg.id} inspected="
            f"{msg.inspected}"

        )



        self.publish_tree()

        self.publish_marker()



    ##################################################
    # Confidence aging
    ##################################################

    def update_confidence(self):


        now = time.time()



        for tree in self.tree_database.values():


            elapsed = now-tree["last_seen"]



            if elapsed > self.timeout:


                tree["confidence"] -= (

                    self.confidence_decay

                )



                if tree["confidence"] < 0:

                    tree["confidence"]=0



        self.publish_tree()



    ##################################################
    # Publish TreeArray
    ##################################################

    def publish_tree(self):


        msg = TreeArray()


        for tree in self.tree_database.values():


            t = Tree()


            t.id = tree["id"]


            t.x = tree["x"]

            t.y = tree["y"]

            t.z = tree["z"]


            t.confidence = tree["confidence"]


            t.inspected = tree["inspected"]



            msg.trees.append(t)



        self.tree_pub.publish(msg)



    ##################################################
    # RVIZ Marker
    ##################################################

    def publish_marker(self):


        markers = MarkerArray()



        ##################################################
        # Tree sphere
        ##################################################

        sphere = Marker()


        sphere.header.frame_id = self.frame_id

        sphere.header.stamp = (
            self.get_clock()
            .now()
            .to_msg()
        )


        sphere.ns="trees"

        sphere.id=0


        sphere.type=Marker.SPHERE_LIST

        sphere.action=Marker.ADD



        sphere.scale.x=0.5

        sphere.scale.y=0.5

        sphere.scale.z=0.5



        sphere.pose.orientation.w=1.0



        for tree in self.tree_database.values():


            p=Point()

            p.x=tree["x"]

            p.y=tree["y"]

            p.z=tree["z"]


            sphere.points.append(p)



        markers.markers.append(sphere)



        ##################################################
        # Tree ID text
        ##################################################

        marker_id=1000



        for tree in self.tree_database.values():


            text=Marker()


            text.header.frame_id=self.frame_id


            text.header.stamp=(

                self.get_clock()
                .now()
                .to_msg()

            )


            text.ns="tree_id"

            text.id=marker_id

            marker_id+=1



            text.type=Marker.TEXT_VIEW_FACING

            text.action=Marker.ADD



            text.pose.position.x=tree["x"]

            text.pose.position.y=tree["y"]

            text.pose.position.z=1.5



            text.pose.orientation.w=1.0



            text.scale.z=0.5



            text.text=(

                f"{tree['id']} "

                f"C:{tree['confidence']:.1f}"

            )


            markers.markers.append(text)



        self.marker_pub.publish(markers)



##################################################

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