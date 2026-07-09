#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node

from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    HistoryPolicy,
    DurabilityPolicy
)

from uav_interfaces.msg import Tree
from uav_interfaces.msg import TreeArray

from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import Pose

from std_msgs.msg import String

class TreeInspectionManager(Node):

    def __init__(self):

        super().__init__(
            "tree_inspection_manager"
        )


        ##################################################
        # Parameter
        ##################################################

        self.min_confidence = 0.3

        self.inspection_radius = 6.0

        self.inspection_altitude = 8.0

        self.orbit_points = 8

        self.finish_distance = 2.0



        ##################################################
        # State
        ##################################################

        self.state = "WAITING"

        self.trees = []

        self.current_tree = None

        self.waypoints = None

        self.orbit_index = 0

        self.have_pose = False



        ##################################################
        # UAV Pose
        ##################################################

        self.uav_x = 0.0
        self.uav_y = 0.0
        self.uav_z = 0.0



        ##################################################
        # QoS
        ##################################################

        map_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )



        ##################################################
        # Subscriber
        ##################################################

        self.create_subscription(
            TreeArray,
            "/map/trees",
            self.tree_callback,
            map_qos
        )


        self.create_subscription(
            PoseStamped,
            "/mavros/local_position/pose",
            self.pose_callback,
            10
        )



        ##################################################
        # Publisher
        ##################################################

        self.waypoint_pub = self.create_publisher(
            PoseArray,
            "/mission/inspection_waypoints",
            10
        )


        self.status_pub = self.create_publisher(
            String,
            "/mission/status",
            10
        )


        self.tree_update_pub = self.create_publisher(
            Tree,
            "/map/tree_update",
            10
        )



        ##################################################

        self.timer = self.create_timer(
            1.0,
            self.loop
        )


        self.get_logger().info(
            "Tree Inspection Manager Ready"
        )



    ##################################################
    # Tree database
    ##################################################

    def tree_callback(self,msg):

        self.trees = msg.trees


        self.get_logger().info(
            f"Received {len(self.trees)} trees"
        )



    ##################################################
    # UAV pose
    ##################################################

    def pose_callback(self,msg):

        self.uav_x = msg.pose.position.x

        self.uav_y = msg.pose.position.y

        self.uav_z = msg.pose.position.z


        self.have_pose = True



    ##################################################
    # Distance
    ##################################################

    def distance(self,x,y):

        return math.sqrt(

            (x-self.uav_x)**2 +

            (y-self.uav_y)**2

        )



    ##################################################
    # Select nearest tree
    ##################################################

    def select_tree(self):


        candidate = None

        best = float("inf")


        for tree in self.trees:


            if tree.inspected:

                continue


            if tree.confidence < self.min_confidence:

                continue



            d = self.distance(
                tree.x,
                tree.y
            )



            if d < best:

                best = d

                candidate = tree



        return candidate



    ##################################################
    # Generate orbit waypoint
    ##################################################

    def generate_orbit(self,tree):


        poses = PoseArray()


        poses.header.frame_id = "odom"

        poses.header.stamp = (
            self.get_clock()
            .now()
            .to_msg()
        )



        for i in range(self.orbit_points):


            angle = (
                2 *
                math.pi *
                i /
                self.orbit_points
            )


            pose = Pose()


            pose.position.x = (
                tree.x +
                self.inspection_radius *
                math.cos(angle)
            )


            pose.position.y = (
                tree.y +
                self.inspection_radius *
                math.sin(angle)
            )


            pose.position.z = (
                self.inspection_altitude
            )


            pose.orientation.w = 1.0


            poses.poses.append(pose)



        return poses



    ##################################################
    # Status
    ##################################################

    def publish_status(self,text):

        msg = String()

        msg.data = text

        self.status_pub.publish(msg)



    ##################################################
    # Main loop
    ##################################################

    def loop(self):


        if not self.have_pose:

            self.publish_status(
                "WAITING_UAV_POSE"
            )

            return



        if len(self.trees)==0:

            self.publish_status(
                "WAITING_TREE_MAP"
            )

            return



        ##################################################
        # Choose tree
        ##################################################

        if self.current_tree is None:


            tree = self.select_tree()



            if tree is None:


                self.state="FINISHED"


                self.publish_status(
                    "MISSION_FINISHED"
                )

                return



            self.current_tree = tree


            self.waypoints = (
                self.generate_orbit(tree)
            )


            self.orbit_index = 0


            self.waypoint_pub.publish(
                self.waypoints
            )


            self.state="INSPECTING"


            self.publish_status(
                f"INSPECT_TREE_{tree.id}"
            )


            self.get_logger().info(

                f"Start inspection Tree {tree.id}"

            )


            return



        ##################################################
        # Check waypoint progress
        ##################################################

        target = self.waypoints.poses[
            self.orbit_index
        ]


        d = self.distance(
            target.position.x,
            target.position.y
        )


        if d < self.finish_distance:


            self.orbit_index += 1



            if self.orbit_index >= len(
                self.waypoints.poses
            ):


                self.finish_tree()
        self.get_logger().info(
            f"Publishing {len(self.waypoints.poses)} orbit waypoints"
        )
        
        self.waypoint_pub.publish(self.waypoints)



    ##################################################
    # Finish tree
    ##################################################

    def finish_tree(self):


        update = Tree()


        update.id = self.current_tree.id

        update.x = self.current_tree.x

        update.y = self.current_tree.y

        update.z = self.current_tree.z


        update.confidence = (
            self.current_tree.confidence
        )


        update.inspected = True



        self.tree_update_pub.publish(
            update
        )


        self.publish_status(
            f"TREE_{update.id}_DONE"
        )


        self.current_tree = None




def main(args=None):

    rclpy.init(args=args)

    node = TreeInspectionManager()

    try:

        rclpy.spin(node)

    except KeyboardInterrupt:

        pass


    node.destroy_node()

    rclpy.shutdown()



if __name__ == "__main__":

    main()