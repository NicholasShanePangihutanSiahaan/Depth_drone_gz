#!/usr/bin/env python3


import math

import rclpy

from rclpy.node import Node


from uav_interfaces.msg import Tree
from uav_interfaces.msg import TreeArray


from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Point



class TreeInspectionManager(Node):

    def __init__(self):

        super().__init__(
            "tree_inspection_manager"
        )


        ##################################################
        # Parameters
        ##################################################

        # minimal confidence pohon
        self.min_confidence = 0.2


        # jarak UAV ke pohon ketika inspeksi dianggap selesai
        self.inspection_distance = 5.0


        # tinggi inspeksi UAV
        self.inspection_altitude = 8.0


        # radius posisi mengelilingi pohon
        self.inspection_radius = 5.0



        ##################################################
        # State
        ##################################################

        self.state = "WAITING"


        self.trees = []


        self.current_target_id = -1


        ##################################################
        # UAV pose
        ##################################################

        self.uav_x = 0.0
        self.uav_y = 0.0
        self.uav_z = 0.0



        ##################################################
        # Subscriber
        ##################################################

        self.tree_sub = self.create_subscription(
            TreeArray,
            "/map/trees",
            self.tree_callback,
            10
        )



        self.pose_sub = self.create_subscription(
            PoseStamped,
            "/mavros/local_position/pose",
            self.pose_callback,
            10
        )



        ##################################################
        # Publisher
        ##################################################

        # target untuk planner

        self.target_pub = self.create_publisher(
            Point,
            "/mission/inspection_target",
            10
        )



        # update status pohon

        self.tree_update_pub = self.create_publisher(
            Tree,
            "/map/tree_update",
            10
        )



        ##################################################
        # Timer
        ##################################################

        self.timer = self.create_timer(
            1.0,
            self.manager_loop
        )



        self.get_logger().info(
            "Tree Inspection Manager Started"
        )



    ##################################################
    # Receive tree database
    ##################################################

    def tree_callback(self,msg):

        self.trees = msg.trees



    ##################################################
    # Receive UAV pose
    ##################################################

    def pose_callback(self,msg):

        self.uav_x = msg.pose.position.x

        self.uav_y = msg.pose.position.y

        self.uav_z = msg.pose.position.z



    ##################################################
    # Distance
    ##################################################

    def distance(
        self,
        x,
        y
    ):


        return math.sqrt(

            (x-self.uav_x)**2 +

            (y-self.uav_y)**2

        )



    ##################################################
    # Create safe inspection point
    ##################################################

    def create_inspection_point(
        self,
        tree
    ):


        dx = self.uav_x - tree.x

        dy = self.uav_y - tree.y



        dist = math.sqrt(
            dx*dx +
            dy*dy
        )



        # jika tepat di posisi pohon

        if dist < 0.1:

            dx = 1.0
            dy = 0.0

            dist = 1.0



        scale = (
            self.inspection_radius /
            dist
        )



        target = Point()



        target.x = (
            tree.x +
            dx * scale
        )


        target.y = (
            tree.y +
            dy * scale
        )


        target.z = (
            self.inspection_altitude
        )



        return target



    ##################################################
    # Find nearest tree
    ##################################################

    def select_target(self):


        nearest = None


        nearest_distance = float("inf")



        for tree in self.trees:



            if tree.inspected:

                continue



            if tree.confidence < self.min_confidence:

                continue



            d = self.distance(
                tree.x,
                tree.y
            )



            if d < nearest_distance:

                nearest_distance = d

                nearest = tree



        return nearest



    ##################################################
    # Check inspection finished
    ##################################################

    def inspection_complete(
        self,
        tree
    ):


        d = self.distance(
            tree.x,
            tree.y
        )



        if d <= self.inspection_distance:

            return True



        return False



    ##################################################
    # Send tree update
    ##################################################

    def mark_tree_finished(
        self,
        tree
    ):


        update = Tree()


        update.id = tree.id

        update.x = tree.x
        update.y = tree.y
        update.z = tree.z


        update.confidence = tree.confidence


        update.inspected = True



        self.tree_update_pub.publish(
            update
        )



        self.get_logger().info(
            f"Tree {tree.id} inspection completed"
        )



    ##################################################
    # Main state machine
    ##################################################

    def manager_loop(self):


        ##############################################
        # No tree
        ##############################################

        if len(self.trees)==0:


            self.state="WAITING"


            return



        ##############################################
        # Already moving
        ##############################################

        if self.current_target_id != -1:



            current_tree = None



            for tree in self.trees:

                if tree.id == self.current_target_id:

                    current_tree = tree



            if current_tree is None:

                self.current_target_id=-1

                return




            if self.inspection_complete(
                current_tree
            ):



                self.mark_tree_finished(
                    current_tree
                )



                self.current_target_id=-1


                self.state="COMPLETED"



            return




        ##############################################
        # Select new tree
        ##############################################

        target_tree = self.select_target()



        if target_tree is None:


            self.state="FINISHED"


            self.get_logger().info(
                "All trees inspected"
            )


            return




        ##############################################
        # Create target point
        ##############################################

        target_point = self.create_inspection_point(
            target_tree
        )



        self.target_pub.publish(
            target_point
        )



        self.current_target_id = target_tree.id



        self.state="MOVING"



        self.get_logger().info(

            f"""
            New inspection target

            Tree ID : {target_tree.id}

            Tree Position:
            ({target_tree.x:.2f},
             {target_tree.y:.2f})

            UAV Target:
            ({target_point.x:.2f},
             {target_point.y:.2f},
             {target_point.z:.2f})
            """

        )




##################################################


def main(args=None):


    rclpy.init(args=args)


    node = TreeInspectionManager()


    try:

        rclpy.spin(node)


    except KeyboardInterrupt:

        pass



    node.destroy_node()


    rclpy.shutdown()



if __name__=="__main__":

    main()