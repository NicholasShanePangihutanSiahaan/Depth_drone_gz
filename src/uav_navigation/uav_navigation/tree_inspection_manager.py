#!/usr/bin/env python3

import math
import time
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
        self.orbit_start_time = None
        self.target_locked = False

        # self.validation_count = 0

        self.required_orbit = 1
        super().__init__(
            "tree_inspection_manager"
        )

        qos_sensor = QoSProfile(

            reliability=ReliabilityPolicy.BEST_EFFORT,

            history=HistoryPolicy.KEEP_LAST,

            depth=10

        )
        ##################################################
        # Parameter
        ##################################################

        self.min_confidence = 0.3

        self.inspection_radius = 3.0

        self.inspection_altitude = 5.0

        self.orbit_points = 8

        # self.finish_distance = 2.0



        ##################################################
        # State Machine
        ##################################################

        self.state = "WAITING_UAV"

        # daftar pohon
        self.trees = []

        # pose UAV sudah diterima?
        self.have_pose = False

        # pohon yang sedang diinspeksi
        self.current_tree = None

        # orbit waypoint yang sedang digunakan
        self.current_waypoints = None

        # trajectory sudah dikirim?
        self.trajectory_sent = False

        # trajectory sudah selesai?
        self.trajectory_completed = False


        self.orbit_start_time=None

        self.target_locked=False

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
            qos_sensor
        )

        self.create_subscription(
            String,
            "/navigation/follower_status",
            self.follower_callback,
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

        if not self.target_locked:
            self.trees = msg.trees
            return

        for incoming in msg.trees:

            for i, tree in enumerate(self.trees):

                if tree.id == incoming.id:
                    self.trees[i] = incoming
                    break
            if self.target_locked:

                return

        self.trees=msg.trees


        self.get_logger().info(
            f"Received {len(self.trees)} trees"
        )

        for tree in self.trees:

            self.get_logger().info(
                f"Tree ID={tree.id} "
                f"pos=({tree.x:.2f},{tree.y:.2f},{tree.z:.2f}) "
                f"confidence={tree.confidence:.2f} "
                f"inspected={tree.inspected}"
            )



    ##################################################
    # UAV pose
    ##################################################

    def pose_callback(self,msg):

        self.uav_x = msg.pose.position.x

        self.uav_y = msg.pose.position.y

        self.uav_z = msg.pose.position.z

        self.get_logger().info(
            f"[UAV POSE] "
            f"x={self.uav_x:.2f} "
            f"y={self.uav_y:.2f} "
            f"z={self.uav_z:.2f}"
        )

        self.have_pose = True

    ##################################################
    # Path follower status
    ##################################################

    def follower_callback(self, msg):

        status = msg.data

        if status == "TRAJECTORY_COMPLETED":

            self.get_logger().info(
                "Trajectory completed"
            )

            self.trajectory_completed = True

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

        if candidate:

            self.get_logger().info(
                f"[SELECT TREE] "
                f"Selected tree {candidate.id} "
                f"distance={best:.2f} m"
            )

        else:

            self.get_logger().info(
                "[SELECT TREE] No available tree"
            )

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

        self.get_logger().info(
            f"[ORBIT] Generated {len(poses.poses)} orbit points "
            f"radius={self.inspection_radius} "
            f"altitude={self.inspection_altitude}"
        )


        for i,p in enumerate(poses.poses):

            self.get_logger().info(
                f"Orbit {i}: "
                f"x={p.position.x:.2f} "
                f"y={p.position.y:.2f} "
                f"z={p.position.z:.2f}"
            )

        return poses



    ##################################################
    # Status
    ##################################################

    def publish_status(self,text):

        msg = String()

        msg.data = text

        self.status_pub.publish(msg)



    ##################################################
    # Main Loop
    ##################################################

    def loop(self):

        ##################################################
        # Waiting UAV Pose
        ##################################################
        self.get_logger().info(
            f"[STATE] {self.state}"
        )
        if not self.have_pose:

            self.state = "WAITING_UAV"

            self.publish_status(
                "WAITING_UAV_POSE"
            )

            return


        ##################################################
        # Waiting Tree Map
        ##################################################

        if len(self.trees) == 0:

            self.state = "WAITING_TREE_MAP"

            self.publish_status(
                "WAITING_TREE_MAP"
            )

            return


        ##################################################
        # No active tree
        ##################################################

        if (
            self.current_tree is None
            and
            not self.target_locked
        ):
            tree = self.select_tree()

            if tree is None:

                self.state = "MISSION_FINISHED"

                self.publish_status(
                    "MISSION_FINISHED"
                )

                return
            
            self.current_tree = tree

            self.target_locked = True

            ##############################################
            # Start new tree
            ##############################################
            
            self.current_waypoints = (
                self.generate_orbit(tree)
            )

            self.trajectory_sent = False

            self.trajectory_completed = False

            self.state = "START_INSPECTION"


        ##################################################
        # Publish trajectory once
        ##################################################

        if (
            self.state == "START_INSPECTION"
            and
            not self.trajectory_sent
        ):

            self.waypoint_pub.publish(
                self.current_waypoints
            )
            self.orbit_start_time = time.time()
            self.get_logger().info(
            "[MISSION] Inspection waypoint published"
        )

            self.publish_status(
                f"INSPECT_TREE_{self.current_tree.id}"
            )

            self.get_logger().info(

                f"Start inspection Tree {self.current_tree.id}"

            )

            self.trajectory_sent = True

            self.state = "WAIT_TRAJECTORY"

            return


        ##################################################
        # Waiting trajectory completed
        ##################################################

        if self.state == "WAIT_TRAJECTORY":

            if not self.trajectory_completed:

                self.publish_status(

                    f"INSPECT_TREE_{self.current_tree.id}"

                )

                return


            ##############################################
            # Orbit finished
            ##############################################

            if self.trajectory_completed:

                if self.check_orbit_valid():

                    self.finish_tree()

                else:

                    self.get_logger().warn(
                        "Orbit invalid. Regenerate orbit."
                    )

                self.target_locked=False

            # self.current_tree = None

            self.current_waypoints = None

            self.trajectory_sent = False

            self.trajectory_completed = False

            self.state = "SELECT_TREE"
    ##################################################
    # Finish tree
    ##################################################
    def check_orbit_valid(self):


        if self.orbit_start_time is None:
            return False


        elapsed=time.time()-self.orbit_start_time

        self.get_logger().info(
            f"Orbit duration = {elapsed:.1f}s"
        )

        if elapsed < 30:

            self.get_logger().info(
            "Orbit too short"
            )

            return False


        return True

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

        update.validated = True
        update.inspected = True
        update.orbit_count = 1
        
        self.tree_update_pub.publish(
            update
        )
            # update database lokal
        for tree in self.trees:

            if tree.id == update.id:

                tree.inspected = True

                break


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