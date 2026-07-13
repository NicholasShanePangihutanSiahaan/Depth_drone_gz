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


from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Point

from nav_msgs.msg import Path

from std_msgs.msg import String



class PathFollower(Node):

    def __init__(self):

        super().__init__(
            "path_follower"
        )

        qos_sensor = QoSProfile(

            reliability=ReliabilityPolicy.BEST_EFFORT,

            history=HistoryPolicy.KEEP_LAST,

            depth=10

        )
        ##################################################
        # Parameters
        ##################################################

        # jarak dianggap waypoint tercapai

        self.waypoint_threshold = 0.8


        # minimal waktu update

        self.update_rate = 0.1



        ##################################################
        # State
        ##################################################

        self.state = "WAITING_PATH"


        self.path = []


        self.current_index = 0


        self.current_pose = None



        ##################################################
        # Subscriber
        ##################################################

        self.pose_sub = self.create_subscription(
            PoseStamped,
            "/mavros/local_position/pose",
            self.pose_callback,
            qos_sensor
        )



        self.path_sub = self.create_subscription(
            Path,
            "/navigation/trajectory",
            self.path_callback,
            10
        )



        ##################################################
        # Publisher
        ##################################################

        self.target_pub = self.create_publisher(
            Point,
            "/control/target_point",
            10
        )



        self.status_pub = self.create_publisher(
            String,
            "/navigation/follower_status",
            10
        )



        ##################################################

        self.timer = self.create_timer(
            self.update_rate,
            self.follow_loop
        )



        self.get_logger().info(
            "Path Follower Started"
        )



    ##################################################
    # Receive UAV position
    ##################################################

    def pose_callback(self,msg):

        self.current_pose = msg.pose



    ##################################################
    # Receive trajectory
    ##################################################

    def path_callback(self,msg):

        self.path = msg.poses


        self.current_index = 0

        self.get_logger().info(
            f"[FOLLOWER] Received trajectory "
            f"{len(self.path)} points"
        )

        if len(self.path)>0:

            self.state="FOLLOWING"


            self.get_logger().info(
                f"Trajectory received : {len(self.path)} points"
            )

        else:

            self.state="WAITING_PATH"



    ##################################################
    # Distance calculation
    ##################################################

    def distance(
        self,
        target
    ):


        dx = (
            target.x -
            self.current_pose.position.x
        )


        dy = (
            target.y -
            self.current_pose.position.y
        )


        dz = (
            target.z -
            self.current_pose.position.z
        )


        return math.sqrt(
            dx*dx+
            dy*dy+
            dz*dz
        )



    ##################################################
    # Publish target
    ##################################################

    def publish_target(
        self,
        pose
    ):


        msg = Point()


        msg.x = pose.position.x

        msg.y = pose.position.y

        msg.z = pose.position.z


        self.target_pub.publish(msg)

        self.get_logger().info(
            f"[TARGET] "
            f"x={msg.x:.2f} "
            f"y={msg.y:.2f} "
            f"z={msg.z:.2f}"
        )

    ##################################################
    # Publish status
    ##################################################

    def publish_status(
        self,
        text
    ):

        msg = String()

        msg.data = text

        self.status_pub.publish(msg)



    ##################################################
    # Main follower loop
    ##################################################

    def follow_loop(self):


        ##################################################
        # No pose
        ##################################################

        if self.current_pose is None:

            self.publish_status(
                "WAITING_UAV_POSE"
            )

            return



        ##################################################
        # No path
        ##################################################

        if len(self.path)==0:

            self.state="WAITING_PATH"

            self.publish_status(
                "WAITING_PATH"
            )

            return



        ##################################################
        # Finished
        ##################################################

        if self.current_index >= len(self.path):


            self.state="COMPLETED"


            self.publish_status(
                "TRAJECTORY_COMPLETED"
            )


            return



        ##################################################
        # Current waypoint
        ##################################################

        target_pose = self.path[
            self.current_index
        ]



        distance = self.distance(
            target_pose.pose.position
        )



        ##################################################
        # Waypoint reached
        ##################################################

        if distance < self.waypoint_threshold:


            self.current_index += 1



            self.get_logger().info(

                f"Waypoint reached "
                f"{self.current_index}/"
                f"{len(self.path)}"

            )



            if self.current_index >= len(self.path):


                self.state="COMPLETED"


                self.publish_status(
                    "TRAJECTORY_COMPLETED"
                )


                return



            target_pose = self.path[
                self.current_index
            ]



        ##################################################
        # Send target
        ##################################################

        self.publish_target(
            target_pose.pose
        )



        self.state="FOLLOWING"



        self.publish_status(

            f"FOLLOWING "
            f"{self.current_index+1}/"
            f"{len(self.path)} "
            f"distance={distance:.2f}"

        )





##################################################

def main(args=None):

    rclpy.init(args=args)


    node = PathFollower()


    try:

        rclpy.spin(node)


    except KeyboardInterrupt:

        pass



    node.destroy_node()

    rclpy.shutdown()



if __name__=="__main__":

    main()