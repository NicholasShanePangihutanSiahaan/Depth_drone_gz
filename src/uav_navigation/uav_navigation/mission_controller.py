#!/usr/bin/env python3

import rclpy

from rclpy.node import Node

from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    HistoryPolicy
)

from std_msgs.msg import String
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Point


class MissionController(Node):

    def __init__(self):

        super().__init__("mission_controller")


        qos_sensor = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )


        self.tree_map_ready = False
        self.global_path_ready = False

        self.navigation_started = False
        self.mission_finished = False


        self.total_waypoints = 0

        self.current_x = 0.0
        self.current_y = 0.0
        self.current_z = 0.0


        self.current_waypoint = 0



        # TREE MAP

        self.create_subscription(
            String,
            "/map/tree_locations",
            self.tree_callback,
            10
        )


        # GLOBAL PATH

        self.create_subscription(
            Path,
            "/navigation/global_path",
            self.path_callback,
            10
        )


        # MAVROS POSITION

        self.create_subscription(
            PoseStamped,
            "/mavros/local_position/pose",
            self.pose_callback,
            qos_sensor
        )



        # OUTPUT STATUS

        self.status_pub = self.create_publisher(
            String,
            "/mission/status",
            10
        )


        # OUTPUT TARGET

        self.target_pub = self.create_publisher(
            Point,
            "/navigation/target_point",
            10
        )


        self.timer = self.create_timer(
            1.0,
            self.timer_callback
        )


        self.get_logger().info(
            "Mission Controller Started"
        )



    def tree_callback(self,msg):

        if not self.tree_map_ready:

            self.tree_map_ready=True

            self.get_logger().info(
                "Tree map received"
            )



    def path_callback(self,msg):

        self.global_path_ready=True

        self.total_waypoints=len(msg.poses)


        self.get_logger().info(
            f"Global path received : {self.total_waypoints} points"
        )



    def pose_callback(self,msg):

        self.current_x = msg.pose.position.x
        self.current_y = msg.pose.position.y
        self.current_z = msg.pose.position.z



    def timer_callback(self):


        status=String()



        if not self.tree_map_ready:

            status.data="WAIT_TREE_MAP"

            self.status_pub.publish(status)

            return



        if not self.global_path_ready:

            status.data="WAIT_GLOBAL_PATH"

            self.status_pub.publish(status)

            return



        if not self.navigation_started:


            self.navigation_started=True


            self.get_logger().info(
                "MISSION STARTED"
            )



        status.data=(

            f"NAVIGATING | "
            f"Waypoint={self.current_waypoint}/"
            f"{self.total_waypoints} | "
            f"UAV=("
            f"{self.current_x:.2f},"
            f"{self.current_y:.2f},"
            f"{self.current_z:.2f})"

        )


        self.status_pub.publish(status)



        # sementara kirim waypoint pertama
        # nanti diganti waypoint_manager


        target=Point()

        target.x=self.current_x+5.0
        target.y=self.current_y
        target.z=5.0


        self.target_pub.publish(target)




def main(args=None):

    rclpy.init(args=args)

    node=MissionController()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()



if __name__=="__main__":

    main()