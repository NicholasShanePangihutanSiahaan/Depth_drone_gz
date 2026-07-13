#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from std_msgs.msg import Bool

from geometry_msgs.msg import PoseStamped

from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from rclpy.qos import HistoryPolicy


class MissionExecutor(Node):

    def __init__(self):

        super().__init__("mission_executor")

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        ############################################
        self.mission_status = ""
        
        self.state = "INIT"

        self.current_pose = None

        self.home_saved = False

        ############################################
        # subscriber
        ############################################

        self.create_subscription(
            String,
            "/mission/status",
            self.mission_callback,
            10
        )

        self.create_subscription(
            PoseStamped,
            "/mavros/local_position/pose",
            self.pose_callback,
            qos
        )

        self.flight_status = ""

        self.create_subscription(
            String,
            "/flight/status",
            self.flight_callback,
            10
        )

        ############################################
        # publisher
        ############################################

        self.status_pub = self.create_publisher(
            String,
            "/mission/executor_status",
            10
        )

        self.start_pub = self.create_publisher(
            Bool,
            "/mission/start_inspection",
            10
        )

        self.land_pub = self.create_publisher(
            Bool,
            "/mission/land",
            10
        )


        self.takeoff_pub = self.create_publisher(
            Bool,
            "/mission/takeoff",
            10
        )
        ############################################

        self.timer = self.create_timer(
            1.0,
            self.loop
        )

        self.get_logger().info("Mission Executor Started")

    def mission_callback(self,msg):

        self.mission_status = msg.data

    def flight_callback(self,msg):

        self.flight_status = msg.data

    #######################################################

    def pose_callback(self, msg):

        self.current_pose = msg.pose

        if not self.home_saved:

            self.home = msg.pose.position

            self.home_saved = True

            self.get_logger().info(
                f"Home : {self.home.x:.2f}, {self.home.y:.2f}, {self.home.z:.2f}"
            )

    #######################################################

    def loop(self):

        status = String()
        
        ###################################################

        if self.current_pose is None:

            return

        ###################################################
        
        if self.state=="INIT":

            msg=Bool()

            msg.data=True

            self.takeoff_pub.publish(msg)

            self.state="WAIT_HOVER"

        ###################################################

        elif self.state=="WAIT_HOVER":

            if self.flight_status=="HOVER":

                self.state="START_INSPECTION"

        ###################################################

        elif self.state=="START_INSPECTION":

            msg=Bool()

            msg.data=True

            self.start_pub.publish(msg)

            self.state="WAIT_MISSION"

        ###################################################

        elif self.state=="WAIT_MISSION":

            if self.mission_status=="MISSION_FINISHED":

                self.state="LAND"

        elif self.state=="LAND":
        
            msg=Bool()

            msg.data=True

            self.land_pub.publish(msg)

            self.state="DONE"

        elif self.state=="DONE":

            pass
        ###################################################
        status.data = self.state

        self.status_pub.publish(status)


def main(args=None):

    rclpy.init(args=args)

    node = MissionExecutor()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == "__main__":

    main()