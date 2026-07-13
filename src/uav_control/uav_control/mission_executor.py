#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from std_msgs.msg import Bool

from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseArray

from mavros_msgs.msg import State

from mavros_msgs.srv import CommandBool
from mavros_msgs.srv import SetMode

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

        self.state = "INIT"

        self.current_pose = None

        self.current_state = State()

        self.home_saved = False

        ############################################
        # subscriber
        ############################################

        self.create_subscription(
            PoseStamped,
            "/mavros/local_position/pose",
            self.pose_callback,
            qos
        )

        self.create_subscription(
            State,
            "/mavros/state",
            self.state_callback,
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

        self.wp_pub = self.create_publisher(
            PoseArray,
            "/mission/inspection_waypoints",
            10
        )

        ############################################
        # service
        ############################################

        self.arm_client = self.create_client(
            CommandBool,
            "/mavros/cmd/arming"
        )

        self.mode_client = self.create_client(
            SetMode,
            "/mavros/set_mode"
        )

        ############################################

        self.timer = self.create_timer(
            1.0,
            self.loop
        )

        self.get_logger().info("Mission Executor Started")

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

    def state_callback(self, msg):

        self.current_state = msg

    #######################################################

    def arm(self):

        if self.current_state.armed:
            return True

        if not self.arm_client.wait_for_service(timeout_sec=1.0):
            return False

        req = CommandBool.Request()
        req.value = True

        future = self.arm_client.call_async(req)

        self.get_logger().info("Arming...")

        return True

    #######################################################

    def guided(self):

        if self.current_state.mode == "GUIDED":
            return True

        if not self.mode_client.wait_for_service(timeout_sec=1.0):
            return False

        req = SetMode.Request()

        req.custom_mode = "GUIDED"

        future = self.mode_client.call_async(req)

        self.get_logger().info("GUIDED mode...")

        return True

    #######################################################

    def publish_inspection(self):

        msg = PoseArray()

        msg.header.frame_id = "map"

        altitude = 3.0

        points = [

            (5.0,0.0),
            (5.0,5.0),
            (0.0,5.0),
            (0.0,0.0)

        ]

        for x,y in points:

            p = Pose()

            p.position.x = x
            p.position.y = y
            p.position.z = altitude

            p.orientation.w = 1.0

            msg.poses.append(p)

        self.wp_pub.publish(msg)

        self.get_logger().info("Inspection waypoint published")

    #######################################################

    def loop(self):

        status = String()

        ###################################################

        if self.current_pose is None:

            return

        ###################################################

        if self.state == "INIT":

            self.guided()

            self.arm()

            status.data = "ARMING"

            if self.current_state.mode == "GUIDED" and self.current_state.armed:

                self.state = "TAKEOFF"

        ###################################################

        elif self.state == "TAKEOFF":

            if self.current_pose.position.z >= 2.8:

                self.state = "MISSION"

            status.data = "TAKEOFF"

        ###################################################

        elif self.state == "MISSION":

            self.publish_inspection()

            status.data = "MISSION"

            self.state = "WAIT"

        ###################################################

        elif self.state == "WAIT":

            status.data = "FOLLOW_PATH"

        ###################################################

        self.status_pub.publish(status)


def main(args=None):

    rclpy.init(args=args)

    node = MissionExecutor()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == "__main__":

    main()