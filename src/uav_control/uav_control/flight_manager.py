#!/usr/bin/env python3

import math

import rclpy

from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Point

from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool
from mavros_msgs.srv import SetMode
from mavros_msgs.srv import CommandTOL


class FlightManager(Node):

    def __init__(self):

        super().__init__("flight_manager")

        self.state = State()

        self.pose = PoseStamped()

        self.target = Point()

        self.have_target = False

        self.create_subscription(
            State,
            "/mavros/state",
            self.state_callback,
            10
        )

        self.create_subscription(
            PoseStamped,
            "/mavros/local_position/pose",
            self.pose_callback,
            10
        )

        self.create_subscription(
            Point,
            "/navigation/target_point",
            self.target_callback,
            10
        )

        self.setpoint_pub = self.create_publisher(
            PoseStamped,
            "/mavros/setpoint_position/local",
            10
        )

        self.arm_client = self.create_client(
            CommandBool,
            "/mavros/cmd/arming"
        )

        self.mode_client = self.create_client(
            SetMode,
            "/mavros/set_mode"
        )

        self.land_client = self.create_client(
            CommandTOL,
            "/mavros/cmd/land"
        )

        self.stage = "TAKEOFF"

        self.takeoff_height = 5.0

        self.timer = self.create_timer(
            0.05,
            self.control_loop
        )

        self.get_logger().info("Flight Manager Started")

    def state_callback(self, msg):

        self.state = msg

    def pose_callback(self, msg):

        self.pose = msg

    def target_callback(self, msg):

        self.target = msg

        self.have_target = True

    def arm(self):

        if not self.arm_client.wait_for_service(timeout_sec=1.0):
            return

        req = CommandBool.Request()

        req.value = True

        self.arm_client.call_async(req)

    def set_offboard(self):

        if not self.mode_client.wait_for_service(timeout_sec=1.0):
            return

        req = SetMode.Request()

        req.custom_mode = "OFFBOARD"

        self.mode_client.call_async(req)

    def land(self):

        if not self.land_client.wait_for_service(timeout_sec=1.0):
            return

        req = CommandTOL.Request()

        req.altitude = 0.0
        req.latitude = 0.0
        req.longitude = 0.0
        req.min_pitch = 0.0
        req.yaw = 0.0

        self.land_client.call_async(req)

    def publish_position(self, x, y, z):

        msg = PoseStamped()

        msg.header.stamp = self.get_clock().now().to_msg()

        msg.header.frame_id = "map"

        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        msg.pose.position.z = float(z)

        msg.pose.orientation.w = 1.0

        self.setpoint_pub.publish(msg)

    def reached(self, x, y, z):

        dx = self.pose.pose.position.x - x
        dy = self.pose.pose.position.y - y
        dz = self.pose.pose.position.z - z

        d = math.sqrt(dx * dx + dy * dy + dz * dz)

        return d < 0.5

    def control_loop(self):

        self.set_offboard()

        self.arm()

        if self.stage == "TAKEOFF":

            self.publish_position(
                self.pose.pose.position.x,
                self.pose.pose.position.y,
                self.takeoff_height
            )

            if self.reached(
                self.pose.pose.position.x,
                self.pose.pose.position.y,
                self.takeoff_height
            ):

                self.stage = "MISSION"

                self.get_logger().info("Takeoff Complete")

        elif self.stage == "MISSION":

            if self.have_target:

                self.publish_position(
                    self.target.x,
                    self.target.y,
                    self.takeoff_height
                )

        elif self.stage == "LAND":

            self.land()


def main(args=None):

    rclpy.init(args=args)

    node = FlightManager()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == "__main__":

    main()