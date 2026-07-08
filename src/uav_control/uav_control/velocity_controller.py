#!/usr/bin/env python3

import math

import rclpy

from rclpy.node import Node

from geometry_msgs.msg import Point
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TwistStamped

from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from rclpy.qos import HistoryPolicy


class VelocityController(Node):

    def __init__(self):

        super().__init__("velocity_controller")

        qos_sensor = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.current_pose = None
        self.target_point = None

        # ---------- Controller Gain ----------
        self.kp_xy = 0.8
        self.kp_z = 0.6

        # ---------- Velocity Limit ----------
        self.max_vel_xy = 2.0
        self.max_vel_z = 1.0

        self.pose_sub = self.create_subscription(
            PoseStamped,
            "/mavros/local_position/pose",
            self.pose_callback,
            qos_sensor
        )

        self.target_sub = self.create_subscription(
            Point,
            "/control/target_point",
            self.target_callback,
            10
        )

        self.vel_pub = self.create_publisher(
            TwistStamped,
            "/mavros/setpoint_velocity/cmd_vel",
            10
        )

        self.timer = self.create_timer(
            0.05,
            self.control_loop
        )

        self.get_logger().info("Velocity Controller Started")

    #####################################################

    def pose_callback(self, msg):

        self.current_pose = msg.pose.position

    #####################################################

    def target_callback(self, msg):

        self.target_point = msg

    #####################################################

    def saturate(self, value, limit):

        if value > limit:
            return limit

        if value < -limit:
            return -limit

        return value

    #####################################################

    def control_loop(self):

        if self.current_pose is None:
            return

        if self.target_point is None:
            return

        ex = self.target_point.x - self.current_pose.x
        ey = self.target_point.y - self.current_pose.y
        ez = self.target_point.z - self.current_pose.z

        vx = self.kp_xy * ex
        vy = self.kp_xy * ey
        vz = self.kp_z * ez

        vx = self.saturate(vx, self.max_vel_xy)
        vy = self.saturate(vy, self.max_vel_xy)
        vz = self.saturate(vz, self.max_vel_z)

        cmd = TwistStamped()

        cmd.header.stamp = self.get_clock().now().to_msg()

        cmd.twist.linear.x = vx
        cmd.twist.linear.y = vy
        cmd.twist.linear.z = vz

        cmd.twist.angular.x = 0.0
        cmd.twist.angular.y = 0.0
        cmd.twist.angular.z = 0.0

        self.vel_pub.publish(cmd)


############################################################

def main(args=None):

    rclpy.init(args=args)

    node = VelocityController()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == "__main__":
    main()