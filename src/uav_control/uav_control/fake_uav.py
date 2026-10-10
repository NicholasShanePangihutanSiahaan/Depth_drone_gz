#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TwistStamped


class FakeUAV(Node):

    def __init__(self):

        super().__init__("fake_uav")

        #########################################
        # UAV State
        #########################################

        self.x = 0.0
        self.y = 0.0
        self.z = 8.0

        self.vx = 0.0
        self.vy = 0.0
        self.vz = 0.0

        self.dt = 0.05

        #########################################
        # Subscriber
        #########################################

        self.vel_sub = self.create_subscription(
            TwistStamped,
            "/mavros/setpoint_velocity/cmd_vel",
            self.velocity_callback,
            10
        )

        #########################################
        # Publisher
        #########################################

        self.pose_pub = self.create_publisher(
            PoseStamped,
            "/mavros/local_position/pose",
            10
        )

        #########################################

        self.timer = self.create_timer(
            self.dt,
            self.update
        )

        self.get_logger().info("Fake UAV Started")

    #########################################

    def velocity_callback(self, msg):

        self.vx = msg.twist.linear.x
        self.vy = msg.twist.linear.y
        self.vz = msg.twist.linear.z

    #########################################

    def update(self):

        ####################################
        # Integrate velocity
        ####################################

        self.x += self.vx * self.dt
        self.y += self.vy * self.dt
        self.z += self.vz * self.dt

        ####################################
        # Publish pose
        ####################################

        pose = PoseStamped()

        pose.header.stamp = (
            self.get_clock()
            .now()
            .to_msg()
        )

        pose.header.frame_id = "odom"

        pose.pose.position.x = self.x
        pose.pose.position.y = self.y
        pose.pose.position.z = self.z

        pose.pose.orientation.w = 1.0

        self.pose_pub.publish(pose)


def main(args=None):

    rclpy.init(args=args)

    node = FakeUAV()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()