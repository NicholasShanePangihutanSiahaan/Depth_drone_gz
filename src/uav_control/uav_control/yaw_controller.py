#!/usr/bin/env python3

import math

import rclpy

from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Point
from geometry_msgs.msg import TwistStamped

from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from rclpy.qos import HistoryPolicy


def quaternion_to_yaw(q):

    siny = 2.0 * (q.w * q.z + q.x * q.y)

    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)

    return math.atan2(siny, cosy)


class YawController(Node):

    def __init__(self):

        super().__init__("yaw_controller")

        qos_sensor = QoSProfile(

            reliability=ReliabilityPolicy.BEST_EFFORT,

            history=HistoryPolicy.KEEP_LAST,

            depth=10

        )

        self.current_pose = None

        self.current_yaw = 0.0

        self.target = None

        self.kp = 1.2

        self.max_yaw_rate = 1.0

        self.create_subscription(

            PoseStamped,

            "/mavros/local_position/pose",

            self.pose_callback,

            qos_sensor

        )

        self.create_subscription(

            Point,

            "/control/target_point",

            self.target_callback,

            10

        )

        self.pub = self.create_publisher(

            TwistStamped,

            "/mavros/setpoint_velocity/cmd_vel",

            10

        )

        self.timer = self.create_timer(

            0.05,

            self.control_loop

        )

        self.get_logger().info("Yaw Controller Started")

    ###################################################

    def pose_callback(self, msg):

        self.current_pose = msg.pose.position

        self.current_yaw = quaternion_to_yaw(

            msg.pose.orientation

        )

    ###################################################

    def target_callback(self, msg):

        self.target = msg

    ###################################################

    def normalize_angle(self, angle):

        while angle > math.pi:

            angle -= 2.0 * math.pi

        while angle < -math.pi:

            angle += 2.0 * math.pi

        return angle

    ###################################################

    def saturate(self, value, limit):

        if value > limit:

            return limit

        if value < -limit:

            return -limit

        return value

    ###################################################

    def control_loop(self):

        if self.current_pose is None:

            return

        if self.target is None:

            return

        dx = self.target.x - self.current_pose.x

        dy = self.target.y - self.current_pose.y

        desired_yaw = math.atan2(dy, dx)

        error = self.normalize_angle(

            desired_yaw - self.current_yaw

        )

        yaw_rate = self.kp * error

        yaw_rate = self.saturate(

            yaw_rate,

            self.max_yaw_rate

        )

        cmd = TwistStamped()

        cmd.header.stamp = self.get_clock().now().to_msg()

        cmd.twist.angular.z = yaw_rate

        self.pub.publish(cmd)


#######################################################

def main(args=None):

    rclpy.init(args=args)

    node = YawController()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == "__main__":

    main()