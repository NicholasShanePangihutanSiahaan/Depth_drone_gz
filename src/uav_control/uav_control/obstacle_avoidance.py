#!/usr/bin/env python3

import math

import rclpy

from rclpy.node import Node

from geometry_msgs.msg import TwistStamped

from nav_msgs.msg import OccupancyGrid


class ObstacleAvoidance(Node):

    def __init__(self):

        super().__init__("obstacle_avoidance")

        self.grid = None

        self.cmd = TwistStamped()

        self.create_subscription(
            OccupancyGrid,
            "/occupancy_map",
            self.map_callback,
            10
        )

        self.create_subscription(
            TwistStamped,
            "/control/cmd_vel",
            self.cmd_callback,
            10
        )

        self.pub = self.create_publisher(
            TwistStamped,
            "/control/cmd_vel_safe",
            10
        )

        self.timer = self.create_timer(
            0.05,
            self.control_loop
        )

        self.get_logger().info("Obstacle Avoidance Started")

    #######################################################

    def map_callback(self,msg):

        self.grid = msg

    #######################################################

    def cmd_callback(self,msg):

        self.cmd = msg

    #######################################################

    def obstacle_ahead(self):

        if self.grid is None:

            return False

        width = self.grid.info.width

        height = self.grid.info.height

        cx = width // 2

        cy = height // 2

        data = self.grid.data

        for y in range(cy-3, cy+3):

            for x in range(cx, cx+6):

                index = y*width + x

                if index >= len(data):

                    continue

                if data[index] > 50:

                    return True

        return False

    #######################################################

    def control_loop(self):

        cmd = TwistStamped()

        cmd.header.stamp = self.get_clock().now().to_msg()

        cmd.twist = self.cmd.twist

        if self.obstacle_ahead():

            cmd.twist.linear.x = 0.0

            cmd.twist.linear.y = 0.0

            cmd.twist.angular.z = 0.6

            self.get_logger().warn("Obstacle Detected")

        self.pub.publish(cmd)


###########################################################

def main(args=None):

    rclpy.init(args=args)

    node = ObstacleAvoidance()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__=="__main__":

    main()