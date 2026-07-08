#!/usr/bin/env python3

import rclpy

from rclpy.node import Node

from std_msgs.msg import Bool

from mavros_msgs.srv import CommandTOL


class LandingController(Node):

    def __init__(self):

        super().__init__("landing_controller")

        self.create_subscription(
            Bool,
            "/mission/land",
            self.land_callback,
            10
        )

        self.land_client = self.create_client(
            CommandTOL,
            "/mavros/cmd/land"
        )

        while not self.land_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for MAVROS landing service...")

        self.get_logger().info("Landing Controller Started")

    def land_callback(self, msg):

        if not msg.data:
            return

        self.get_logger().info("Landing requested")

        req = CommandTOL.Request()

        req.min_pitch = 0.0
        req.yaw = 0.0
        req.latitude = 0.0
        req.longitude = 0.0
        req.altitude = 0.0

        future = self.land_client.call_async(req)

        future.add_done_callback(self.land_response)

    def land_response(self, future):

        try:

            result = future.result()

            if result.success:
                self.get_logger().info("Landing command accepted")
            else:
                self.get_logger().warn("Landing command rejected")

        except Exception as e:

            self.get_logger().error(f"Landing failed : {e}")


def main(args=None):

    rclpy.init(args=args)

    node = LandingController()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == "__main__":

    main()