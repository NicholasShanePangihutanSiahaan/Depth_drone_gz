#!/usr/bin/env python3

import math

import rclpy

from rclpy.node import Node

from geometry_msgs.msg import Point

from std_msgs.msg import String


class TreeMapper(Node):

    def __init__(self):

        super().__init__("tree_mapper")

        self.tree_list = []

        self.tree_id = 1

        self.tree_sub = self.create_subscription(
            Point,
            "/perception/tree_target",
            self.tree_callback,
            10
        )

        self.tree_pub = self.create_publisher(
            String,
            "/map/tree_locations",
            10
        )

        self.get_logger().info("Tree Mapper Started")

    def tree_callback(self, msg):

        x = msg.x
        y = msg.y

        duplicate = False

        for tree in self.tree_list:

            dx = x - tree["x"]
            dy = y - tree["y"]

            d = math.sqrt(dx * dx + dy * dy)

            if d < 1.0:
                duplicate = True
                break

        if duplicate:
            return

        tree = {
            "id": self.tree_id,
            "x": x,
            "y": y
        }

        self.tree_list.append(tree)

        self.tree_id += 1

        self.get_logger().info(
            f"Tree #{tree['id']} added ({tree['x']:.2f},{tree['y']:.2f})"
        )

        text = ""

        for t in self.tree_list:

            text += f"{t['id']},{t['x']:.2f},{t['y']:.2f}\n"

        msg_out = String()

        msg_out.data = text

        self.tree_pub.publish(msg_out)


def main(args=None):

    rclpy.init(args=args)

    node = TreeMapper()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == "__main__":
    main()