#!/usr/bin/env python3

import rclpy

from rclpy.node import Node

from nav_msgs.msg import Odometry

from tf2_ros import TransformBroadcaster

from geometry_msgs.msg import TransformStamped


class TFBroadcaster(Node):

    def __init__(self):

        super().__init__("tf_broadcaster")

        self.br = TransformBroadcaster(self)

        self.create_subscription(
            Odometry,
            "/localization/odom",
            self.callback,
            10
        )

    def callback(self,msg):

        t=TransformStamped()

        t.header=msg.header

        t.header.frame_id="odom"

        t.child_frame_id="base_link"

        t.transform.translation.x=msg.pose.pose.position.x
        t.transform.translation.y=msg.pose.pose.position.y
        t.transform.translation.z=msg.pose.pose.position.z

        t.transform.rotation=msg.pose.pose.orientation

        self.br.sendTransform(t)


def main():

    rclpy.init()

    node=TFBroadcaster()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__=="__main__":
    main()