#!/usr/bin/env python3


import rclpy

from rclpy.node import Node


from sensor_msgs.msg import Image

from std_msgs.msg import Bool

from cv_bridge import CvBridge

import numpy as np



class ObstacleDetector(Node):


    def __init__(self):

        super().__init__(
            "obstacle_detector"
        )


        self.bridge=CvBridge()


        self.sub=self.create_subscription(
            Image,
            "/plantation_uav/zed2i/depth",
            self.callback,
            10
        )


        self.pub=self.create_publisher(
            Bool,
            "/perception/obstacle",
            10
        )




    def callback(self,msg):


        depth=self.bridge.imgmsg_to_cv2(
            msg,
            "32FC1"
        )



        center=depth[
            200:280,
            250:390
        ]



        minimum=np.nanmin(center)



        obstacle=Bool()


        obstacle.data = minimum < 3.0


        self.pub.publish(obstacle)



def main(args=None):

    rclpy.init(args=args)

    node=ObstacleDetector()

    rclpy.spin(node)


if __name__=="__main__":
    main()