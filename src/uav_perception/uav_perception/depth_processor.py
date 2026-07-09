#!/usr/bin/env python3


import rclpy

from rclpy.node import Node

from sensor_msgs.msg import Image

from geometry_msgs.msg import Point

from cv_bridge import CvBridge



import numpy as np



class DepthProcessor(Node):


    def __init__(self):

        super().__init__(
            "depth_processor"
        )


        self.bridge=CvBridge()


        self.depth=None


        self.pixel=None



        self.depth_sub=self.create_subscription(
            Image,
            "/plantation_uav/zed2i/depth",
            self.depth_callback,
            10
        )


        self.pixel_sub=self.create_subscription(
            Point,
            "/perception/tree_pixel",
            self.pixel_callback,
            10
        )


        self.pub=self.create_publisher(
            Point,
            "/perception/tree_distance",
            10
        )



    def depth_callback(self,msg):

        self.depth=self.bridge.imgmsg_to_cv2(
            msg,
            desired_encoding="32FC1"
        )


        self.process()



    def pixel_callback(self,msg):

        self.pixel=msg


    def process(self):


        if self.depth is None:
            return


        if self.pixel is None:
            return



        u=int(self.pixel.x)
        v=int(self.pixel.y)



        if v>=self.depth.shape[0]:
            return

        if u>=self.depth.shape[1]:
            return



        distance=self.depth[v,u]



        if np.isnan(distance):
            return



        out=Point()

        out.z=float(distance)

        self.pub.publish(out)



def main(args=None):

    rclpy.init(args=args)

    node=DepthProcessor()

    rclpy.spin(node)


if __name__=="__main__":
    main()