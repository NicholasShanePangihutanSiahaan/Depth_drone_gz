#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from std_msgs.msg import Bool

from cv_bridge import CvBridge

import cv2
import numpy as np



class TreeDetector(Node):

    def __init__(self):

        super().__init__(
            "tree_detector"
        )


        self.bridge = CvBridge()


        self.sub = self.create_subscription(
            Image,
            "/plantation_uav/zed2i/left/image_rect_color",
            self.image_callback,
            10
        )


        self.pub = self.create_publisher(
            Point,
            "/perception/tree_pixel",
            10
        )


        self.detect_pub = self.create_publisher(
            Bool,
            "/perception/tree_detected",
            10
        )


        self.get_logger().info(
            "Tree detector running"
        )



    def image_callback(self,msg):

        image=self.bridge.imgmsg_to_cv2(
            msg,
            "bgr8"
        )


        hsv=cv2.cvtColor(
            image,
            cv2.COLOR_BGR2HSV
        )


        lower=np.array(
            [25,40,20]
        )

        upper=np.array(
            [95,255,255]
        )


        mask=cv2.inRange(
            hsv,
            lower,
            upper
        )


        contours,_=cv2.findContours(
            mask,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE
        )


        found=False


        for c in contours:


            area=cv2.contourArea(c)


            if area > 500:


                x,y,w,h=cv2.boundingRect(c)


                center_x=x+w/2
                center_y=y+h/2


                p=Point()

                p.x=center_x
                p.y=center_y
                p.z=0


                self.pub.publish(p)


                found=True

                break



        flag=Bool()

        flag.data=found


        self.detect_pub.publish(flag)




def main(args=None):

    rclpy.init(args=args)

    node=TreeDetector()

    rclpy.spin(node)


    node.destroy_node()

    rclpy.shutdown()



if __name__=="__main__":
    main()