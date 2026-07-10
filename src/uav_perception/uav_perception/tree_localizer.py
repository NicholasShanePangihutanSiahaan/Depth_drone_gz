#!/usr/bin/env python3


import rclpy

from rclpy.node import Node


from geometry_msgs.msg import Point


class TreeLocalizer(Node):


    def __init__(self):

        super().__init__(
            "tree_localizer"
        )


        self.pixel=None
        self.depth=None



        self.sub1=self.create_subscription(
            Point,
            "/perception/tree_pixel",
            self.pixel_callback,
            10
        )


        self.sub2=self.create_subscription(
            Point,
            "/perception/tree_distance",
            self.depth_callback,
            10
        )



        self.pub=self.create_publisher(
            Point,
            "/perception/tree_position_camera",
            10
        )



    def pixel_callback(self,msg):

        self.pixel=msg


        self.calculate()



    def depth_callback(self,msg):

        self.depth=msg.z


        self.calculate()



    def calculate(self):


        if self.pixel is None:
            return


        if self.depth is None:
            return



        fx = 381.3611502479812
        fy = 381.3611502479812

        cx = 320.0
        cy = 240.0



        Z=self.depth


        X=(self.pixel.x-cx)*Z/fx

        Y=(self.pixel.y-cy)*Z/fy



        point=Point()

        point.x=X
        point.y=Y
        point.z=Z


        self.pub.publish(point)



def main(args=None):

    rclpy.init(args=args)

    node=TreeLocalizer()

    rclpy.spin(node)


if __name__=="__main__":
    main()