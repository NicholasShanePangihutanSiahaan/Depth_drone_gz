#!/usr/bin/env python3


import math

import rclpy
from rclpy.node import Node


from geometry_msgs.msg import Point
from geometry_msgs.msg import PoseStamped

from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    HistoryPolicy,
    DurabilityPolicy
)

from geometry_msgs.msg import TwistStamped



class VelocityController(Node):

    def __init__(self):

        self.counter=0
        super().__init__(
            "velocity_controller"
        )

        qos_sensor = QoSProfile(

            reliability=ReliabilityPolicy.BEST_EFFORT,

            history=HistoryPolicy.KEEP_LAST,

            depth=10

        )
        ##################################################
        # Parameter
        ##################################################

        # proportional gain

        self.kp_xy = 0.8

        self.kp_z = 0.8



        # velocity limit

        self.max_velocity_xy = 3.0

        self.max_velocity_z = 1.5



        # waypoint dianggap tercapai

        self.goal_threshold = 0.5



        ##################################################
        # State
        ##################################################

        self.current_pose = None

        self.target = None



        ##################################################
        # Subscriber
        ##################################################

        self.pose_sub = self.create_subscription(
            PoseStamped,
            "/mavros/local_position/pose",
            self.pose_callback,
            qos_sensor
        )



        self.target_sub = self.create_subscription(
            Point,
            "/control/target_point",
            self.target_callback,
            10
        )



        ##################################################
        # Publisher
        ##################################################

        self.velocity_pub = self.create_publisher(
            TwistStamped,
            "/mavros/setpoint_velocity/cmd_vel",
            10
        )



        ##################################################

        self.timer = self.create_timer(
            0.05,
            self.control_loop
        )


        self.get_logger().info(
            "Velocity Controller Started"
        )



    ##################################################
    # UAV pose
    ##################################################

    def pose_callback(
        self,
        msg
    ):

        self.current_pose = msg.pose



    ##################################################
    # Target waypoint
    ##################################################

    def target_callback(
        self,
        msg
    ):

        self.target = msg



    ##################################################
    # Limit velocity
    ##################################################

    def limit(
        self,
        value,
        maximum
    ):


        if value > maximum:

            return maximum


        if value < -maximum:

            return -maximum


        return value



    ##################################################
    # Control loop
    ##################################################

    def control_loop(self):

        self.get_logger().info(
            "[CONTROL LOOP] running"
        )

        if self.current_pose is None:

            return



        if self.target is None:

            return



        ##################################################
        # Position error
        ##################################################

        ex = (
            self.target.x -
            self.current_pose.position.x
        )


        ey = (
            self.target.y -
            self.current_pose.position.y
        )


        ez = (
            self.target.z -
            self.current_pose.position.z
        )



        distance = math.sqrt(

            ex*ex+
            ey*ey+
            ez*ez

        )



        ##################################################
        # Create velocity command
        ##################################################

        cmd = TwistStamped()


        cmd.header.stamp = (
            self.get_clock()
            .now()
            .to_msg()
        )


        cmd.header.frame_id = (
            "base_link"
        )



        ##################################################
        # Stop near waypoint
        ##################################################

        if distance < self.goal_threshold:


            cmd.twist.linear.x = 0.0

            cmd.twist.linear.y = 0.0

            cmd.twist.linear.z = 0.0



        else:


            vx = self.kp_xy * ex

            vy = self.kp_xy * ey

            vz = self.kp_z * ez



            cmd.twist.linear.x = self.limit(
                vx,
                self.max_velocity_xy
            )


            cmd.twist.linear.y = self.limit(
                vy,
                self.max_velocity_xy
            )


            cmd.twist.linear.z = self.limit(
                vz,
                self.max_velocity_z
            )

        self.counter +=1


        if self.counter % 20 == 0:

            self.get_logger().info(
                f"[VELOCITY] "
                f"vx={cmd.twist.linear.x:.2f} "
                f"vy={cmd.twist.linear.y:.2f} "
                f"vz={cmd.twist.linear.z:.2f}"
            )

        self.velocity_pub.publish(
            cmd
        )



##################################################


def main(args=None):


    rclpy.init(args=args)


    node = VelocityController()


    try:

        rclpy.spin(node)


    except KeyboardInterrupt:

        pass



    node.destroy_node()

    rclpy.shutdown()



if __name__=="__main__":

    main()