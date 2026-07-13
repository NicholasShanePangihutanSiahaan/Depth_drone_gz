#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool

from mavros_msgs.srv import (
    CommandBool,
    SetMode,
    CommandTOL
)

from mavros_msgs.msg import State

from geometry_msgs.msg import PoseStamped


class FlightManager(Node):

    def __init__(self):

        super().__init__(
            "flight_manager"
        )


        self.current_state = None

        self.current_alt = 0.0


        self.arm_done = False
        self.takeoff_done = False



        #
        # Subscriber
        #

        self.state_sub = self.create_subscription(
            State,
            "/mavros/state",
            self.state_callback,
            10
        )


        self.pose_sub = self.create_subscription(
            PoseStamped,
            "/mavros/local_position/pose",
            self.pose_callback,
            10
        )


        self.command_sub = self.create_subscription(
            Bool,
            "/mission/takeoff",
            self.takeoff_callback,
            10
        )


        #
        # MAVROS service
        #

        self.arm_client = self.create_client(
            CommandBool,
            "/mavros/cmd/arming"
        )


        self.mode_client = self.create_client(
            SetMode,
            "/mavros/set_mode"
        )


        self.takeoff_client = self.create_client(
            CommandTOL,
            "/mavros/cmd/takeoff"
        )


        self.timer = self.create_timer(
            1.0,
            self.loop
        )


        self.request_takeoff=False


        self.get_logger().info(
            "Flight Manager Started"
        )



    def state_callback(self,msg):

        self.current_state=msg



    def pose_callback(self,msg):

        self.current_alt = msg.pose.position.z



    def takeoff_callback(self,msg):

        if msg.data:

            self.request_takeoff=True

            self.get_logger().info(
                "Takeoff requested"
            )



    def set_guided(self):

        if not self.current_state:
            return


        if self.current_state.mode != "GUIDED":

            req=SetMode.Request()

            req.custom_mode="GUIDED"


            self.mode_client.call_async(req)


            self.get_logger().info(
                "GUIDED mode requested"
            )



    def arm(self):

        if self.arm_done:
            return


        req=CommandBool.Request()

        req.value=True


        future=self.arm_client.call_async(req)


        future.add_done_callback(
            self.arm_callback
        )



    def arm_callback(self,future):

        result=future.result()


        if result.success:

            self.arm_done=True

            self.get_logger().info(
                "ARM SUCCESS"
            )



    def takeoff(self):

        if self.takeoff_done:
            return


        req=CommandTOL.Request()

        req.altitude=5.0


        future=self.takeoff_client.call_async(req)


        future.add_done_callback(
            self.takeoff_callback_done
        )



    def takeoff_callback_done(self,future):

        result=future.result()


        if result.success:

            self.takeoff_done=True

            self.get_logger().info(
                "TAKEOFF SUCCESS"
            )



    def loop(self):

        if not self.request_takeoff:
            return


        if not self.current_state:
            return


        #
        # Step 1 GUIDED
        #

        if self.current_state.mode != "GUIDED":

            self.set_guided()

            return



        #
        # Step 2 ARM
        #

        if not self.current_state.armed:

            self.arm()

            return



        #
        # Step 3 TAKEOFF
        #

        if not self.takeoff_done:

            self.takeoff()

            return



        self.get_logger().info(
            f"HOVER ALT={self.current_alt:.2f}"
        )



def main(args=None):

    rclpy.init(args=args)

    node=FlightManager()


    try:

        rclpy.spin(node)

    except KeyboardInterrupt:
        pass


    node.destroy_node()

    rclpy.shutdown()



if __name__=="__main__":

    main()