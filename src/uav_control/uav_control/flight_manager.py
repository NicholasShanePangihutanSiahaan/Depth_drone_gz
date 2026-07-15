#!/usr/bin/env python3

from unittest import result

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_msgs.msg import Bool

from mavros_msgs.srv import (
    CommandBool,
    SetMode,
    CommandTOL
)

from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from rclpy.qos import HistoryPolicy
from mavros_msgs.msg import State

from geometry_msgs.msg import PoseStamped
sensor_qos = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=10
)

class FlightManager(Node):


    def __init__(self):
        self.current_alt = 0.0
        self.takeoff_requested = False
        self.arm_requested = False
        super().__init__(
            "flight_manager"
        )

        self.flight_state = "IDLE"
        self.arm_request_pending = False
        self.takeoff_altitude = 4.0

        self.hover_tolerance = 0.2

        self.request_land = False
        self.current_state = None

        self.current_alt = 0.0

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
            sensor_qos
        )


        self.command_sub = self.create_subscription(
            Bool,
            "/mission/takeoff",
            self.takeoff_callback,
            10
        )
        self.land_sub = self.create_subscription(
            Bool,
            "/mission/land",
            self.land_callback,
            10
        )
        
        
        self.flight_status_pub = self.create_publisher(
            String,
            "/flight/status",
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


        self.flight_command = "NONE"


        self.get_logger().info(
            "Flight Manager Started"
        )



    def state_callback(self,msg):

        self.current_state=msg



    def pose_callback(self,msg):

        self.current_alt = msg.pose.position.z

    def land_callback(self, msg):

        if msg.data:

            self.flight_command = "LAND"

    def publish_status(self,status):

        msg = String()

        msg.data = status

        self.flight_status_pub.publish(msg)
    def takeoff_callback(self,msg):

        if msg.data:
            self.flight_command = "TAKEOFF"

            self.get_logger().info(
                "Mission requested TAKEOFF"
            )



    def set_guided(self):

        if not self.current_state:
            return


        if self.current_state.mode != "GUIDED":

            req=SetMode.Request()

            req.custom_mode="GUIDED"

            if not self.mode_client.wait_for_service(timeout_sec=1.0):

                return

            self.mode_client.call_async(req)


            self.get_logger().info(
                "GUIDED mode requested"
            )



    def arm(self):

        if self.current_state.armed:
            return


        req=CommandBool.Request()

        req.value=True

        self.get_logger().info("Calling ARM service")

        if not self.arm_client.wait_for_service(timeout_sec=1.0):
        
            return

        future=self.arm_client.call_async(req)


        future.add_done_callback(
            self.arm_callback
        )

    def arm_callback(self, future):
 
        self.arm_request_pending=False
        try:

            result = future.result()

            if result.success:

                self.get_logger().info(f"Arm response = {result.success}")
            
            else:

                self.arm_requested = False

        except Exception as e:

            self.arm_requested = False
            self.get_logger().error(str(e))

    def reached_hover(self):

        return (
            abs(
                self.current_alt -
                self.takeoff_altitude
            ) < self.hover_tolerance
        )

    def takeoff(self):

        req=CommandTOL.Request()

        req.altitude = self.takeoff_altitude

        self.get_logger().info(
                f"Calling TAKEOFF altitude={req.altitude}"
            )


        if not self.takeoff_client.wait_for_service(timeout_sec=1.0):
        
            return
        future=self.takeoff_client.call_async(req)


        future.add_done_callback(
            self.takeoff_callback_done
        )

    def takeoff_callback_done(self,future):

        try:

            result = future.result()

            self.get_logger().info(
                f"TAKEOFF response success={result.success} result={result.result}"
            )
            if result.success:

                self.flight_state = "WAIT_CLIMB"

                # self.get_logger().info("Takeoff accepted")

            else:

                self.takeoff_requested = False

        except Exception as e:

            self.takeoff_requested = False
            self.get_logger().error(str(e))

    def loop(self):


        self.get_logger().info(

            f"[FLIGHT] {self.flight_state}"

        )

        if self.flight_command == "NONE":
            return


        if not self.current_state:
            return

        if self.flight_state == "IDLE":

            if self.flight_command == "TAKEOFF":

                self.flight_state = "WAIT_GUIDED"

                self.publish_status("IDLE -> GUIDED")

                return
            
        elif self.flight_state == "WAIT_GUIDED":

            if self.current_state.mode != "GUIDED":

                self.set_guided()
                return

            else:

                self.publish_status("GUIDED")

                self.flight_state = "WAIT_ARM"


        elif self.flight_state == "WAIT_ARM":

            if self.current_state.armed:

                self.publish_status("ARMED")

                self.flight_state = "WAIT_TAKEOFF"

                self.arm_requested = False
                self.arm_request_pending = False


            else:

                if not self.arm_requested:

                    self.get_logger().info("Sending ARM request")
                    self.arm_request_pending = True

                    self.arm()

                    self.arm_requested = True
        
        elif self.flight_state == "WAIT_TAKEOFF":

            if not self.takeoff_requested:

                self.get_logger().info(
                    f"Sending TAKEOFF request altitude={self.takeoff_altitude}"
                )
                self.takeoff()

                self.takeoff_requested = True

        elif self.flight_state == "WAIT_CLIMB":

            self.get_logger().info(
                f"Waiting climb altitude={self.current_alt:.2f}"
            )

            if self.current_alt > 0.3:

                self.get_logger().info(
                    "Drone has started climbing"
                )

                self.flight_state = "CLIMBING"

        elif self.flight_state == "CLIMBING":

            self.get_logger().info(

                f"Altitude {self.current_alt:.2f}"

            )

            if self.reached_hover():

                self.publish_status("HOVER")

                self.get_logger().info(
                    f"Reached hover altitude {self.current_alt:.2f}"
                )

        elif self.flight_state == "HOVER":
            self.get_logger().info(
                f"HOVER ALT={self.current_alt:.2f}"
            )
            pass




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