#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from std_msgs.msg import String, Bool
from geometry_msgs.msg import PoseStamped


class MissionExecutor(Node):

    def __init__(self):

        super().__init__(
            "mission_executor"
        )


        ###################################
        # STATE
        ###################################

        self.state = "INIT"


        ###################################
        # UAV POSITION
        ###################################

        self.current_x = 0.0
        self.current_y = 0.0
        self.current_z = 0.0


        self.home_x = None
        self.home_y = None
        self.home_z = None



        ###################################
        # SUBSCRIBER
        ###################################

        self.status_sub = self.create_subscription(
            String,
            "/mission/status",
            self.status_callback,
            10
        )


        self.pose_sub = self.create_subscription(
            PoseStamped,
            "/mavros/local_position/pose",
            self.pose_callback,
            10
        )



        ###################################
        # COMMAND PUBLISHER
        ###################################

        self.takeoff_pub = self.create_publisher(
            Bool,
            "/mission/takeoff",
            10
        )


        self.return_home_pub = self.create_publisher(
            Bool,
            "/mission/return_home",
            10
        )


        self.land_pub = self.create_publisher(
            Bool,
            "/mission/land",
            10
        )


        self.executor_pub = self.create_publisher(
            String,
            "/mission/executor_status",
            10
        )



        ###################################

        self.timer = self.create_timer(
            1.0,
            self.loop
        )


        self.get_logger().info(
            "Mission Executor Started"
        )



    ###################################
    # CALLBACK
    ###################################


    def pose_callback(self,msg):

        self.current_x = msg.pose.position.x
        self.current_y = msg.pose.position.y
        self.current_z = msg.pose.position.z



        # simpan home pertama kali

        if self.home_x is None:

            self.home_x = self.current_x
            self.home_y = self.current_y
            self.home_z = self.current_z


            self.get_logger().info(
                f"Home saved : {self.home_x},{self.home_y},{self.home_z}"
            )



    def status_callback(self,msg):

        status = msg.data


        self.get_logger().info(
            f"Mission status : {status}"
        )


        if status=="START":

            self.state="TAKEOFF"



        elif status=="INSPECT_TREE":

            self.state="INSPECTION"



        elif status=="MISSION_FINISHED":

            self.state="RETURN_HOME"




    ###################################
    # MAIN FSM
    ###################################


    def loop(self):

        output = String()



        ################################

        if self.state=="INIT":

            output.data="WAITING"



        ################################

        elif self.state=="TAKEOFF":


            self.send_takeoff()


            output.data="TAKEOFF"


            # setelah command
            # lanjut inspection

            self.state="INSPECTION"



        ################################

        elif self.state=="INSPECTION":


            output.data="INSPECTING"



        ################################

        elif self.state=="RETURN_HOME":


            self.send_return_home()


            output.data="RETURN_HOME"


            self.state="LAND"



        ################################

        elif self.state=="LAND":


            self.send_land()


            output.data="LANDING"


            self.state="FINISHED"



        ################################

        elif self.state=="FINISHED":

            output.data="MISSION_COMPLETE"



        self.executor_pub.publish(output)




    ###################################
    # COMMAND
    ###################################


    def send_takeoff(self):

        msg=Bool()

        msg.data=True


        self.takeoff_pub.publish(msg)


        self.get_logger().info(
            "Takeoff command sent"
        )



    def send_return_home(self):

        msg=Bool()

        msg.data=True


        self.return_home_pub.publish(msg)


        self.get_logger().info(
            "Return home command sent"
        )



    def send_land(self):

        msg=Bool()

        msg.data=True


        self.land_pub.publish(msg)


        self.get_logger().info(
            "Landing command sent"
        )




def main(args=None):

    rclpy.init(args=args)

    node=MissionExecutor()


    try:

        rclpy.spin(node)

    except KeyboardInterrupt:

        pass


    node.destroy_node()

    rclpy.shutdown()



if __name__=="__main__":

    main()