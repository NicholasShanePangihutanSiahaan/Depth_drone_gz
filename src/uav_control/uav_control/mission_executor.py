#!/usr/bin/env python3


import rclpy

from rclpy.node import Node


from std_msgs.msg import String
from std_msgs.msg import Bool


from geometry_msgs.msg import PoseStamped



class MissionExecutor(Node):

    def __init__(self):

        super().__init__(
            "mission_executor"
        )


        ##################################################
        # State
        ##################################################

        self.state = "INIT"


        self.mission_started = False

        self.mission_finished = False


        ##################################################
        # UAV State
        ##################################################

        self.current_x = 0.0
        self.current_y = 0.0
        self.current_z = 0.0



        ##################################################
        # Subscribers
        ##################################################


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



        ##################################################
        # Publishers
        ##################################################


        self.takeoff_pub = self.create_publisher(
            Bool,
            "/mission/takeoff",
            10
        )


        self.land_pub = self.create_publisher(
            Bool,
            "/mission/land",
            10
        )


        self.return_home_pub = self.create_publisher(
            Bool,
            "/mission/return_home",
            10
        )


        self.mission_pub = self.create_publisher(
            String,
            "/mission/executor_status",
            10
        )



        ##################################################

        self.timer = self.create_timer(
            1.0,
            self.loop
        )


        self.get_logger().info(
            "Mission Executor Ready"
        )



    ##################################################
    # Callbacks
    ##################################################


    def status_callback(self,msg):

        status = msg.data


        self.get_logger().info(
            f"Mission status : {status}"
        )


        #############################################
        # Mission start
        #############################################

        if "INSPECT_TREE" in status:

            self.mission_started=True

            self.state="INSPECTION"



        #############################################
        # Mission finished
        #############################################

        if status=="MISSION_FINISHED":

            self.state="RETURN_HOME"



    def pose_callback(self,msg):

        self.current_x = (
            msg.pose.position.x
        )

        self.current_y = (
            msg.pose.position.y
        )

        self.current_z = (
            msg.pose.position.z
        )



    ##################################################
    # Main supervisor
    ##################################################


    def loop(self):


        msg=String()


        #############################################

        if self.state=="INIT":


            msg.data="WAITING_FOR_MISSION"


        #############################################

        elif self.state=="INSPECTION":


            msg.data="INSPECTING"


        #############################################

        elif self.state=="RETURN_HOME":


            self.return_home()


            msg.data="RETURN_HOME"


            self.state="LAND"



        #############################################

        elif self.state=="LAND":


            self.land()


            msg.data="LANDING"


            self.state="FINISHED"



        #############################################

        elif self.state=="FINISHED":


            msg.data="MISSION_COMPLETE"



        self.mission_pub.publish(msg)



    ##################################################
    # Commands
    ##################################################


    def return_home(self):


        cmd=Bool()

        cmd.data=True


        self.return_home_pub.publish(cmd)


        self.get_logger().info(
            "Return home command sent"
        )



    def land(self):


        cmd=Bool()

        cmd.data=True


        self.land_pub.publish(cmd)


        self.get_logger().info(
            "Landing command sent"
        )




##################################################


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