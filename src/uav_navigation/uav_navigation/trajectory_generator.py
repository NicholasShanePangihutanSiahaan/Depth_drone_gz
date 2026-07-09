#!/usr/bin/env python3


import math

import rclpy
from rclpy.node import Node


from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseArray


from nav_msgs.msg import Path



class TrajectoryGenerator(Node):

    def __init__(self):

        super().__init__(
            "trajectory_generator"
        )


        ##################################################
        # Parameter
        ##################################################

        # jarak antar interpolasi waypoint

        self.step_distance = 0.3



        ##################################################
        # Subscriber
        ##################################################

        self.waypoint_sub = self.create_subscription(
            PoseArray,
            "/mission/inspection_waypoints",
            self.waypoint_callback,
            10
        )



        ##################################################
        # Publisher
        ##################################################

        self.trajectory_pub = self.create_publisher(
            Path,
            "/navigation/trajectory",
            10
        )



        self.get_logger().info(
            "Trajectory Generator Started"
        )



    ##################################################
    # Receive waypoint list
    ##################################################

    def waypoint_callback(
        self,
        msg
    ):


        if len(msg.poses)==0:

            self.get_logger().warning(
                "Empty waypoint received"
            )

            return



        trajectory = self.generate_path(
            msg
        )


        self.trajectory_pub.publish(
            trajectory
        )


        self.get_logger().info(
            f"Trajectory generated "
            f"{len(trajectory.poses)} points"
        )



    ##################################################
    # Generate smooth path
    ##################################################

    def generate_path(
        self,
        waypoint_msg
    ):


        path = Path()


        path.header.frame_id="odom"

        path.header.stamp=(
            self.get_clock()
            .now()
            .to_msg()
        )



        poses = waypoint_msg.poses



        ##################################################
        # Interpolate each segment
        ##################################################

        for i in range(
            len(poses)-1
        ):


            start = poses[i]

            end = poses[i+1]



            dx = (
                end.position.x -
                start.position.x
            )


            dy = (
                end.position.y -
                start.position.y
            )


            dz = (
                end.position.z -
                start.position.z
            )



            distance = math.sqrt(

                dx*dx+
                dy*dy+
                dz*dz

            )



            steps = max(
                1,
                int(
                    distance /
                    self.step_distance
                )
            )



            yaw = math.atan2(
                dy,
                dx
            )


            qz = math.sin(
                yaw/2.0
            )

            qw = math.cos(
                yaw/2.0
            )



            for j in range(
                steps
            ):


                ratio = (
                    j /
                    float(steps)
                )


                pose = PoseStamped()


                pose.header.frame_id="odom"


                pose.pose.position.x=(

                    start.position.x
                    +
                    ratio*dx

                )


                pose.pose.position.y=(

                    start.position.y
                    +
                    ratio*dy

                )


                pose.pose.position.z=(

                    start.position.z
                    +
                    ratio*dz

                )


                pose.pose.orientation.z=qz

                pose.pose.orientation.w=qw



                path.poses.append(
                    pose
                )



        ##################################################
        # Add last point
        ##################################################

        last = poses[-1]


        final_pose = PoseStamped()


        final_pose.header.frame_id="odom"


        final_pose.pose.position.x = (
            last.position.x
        )

        final_pose.pose.position.y = (
            last.position.y
        )

        final_pose.pose.position.z = (
            last.position.z
        )


        final_pose.pose.orientation.w=1.0



        path.poses.append(
            final_pose
        )



        return path





def main(args=None):


    rclpy.init(args=args)


    node=TrajectoryGenerator()


    try:

        rclpy.spin(node)


    except KeyboardInterrupt:

        pass



    node.destroy_node()


    rclpy.shutdown()



if __name__=="__main__":

    main()