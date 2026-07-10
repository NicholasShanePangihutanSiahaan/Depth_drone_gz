from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    return LaunchDescription([


        Node(
            package="uav_perception",
            executable="tree_detector",
            output="screen"
        ),


        Node(
            package="uav_perception",
            executable="depth_estimator",
            output="screen"
        ),


        Node(
            package="uav_perception",
            executable="obstacle_detector",
            output="screen"
        )

    ])