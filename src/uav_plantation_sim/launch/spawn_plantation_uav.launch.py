#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    pkg_share = FindPackageShare("uav_plantation_sim")
    model_file = PathJoinSubstitution(
        [pkg_share, "models", "plantation_quadrotor", "model.sdf"]
    )

    return LaunchDescription(
        [
            Node(
                package="ros_gz_sim",
                executable="create",
                name="spawn_plantation_uav",
                output="screen",
                arguments=[
                    "-name",
                    "plantation_uav",
                    "-file",
                    model_file,
                    "-x",
                    "0.0",
                    "-y",
                    "0.0",
                    "-z",
                    "1.0",
                    "-Y",
                    "0.0",
                ],
            )
        ]
    )
