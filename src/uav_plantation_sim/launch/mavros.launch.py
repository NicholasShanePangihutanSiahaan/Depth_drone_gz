#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    fcu_url = LaunchConfiguration("fcu_url")
    gcs_url = LaunchConfiguration("gcs_url")
    system_id = LaunchConfiguration("system_id")
    component_id = LaunchConfiguration("component_id")
    target_system_id = LaunchConfiguration("target_system_id")
    target_component_id = LaunchConfiguration("target_component_id")

    return LaunchDescription(
        [
            DeclareLaunchArgument("fcu_url", default_value="udp://127.0.0.1:14551@"),
            DeclareLaunchArgument("gcs_url", default_value="udp://:14550@"),
            DeclareLaunchArgument("system_id", default_value="1"),
            DeclareLaunchArgument("component_id", default_value="1"),
            DeclareLaunchArgument("target_system_id", default_value="1"),
            DeclareLaunchArgument("target_component_id", default_value="1"),
            Node(
                package="mavros",
                executable="mavros_node",
                name="mavros",
                output="screen",
                parameters=[
                    {
                        "fcu_url": fcu_url,
                        "gcs_url": gcs_url,
                        "system_id": system_id,
                        "component_id": component_id,
                        "target_system_id": target_system_id,
                        "target_component_id": target_component_id,
                    }
                ],
            ),
        ]
    )
