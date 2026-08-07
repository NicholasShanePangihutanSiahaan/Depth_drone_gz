#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    share = get_package_share_directory("beehive_drone")
    base = os.path.join(share, "launch", "mission_gazebo_revised.launch.py")
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(base),
            launch_arguments={
                "mission_mode": "all",
                "hold_after_takeoff": "false",
            }.items(),
        )
    ])
