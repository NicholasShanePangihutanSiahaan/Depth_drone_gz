"""Run the real mission stack with the multi-tree state machine enabled."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    real_mission = os.path.join(
        get_package_share_directory('beehive_drone'),
        'launch', 'real_mission.launch.py')
    return LaunchDescription([
        DeclareLaunchArgument('auto_start', default_value='false'),
        DeclareLaunchArgument(
            'analyzer_output_directory',
            default_value='~/beehive_mission_reports/real_multi_tree'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(real_mission),
            launch_arguments={
                'auto_start': LaunchConfiguration('auto_start'),
                'mission_mode': 'multi_tree',
                'analyzer_output_directory':
                    LaunchConfiguration('analyzer_output_directory'),
            }.items()),
    ])
