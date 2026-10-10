"""Real-stack Gazebo test with two deterministic synthetic palm trees."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    stack = os.path.join(
        get_package_share_directory('beehive_drone'),
        'launch', 'real_stack_sim.launch.py')
    return LaunchDescription([
        DeclareLaunchArgument('auto_start', default_value='false'),
        DeclareLaunchArgument(
            'tree_positions',
            default_value=(
                '7.0,0.0;14.0,0.0;21.0,0.0;'
                '3.5,-6.0622;10.5,-6.0622;17.5,-6.0622;'
                '3.5,6.0622;10.5,6.0622;17.5,6.0622')),
        DeclareLaunchArgument(
            'report_output_directory',
            default_value='~/beehive_mission_reports/sim_multi_tree'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(stack),
            launch_arguments={
                'auto_start': LaunchConfiguration('auto_start'),
                'mission_mode': 'multi_tree',
                'tree_positions': LaunchConfiguration('tree_positions'),
                'tree_x': '7.0',
                'tree_y': '0.0',
                'expected_tree_count': '2',
                'report_output_directory':
                    LaunchConfiguration('report_output_directory'),
            }.items()),
    ])
