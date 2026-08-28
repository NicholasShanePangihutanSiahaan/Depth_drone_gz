"""Run the real ROS flight stack against simulated ZED/rangefinder hardware."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def typed(name, value_type):
    return ParameterValue(LaunchConfiguration(name), value_type=value_type)


def generate_launch_description():
    beehive_share = get_package_share_directory('beehive_drone')
    pcl_share = get_package_share_directory('point-cloud-test')
    vision_launch = os.path.join(
        beehive_share, 'launch', 'vision_to_mavros.launch.py')
    bb_launch = os.path.join(pcl_share, 'launch', 'bb_proc_node.launch.py')
    mission_launch = os.path.join(
        beehive_share, 'launch', 'real_mission.launch.py')

    arguments = [
        DeclareLaunchArgument('auto_start', default_value='false'),
        DeclareLaunchArgument('tree_x', default_value='7.0'),
        DeclareLaunchArgument('tree_y', default_value='0.0'),
        DeclareLaunchArgument(
            'tree_ground_z', default_value='0.0',
            description='Use 0.35 for plantation_hilly.sdf.'),
        DeclareLaunchArgument('camera_x', default_value='0.14'),
        DeclareLaunchArgument('camera_y', default_value='0.06'),
        DeclareLaunchArgument('camera_z', default_value='0.02'),
        DeclareLaunchArgument('camera_roll', default_value='0.0'),
        DeclareLaunchArgument('camera_pitch', default_value='0.0'),
        DeclareLaunchArgument('camera_yaw', default_value='0.0'),
        DeclareLaunchArgument('position_noise_stddev', default_value='0.0'),
        DeclareLaunchArgument('dropout_every_n', default_value='0'),
        DeclareLaunchArgument(
            'report_output_directory',
            default_value='~/beehive_mission_reports/sim_real_stack'),
    ]

    zed_adapter = Node(
        package='beehive_drone',
        executable='sim_zed_adapter',
        name='sim_zed_adapter',
        output='screen',
        parameters=[{
            'tree_x': typed('tree_x', float),
            'tree_y': typed('tree_y', float),
            'tree_ground_z': typed('tree_ground_z', float),
            'camera_x': typed('camera_x', float),
            'camera_y': typed('camera_y', float),
            'camera_z': typed('camera_z', float),
            'camera_roll': typed('camera_roll', float),
            'camera_pitch': typed('camera_pitch', float),
            'camera_yaw': typed('camera_yaw', float),
            'position_noise_stddev': typed('position_noise_stddev', float),
            'dropout_every_n': typed('dropout_every_n', int),
        }],
    )
    range_adapter = Node(
        package='beehive_drone',
        executable='sim_rangefinder_bridge',
        name='sim_rangefinder_to_real_topic',
        output='screen',
        parameters=[{
            'input_topic': '/range',
            'output_topic': '/mavros/rangefinder/rangefinder',
            'frame_id': 'range_link',
        }],
    )
    validator = Node(
        package='beehive_drone',
        executable='real_stack_sim_validator',
        name='real_stack_sim_validator',
        output='screen',
        parameters=[{
            'expected_tree_x': typed('tree_x', float),
            'expected_tree_y': typed('tree_y', float),
        }],
    )

    return LaunchDescription(arguments + [
        LogInfo(msg=(
            'REAL-STACK SIM: do not run simulation_mission.launch.py or '
            'pcl_proc_node at the same time.')),
        zed_adapter,
        range_adapter,
        IncludeLaunchDescription(PythonLaunchDescriptionSource(vision_launch)),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(bb_launch)),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(mission_launch),
            launch_arguments={
                'auto_start': LaunchConfiguration('auto_start'),
                'analyzer_output_directory':
                LaunchConfiguration('report_output_directory'),
            }.items()),
        validator,
    ])
