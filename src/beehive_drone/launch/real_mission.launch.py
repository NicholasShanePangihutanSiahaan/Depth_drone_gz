"""Real mission nodes.

MAVROS, ZED wrapper, ``vision_to_mavros``, and ``bb_proc_node.launch.py`` must
already be healthy. Perception is intentionally kept out of this launch so the
mission never starts a second publisher on ``/global_cylinders``.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('beehive_drone'), 'config', 'real.yaml')
    auto_start = LaunchConfiguration('auto_start')
    mission_mode = LaunchConfiguration('mission_mode')
    max_trees = LaunchConfiguration('max_trees')
    record_data = LaunchConfiguration('record_data')
    analyzer_output_directory = LaunchConfiguration(
        'analyzer_output_directory')
    return LaunchDescription([
        DeclareLaunchArgument(
            'auto_start', default_value='false',
            description=(
                'true: otomatis GUIDED/arm/takeoff setelah local pose '
                'tersedia. Aktifkan hanya setelah MAVROS, ZED, dan vision '
                'bridge sehat.')),
        DeclareLaunchArgument(
            'mission_mode', default_value='single_tree',
            description='Pilih single_tree atau multi_tree.'),
        DeclareLaunchArgument(
            'max_trees', default_value='0',
            description='Batas multi-tree; 0 berarti tanpa batas.'),
        DeclareLaunchArgument(
            'record_data', default_value='true',
            description='Jalankan recorder lapangan Jetson.'),
        DeclareLaunchArgument(
            'analyzer_output_directory',
            default_value='~/beehive_mission_reports/real',
            description='Mission analyzer output directory.'),
        # /global_cylinders berasal dari bb_pcl_proc_node yang dijalankan
        # terpisah setelah ZED object detection sehat.
        Node(package='beehive_drone', executable='tree_mapper',
             parameters=[config], output='screen'),
        Node(package='beehive_drone',
             executable='vortex_avoidance_controller', output='screen'),
        Node(package='beehive_drone',
             executable='dynamic_orbit_controller',
             parameters=[config], output='screen'),
        Node(package='beehive_drone',
             executable='position_setpoint_controller',
             parameters=[config], output='screen'),
        Node(package='beehive_drone', executable='flight_manager',
             parameters=[config], output='screen'),
        Node(package='beehive_drone', executable='mission_safety_monitor',
             parameters=[config], output='screen'),
        Node(package='beehive_drone', executable='mission_analyzer',
             parameters=[config, {
                 'output_directory': analyzer_output_directory}],
             output='screen'),
        Node(package='beehive_drone', executable='mission_state_machine',
             parameters=[config, {
                 'auto_start': ParameterValue(auto_start, value_type=bool),
                 'mission_mode': mission_mode,
                 'max_trees': ParameterValue(max_trees, value_type=int),
             }], output='screen'),
        ExecuteProcess(
            cmd=['/bin/bash', '/home/palmbee1/DTETI-WS/data/record_scripts.sh', '/home/palmbee1/DTETI-WS/data/'],
            output='screen',
            condition=IfCondition(record_data),
        )
    ])
