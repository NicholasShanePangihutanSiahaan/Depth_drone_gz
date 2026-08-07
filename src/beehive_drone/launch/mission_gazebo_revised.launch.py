#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = get_package_share_directory("beehive_drone")
    config_file = os.path.join(share, "config", "mission_gazebo_revised.yaml")

    use_pcl = LaunchConfiguration("use_pcl")
    use_yolo_fallback = LaunchConfiguration("use_yolo_fallback")
    use_analyzer = LaunchConfiguration("use_analyzer")
    hold_after_takeoff = LaunchConfiguration("hold_after_takeoff")
    mission_mode = LaunchConfiguration("mission_mode")
    point_cloud_topic = LaunchConfiguration("point_cloud_topic")
    odom_topic = LaunchConfiguration("odom_topic")
    yolo_model_path = LaunchConfiguration("yolo_model_path")
    yolo_device = LaunchConfiguration("yolo_device")

    return LaunchDescription([
        DeclareLaunchArgument("use_pcl", default_value="true"),
        DeclareLaunchArgument("use_yolo_fallback", default_value="false"),
        DeclareLaunchArgument("use_analyzer", default_value="true"),
        # Full one-tree mission by default. Set true for a takeoff-only test.
        DeclareLaunchArgument("hold_after_takeoff", default_value="false"),
        DeclareLaunchArgument("mission_mode", default_value="single"),
        DeclareLaunchArgument(
            "point_cloud_topic", default_value="/zed2i/depth/points"
        ),
        DeclareLaunchArgument(
            "odom_topic", default_value="/mavros/odometry/out"
        ),
        DeclareLaunchArgument("yolo_model_path", default_value=""),
        DeclareLaunchArgument("yolo_device", default_value="cpu"),

        Node(
            package="point-cloud-test",
            executable="pcl_proc_node",
            name="pcl_proc_node",
            output="screen",
            condition=IfCondition(use_pcl),
            parameters=[config_file],
            remappings=[
                ("/input_cloud", point_cloud_topic),
                ("/odom", odom_topic),
                ("/output_cloud", "/perception/pcl/filtered_cloud"),
                ("/clusters", "/perception/pcl/clusters"),
                ("/cylinders", "/perception/pcl/cylinders"),
                ("/global/cylinders", "/perception/pcl/tracked_cylinders"),
            ],
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="map_to_plantation_tf",
            output="screen",
            condition=IfCondition(use_pcl),
            arguments=[
                "--x", "0", "--y", "0", "--z", "0",
                "--roll", "0", "--pitch", "0", "--yaw", "0",
                "--frame-id", "map",
                "--child-frame-id", "plantation",
            ],
        ),
        Node(
            package="beehive_drone",
            executable="pcl_tree_mapper",
            name="pcl_tree_mapper",
            output="screen",
            condition=IfCondition(use_pcl),
            parameters=[
                config_file,
                {
                    "enable_fallback_points": ParameterValue(
                        use_yolo_fallback, value_type=bool
                    )
                },
            ],
        ),
        Node(
            package="beehive_drone",
            executable="yolo_gazebo_detector",
            name="yolo_gazebo_detector",
            output="screen",
            condition=IfCondition(use_yolo_fallback),
            parameters=[
                config_file,
                {"model_path": yolo_model_path, "device": yolo_device},
            ],
        ),
        Node(
            package="beehive_drone",
            executable="tree_localizer",
            name="tree_localizer",
            output="screen",
            condition=IfCondition(use_yolo_fallback),
            parameters=[config_file],
        ),
        Node(
            package="beehive_drone",
            executable="flight_manager",
            name="flight_manager",
            output="screen",
            parameters=[config_file],
        ),
        Node(
            package="beehive_drone",
            executable="mission_state_machine",
            name="mission_state_machine",
            output="screen",
            parameters=[
                config_file,
                {
                    "mission_mode": ParameterValue(mission_mode, value_type=str),
                    "hold_after_takeoff": ParameterValue(
                        hold_after_takeoff, value_type=bool
                    )
                },
            ],
        ),
        Node(
            package="beehive_drone",
            executable="dynamic_orbit_controller",
            name="dynamic_orbit_controller",
            output="screen",
            parameters=[config_file],
        ),
        Node(
            package="beehive_drone",
            executable="vortex_avoidance_controller",
            name="vortex_avoidance_controller",
            output="screen",
            parameters=[config_file],
        ),
        Node(
            package="beehive_drone",
            executable="velocity_controller",
            name="velocity_controller",
            output="screen",
            parameters=[config_file],
        ),
        Node(
            package="beehive_drone",
            executable="mission_analyzer",
            name="mission_analyzer",
            output="screen",
            condition=IfCondition(use_analyzer),
            parameters=[config_file],
        ),
    ])
