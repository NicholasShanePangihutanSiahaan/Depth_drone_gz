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
    package_share = get_package_share_directory("beehive_drone")
    config_file = os.path.join(package_share, "config", "mission.yaml")

    use_pcl = LaunchConfiguration("use_pcl")
    use_yolo_fallback = LaunchConfiguration("use_yolo_fallback")
    use_analyzer = LaunchConfiguration("use_analyzer")
    hold_after_takeoff = LaunchConfiguration("hold_after_takeoff")
    mission_mode = LaunchConfiguration("mission_mode")
    use_depth_pointcloud = LaunchConfiguration("use_depth_pointcloud")
    point_cloud_topic = LaunchConfiguration("point_cloud_topic")
    rgb_topic = LaunchConfiguration("rgb_topic")
    depth_topic = LaunchConfiguration("depth_topic")
    camera_info_topic = LaunchConfiguration("camera_info_topic")
    depth_camera_info_topic = LaunchConfiguration("depth_camera_info_topic")
    odom_topic = LaunchConfiguration("odom_topic")
    mission_output_dir = LaunchConfiguration("mission_output_dir")
    yolo_model_path = LaunchConfiguration("yolo_model_path")
    yolo_device = LaunchConfiguration("yolo_device")

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_pcl", default_value="true"),
            DeclareLaunchArgument("use_yolo_fallback", default_value="false"),
            DeclareLaunchArgument("use_analyzer", default_value="true"),
            DeclareLaunchArgument("hold_after_takeoff", default_value="false"),
            # plantation_sim already bridges /zed2i/depth/points. Enabling both
            # publishers creates duplicate clouds and unstable PCL tracks.
            DeclareLaunchArgument("use_depth_pointcloud", default_value="false"),
            DeclareLaunchArgument("mission_mode", default_value="single"),
            DeclareLaunchArgument(
                "point_cloud_topic", default_value="/zed2i/depth/points"
            ),
            DeclareLaunchArgument("rgb_topic", default_value="/zed2i/left/image_rect_color"),
            DeclareLaunchArgument("depth_topic", default_value="/zed2i/depth/depth_registered"),
            DeclareLaunchArgument(
                "camera_info_topic", default_value="/zed2i/left/camera_info"
            ),
            DeclareLaunchArgument(
                "depth_camera_info_topic", default_value="/zed2i/depth/camera_info"
            ),
            DeclareLaunchArgument("odom_topic", default_value="/mavros/odometry/out"),
            DeclareLaunchArgument(
                "mission_output_dir", default_value="~/beehive_mission_results"
            ),
            DeclareLaunchArgument("yolo_model_path", default_value=""),
            DeclareLaunchArgument("yolo_device", default_value="cpu"),

            Node(
                package="depth_image_proc",
                executable="point_cloud_xyz_node",
                name="depth_to_pointcloud",
                output="screen",
                condition=IfCondition(use_depth_pointcloud),
                remappings=[
                    ("image_rect", depth_topic),
                    ("camera_info", depth_camera_info_topic),
                    ("points", point_cloud_topic),
                ],
            ),
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
                    "--x",
                    "0",
                    "--y",
                    "0",
                    "--z",
                    "0",
                    "--roll",
                    "0",
                    "--pitch",
                    "0",
                    "--yaw",
                    "0",
                    "--frame-id",
                    "map",
                    "--child-frame-id",
                    "plantation",
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
                    {
                        "model_path": yolo_model_path,
                        "device": yolo_device,
                        "rgb_topic": rgb_topic,
                        "depth_topic": depth_topic,
                        "camera_info_topic": camera_info_topic,
                    },
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
                parameters=[
                    config_file,
                    {"output_dir": mission_output_dir},
                ],
            ),
        ]
    )
