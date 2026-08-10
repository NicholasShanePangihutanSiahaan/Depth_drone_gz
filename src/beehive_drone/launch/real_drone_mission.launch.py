#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_depth_pointcloud = LaunchConfiguration("use_depth_pointcloud")
    use_yolo_fallback = LaunchConfiguration("use_yolo_fallback")
    use_analyzer = LaunchConfiguration("use_analyzer")
    hold_after_takeoff = LaunchConfiguration("hold_after_takeoff")
    mission_output_dir = LaunchConfiguration("mission_output_dir")
    point_cloud_topic = LaunchConfiguration("point_cloud_topic")
    rgb_topic = LaunchConfiguration("rgb_topic")
    depth_topic = LaunchConfiguration("depth_topic")
    camera_info_topic = LaunchConfiguration("camera_info_topic")
    depth_camera_info_topic = LaunchConfiguration("depth_camera_info_topic")

    mission_launch = PathJoinSubstitution(
        [FindPackageShare("beehive_drone"), "launch", "mission.launch.py"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_depth_pointcloud", default_value="true"),
            DeclareLaunchArgument("use_yolo_fallback", default_value="false"),
            DeclareLaunchArgument("use_analyzer", default_value="true"),
            DeclareLaunchArgument("hold_after_takeoff", default_value="false"),
            DeclareLaunchArgument(
                "mission_output_dir", default_value="~/beehive_mission_results_jetson"
            ),
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
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(mission_launch),
                launch_arguments={
                    "use_pcl": "true",
                    "use_yolo_fallback": use_yolo_fallback,
                    "use_analyzer": use_analyzer,
                    "hold_after_takeoff": hold_after_takeoff,
                    "use_depth_pointcloud": use_depth_pointcloud,
                    "point_cloud_topic": point_cloud_topic,
                    "rgb_topic": rgb_topic,
                    "depth_topic": depth_topic,
                    "camera_info_topic": camera_info_topic,
                    "depth_camera_info_topic": depth_camera_info_topic,
                    "mission_output_dir": mission_output_dir,
                }.items(),
            )
        ]
    )
