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
    fcu_url = LaunchConfiguration("fcu_url")
    gcs_url = LaunchConfiguration("gcs_url")

    world_launch = PathJoinSubstitution(
        [FindPackageShare("uav_plantation_sim"), "launch", "world.launch.py"]
    )
    spawn_launch = PathJoinSubstitution(
        [FindPackageShare("uav_plantation_sim"), "launch", "spawn_plantation_uav.launch.py"]
    )
    mavros_launch = PathJoinSubstitution(
        [FindPackageShare("uav_plantation_sim"), "launch", "mavros.launch.py"]
    )
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
                "mission_output_dir", default_value="~/beehive_mission_results"
            ),
            DeclareLaunchArgument("fcu_url", default_value="udp://127.0.0.1:14551@"),
            DeclareLaunchArgument("gcs_url", default_value="udp://:14550@"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(world_launch),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(spawn_launch),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(mavros_launch),
                launch_arguments={
                    "fcu_url": fcu_url,
                    "gcs_url": gcs_url,
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(mission_launch),
                launch_arguments={
                    "use_pcl": "true",
                    "use_yolo_fallback": use_yolo_fallback,
                    "use_analyzer": use_analyzer,
                    "hold_after_takeoff": hold_after_takeoff,
                    "use_depth_pointcloud": use_depth_pointcloud,
                    "mission_output_dir": mission_output_dir,
                }.items(),
            ),
        ]
    )
