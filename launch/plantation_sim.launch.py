from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare("uav_plantation_sim")
    world_file = PathJoinSubstitution([pkg_share, "worlds", "plantation.sdf"])
    model_file = PathJoinSubstitution([pkg_share, "models", "plantation_quadrotor", "model.sdf"])
    bridge_config = PathJoinSubstitution([pkg_share, "config", "bridge_config.yaml"])
    gz_launch = PathJoinSubstitution([FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"])
    model_path = PathJoinSubstitution([pkg_share, "models"])

    return LaunchDescription([
        SetEnvironmentVariable(
            name="GZ_SIM_RESOURCE_PATH",
            value=[model_path, ":", EnvironmentVariable("GZ_SIM_RESOURCE_PATH", default_value="")],
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gz_launch),
            launch_arguments={
                "gz_args": ["-r -v 4 ", world_file],
            }.items(),
        ),

        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            name="ros_gz_bridge",
            output="screen",
            parameters=[{
                "config_file": bridge_config,
                "qos_overrides./tf_static.publisher.durability": "transient_local",
            }],
        ),

        Node(
            package="ros_gz_sim",
            executable="create",
            name="spawn_plantation_uav",
            output="screen",
            arguments=[
                "-name", "plantation_uav",
                "-file", model_file,
                "-x", "0.0",
                "-y", "0.0",
                "-z", "1.0",
                "-Y", "0.0",
            ],
        ),
    ])
