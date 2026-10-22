# from launch import LaunchDescription
# from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
# from launch.launch_description_sources import PythonLaunchDescriptionSource
# from launch.substitutions import EnvironmentVariable, PathJoinSubstitution
# from launch_ros.actions import Node
# from launch_ros.substitutions import FindPackageShare


# def generate_launch_description():
#     pkg_share = FindPackageShare("uav_plantation_sim")
#     world_file = PathJoinSubstitution([pkg_share, "worlds", "plantation.sdf"])
#     model_file = PathJoinSubstitution([pkg_share, "models", "plantation_quadrotor", "model.sdf"])
#     bridge_config = PathJoinSubstitution([pkg_share, "config", "bridge_config.yaml"])
#     gz_launch = PathJoinSubstitution([FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"])
#     model_path = PathJoinSubstitution([pkg_share, "models"])

#     return LaunchDescription([
#         SetEnvironmentVariable(
#             name="GZ_SIM_RESOURCE_PATH",
#             value="/home/shane/ProjekAtaka/gazebo_sim/models",
#         ),

# 	SetEnvironmentVariable(
#             name="IGN_GAZEBO_RESOURCE_PATH",
#             value="/home/shane/ProjekAtaka/gazebo_sim/models",
#         ),

#         IncludeLaunchDescription(
#             PythonLaunchDescriptionSource(gz_launch),
#             launch_arguments={
#                 "gz_args": ["-r -v 4 ", world_file],
#             }.items(),
#         ),

#         Node(
#             package="ros_gz_bridge",
#             executable="parameter_bridge",
#             name="ros_gz_bridge",
#             output="screen",
#             parameters=[{
#                 "config_file": bridge_config,
#                 "qos_overrides./tf_static.publisher.durability": "transient_local",
#             }],
#         ),

#         Node(
#             package="ros_gz_sim",
#             executable="create",
#             name="spawn_plantation_uav",
#             output="screen",
#             arguments=[
#                 "-name", "plantation_uav",
#                 "-file", model_file,
#                 "-x", "0.0",
#                 "-y", "0.0",
#                 "-z", "1.0",
#                 "-Y", "0.0",
#             ],
#         ),
#     ])

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    pkg_share = FindPackageShare("uav_plantation_sim")
    world_name = LaunchConfiguration("world")
    gz_args = LaunchConfiguration("gz_args")
    spawn_x = LaunchConfiguration("spawn_x")
    spawn_y = LaunchConfiguration("spawn_y")
    spawn_z = LaunchConfiguration("spawn_z")
    world_file = PathJoinSubstitution([pkg_share, "worlds", world_name])
    model_file = PathJoinSubstitution([pkg_share, "models", "plantation_quadrotor", "model.sdf"])
    bridge_config = PathJoinSubstitution([pkg_share, "config", "bridge_config.yaml"])
    gz_launch = PathJoinSubstitution([FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"])

    return LaunchDescription([
        DeclareLaunchArgument(
            "world",
            default_value="plantation.sdf",
            description=(
                "Gazebo world file. Use plantation.sdf for the flat baseline."
            ),
        ),
        DeclareLaunchArgument(
            "gz_args",
            default_value="-r -v 4",
            description="Gazebo arguments; use '-s -r -v 3' for headless tests.",
        ),
        DeclareLaunchArgument("spawn_x", default_value="0.0"),
        DeclareLaunchArgument("spawn_y", default_value="-3.0"),
        DeclareLaunchArgument("spawn_z", default_value="0.1"),
        DeclareLaunchArgument("avoidance_fixture", default_value="false"),
        # Hanya memerlukan variabel GZ modern untuk mencari aset mesh/model
        SetEnvironmentVariable(
            name="GZ_SIM_RESOURCE_PATH",
            value=PathJoinSubstitution([pkg_share, "models"]),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gz_launch),
            launch_arguments={
                "gz_args": [gz_args, " ", world_file],
            }.items(),
        ),
        
        
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                # Command(['cat ', file]) reads the file and outputs its contents as a string
                'robot_description': Command(['cat ', model_file])
            }],
            remappings=[
                # Map standard ROS /joint_states to the bridged Gazebo topic
                ('/joint_states', '/plantation_uav/joint_states')
            ]
        ),


        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='model_to_base_link_tf',
            output='screen',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom']
        ),

        Node(
            package='uav_localization',
            executable='tf_broadcaster',
            name='tf_casting',
            output='screen',
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
                "-x", spawn_x,
                "-y", spawn_y,
                "-z", spawn_z,
                "-Y", "0.0",
            ],
        ),
        ComposableNodeContainer(
            name='depth_processor_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                ComposableNode(
                    package='depth_image_proc',
                    plugin='depth_image_proc::PointCloudXyzNode',
                    name='point_cloud_xyz_node',
                    # Remap input/output topics to match your depth camera configurations
                    remappings=[
                        ('image_rect', '/zed2i/depth/depth_registered'),
                        ('camera_info', '/zed2i/depth/camera_info'),
                        ('points', '/zed2i/depth/points')
                    ]
                )
            ],
            output='screen'
        ),
        Node(package="ros_gz_sim", executable="create", output="screen",
             name="spawn_depth_avoidance_fixture",
             arguments=["-name", "depth_avoidance_fixture", "-file",
                        PathJoinSubstitution([pkg_share, "models",
                                              "depth_avoidance_fixture", "model.sdf"])],
             condition=IfCondition(LaunchConfiguration("avoidance_fixture"))),
    ])
