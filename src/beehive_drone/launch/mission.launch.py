from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg_name = 'beehive_drone'

    # 1. Point-cloud perception & mapping. Cylinder fitting replaces the
    # OpenCV detector, while the original map/FSM contract stays unchanged.
    cloud_topic_arg = DeclareLaunchArgument(
        'input_cloud_topic',
        default_value='/zed2i/depth/points'
    )
    pose_topic_arg = DeclareLaunchArgument(
        'pose_topic',
        default_value='/mavros/local_position/pose'
    )
    optical_frame_arg = DeclareLaunchArgument(
        'input_is_optical_frame',
        # Gazebo RGB-D PointCloudPacked already uses X-forward sensor axes.
        # Set this to true explicitly when using a physical ZED optical cloud.
        default_value='false'
    )
    ground_truth_world_arg = DeclareLaunchArgument(
        'ground_truth_world',
        default_value=PathJoinSubstitution([
            FindPackageShare('uav_plantation_sim'), 'worlds', 'plantation.sdf'
        ]),
        description='SDF world used to reject and measure simulated ghost trees'
    )

    pcl_detector_node = Node(
        package='point-cloud-test',
        executable='pcl_proc_node',
        name='palm_pcl_detector',
        output='screen',
        parameters=[{
            'input_is_optical_frame': LaunchConfiguration(
                'input_is_optical_frame'
            )
        }],
        remappings=[
            ('/input_cloud', LaunchConfiguration('input_cloud_topic')),
            ('/pose', LaunchConfiguration('pose_topic')),
            ('/global/cylinders', '/perception/tracked_trees'),
        ]
    )

    mapper_node = Node(
        package=pkg_name,
        executable='tree_mapper',
        name='tree_mapper',
        output='screen',
        parameters=[{
            'ground_truth_world': LaunchConfiguration('ground_truth_world')
        }]
    )

    # 2. Navigation & Control Nodes
    velocity_node = Node(
        package=pkg_name,
        executable='velocity_controller',
        name='velocity_controller',
        output='screen'
    )

    vortex_node = Node(
        package=pkg_name,
        executable='vortex_avoidance_controller',
        name='vortex_avoidance_controller',
        output='screen'
    )

    orbit_node = Node(
        package=pkg_name,
        executable='dynamic_orbit_controller',
        name='dynamic_orbit_controller',
        output='screen'
    )

    # 3. Hardware Abstraction Layer (HAL) & Analyzer
    # INI YANG SEBELUMNYA HILANG
    flight_manager_node = Node(
        package=pkg_name,
        executable='flight_manager',
        name='flight_manager',
        output='screen'
    )
    
    analyzer_node = Node(
        package=pkg_name,
        executable='mission_analyzer',
        name='mission_analyzer',
        output='screen'
    )

    # 4. The Brain (FSM)
    fsm_node = Node(
        package=pkg_name,
        executable='mission_state_machine',
        name='mission_state_machine',
        output='screen'
    )

    return LaunchDescription([
        cloud_topic_arg,
        pose_topic_arg,
        optical_frame_arg,
        ground_truth_world_arg,
        pcl_detector_node,
        mapper_node,
        velocity_node,
        vortex_node,
        orbit_node,
        flight_manager_node,
        analyzer_node,
        fsm_node
    ])
