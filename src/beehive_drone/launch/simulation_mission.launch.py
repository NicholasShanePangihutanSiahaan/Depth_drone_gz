from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    config = os.path.join(get_package_share_directory('beehive_drone'), 'config', 'sim.yaml')
    pcl = Node(package='point-cloud-test', executable='pcl_proc_node', output='screen',
               remappings=[('/input_cloud', '/zed2i/depth/points'),
                           ('/odom', '/mavros/odometry/out'),
                           ('/output_cloud', '/perception/pcl/non_ground'),
                           ('/clusters', '/perception/pcl/clusters'),
                           ('/cylinders', '/perception/pcl/cylinders'),
                           ('/global/cylinders', '/global_cylinders')],
               parameters=[{'use_transform_pcl': False,
                            'min_sensor_range': 0.3,
                            'max_sensor_range': 15.0,
                            'voxel_leaf_size': 0.30}])
    return LaunchDescription([
        pcl,
        Node(package='beehive_drone', executable='tree_mapper', parameters=[config], output='screen'),
        Node(package='beehive_drone', executable='velocity_controller', output='screen'),
        Node(package='beehive_drone', executable='vortex_avoidance_controller', output='screen'),
        Node(package='beehive_drone', executable='dynamic_orbit_controller', parameters=[config], output='screen'),
        Node(package='beehive_drone', executable='flight_manager', output='screen'),
        Node(package='beehive_drone', executable='mission_safety_monitor', parameters=[config], output='screen'),
        Node(package='beehive_drone', executable='mission_analyzer', output='screen'),
        Node(package='beehive_drone', executable='mission_state_machine', parameters=[config], output='screen'),
    ])
