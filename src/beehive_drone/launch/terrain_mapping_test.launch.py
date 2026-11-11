"""Isolated straight terrain test: sensor-based mapping, no orbit FSM."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config=os.path.join(get_package_share_directory('beehive_drone'),'config','sim.yaml')
    remap=[('/mavros/local_position/pose','/simulation/local_position/pose')]
    return LaunchDescription([
        Node(package='beehive_drone',executable='sim_external_odometry'),
        Node(package='beehive_drone',executable='sim_rangefinder_bridge',parameters=[config]),
        Node(package='beehive_drone',executable='position_setpoint_controller',parameters=[config],remappings=remap),
        Node(package='point-cloud-test',executable='pcl_proc_node',parameters=[config,{'use_transform_pcl':True}],remappings=[('/input_cloud','/zed2i/depth/points'),('/odom','/simulation/ground_truth/odom'),('/global/cylinders','/global_cylinders')]),
        # Fixed-world-Z base rejection is invalid on hills. Disable only that gate;
        # retain radius, height, confidence and repeated-observation gates.
        Node(package='beehive_drone',executable='tree_mapper',parameters=[config,{'max_trunk_base_height':1000.0}]),
        Node(package='beehive_drone',executable='terrain_mapping_recorder',parameters=[{'use_sim_time':True}]),
        Node(package='beehive_drone',executable='terrain_straight_test',output='screen'),
    ])
