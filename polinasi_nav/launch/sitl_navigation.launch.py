"""ROS side of the isolated loopback-only Gazebo/SITL simulation."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


def nodes(context):
    share = get_package_share_directory('polinasi_nav')
    mode = LaunchConfiguration('localisation').perform(context)
    if mode not in ('ground_truth', 'sensor'):
        raise ValueError('localisation must be ground_truth or sensor')
    mission = LaunchConfiguration('mission').perform(context)
    if mission not in ('inspection', 'mapping', 'identification'):
        raise ValueError('mission must be inspection, mapping, or identification')
    isolated = mission in ('mapping', 'identification')
    if mission == 'identification' and mode != 'ground_truth':
        raise ValueError('identification is simulation ground_truth only')
    params = {'use_sim_time': True, 'config_file': LaunchConfiguration('config').perform(context)}
    if LaunchConfiguration('autostart').perform(context) not in ('true', 'false'):
        raise ValueError('autostart must be true or false')
    configuration_nodes = [Node(package='polinasi_nav', executable='mavros_configurator',
                                parameters=[{'use_sim_time': True}], output='screen')] if isolated else []
    if mission == 'mapping':
        configuration_nodes.append(Node(package='polinasi_nav', executable='mapping_io',
            parameters=[params, {'mode': mode,
                'map_log_dir': LaunchConfiguration('map_log_dir').perform(context)}], output='screen'))
    if mission == 'identification':
        configuration_nodes.append(Node(package='polinasi_nav', executable='identification_recorder',
            parameters=[params, {'output_dir': os.path.dirname(LaunchConfiguration('map_log_dir').perform(context))}], output='screen'))
    acquisition_nodes = [] if mission == 'identification' else [
        Node(package='polinasi_nav', executable='sensor_gate', parameters=[params, {
             'drop_after': float(LaunchConfiguration('drop_after').perform(context))}], output='screen'),
        Node(package='polinasi_nav', executable='range_bridge', parameters=[params], output='screen')]
    return configuration_nodes + acquisition_nodes + [
        Node(package='ros_gz_bridge', executable='parameter_bridge', name='simulation_bridge',
             parameters=[{'use_sim_time': True, 'config_file': os.path.join(share,
                 'config/bridge_identification.yaml' if mission == 'identification' else 'config/bridge.yaml')}], output='screen'),
        # mavros_node creates its own UAS node named "mavros". Adding a
        # namespace here would produce /mavros/mavros/* instead of /mavros/*.
        Node(package='mavros', executable='mavros_node',
             parameters=[os.path.join(share, 'config/mavros_mapping.yaml' if isolated else 'config/mavros.yaml')], output='screen'),
        Node(package='polinasi_nav', executable='localisation', parameters=[params, {'mode': mode}], output='screen'),
        Node(package='polinasi_nav', executable='externalnav', parameters=[params], output='screen'),
        Node(package='beehive_drone', executable='flight_manager',
             parameters=[{'use_sim_time': True, 'altitude_source': 'local_position'}], output='screen'),
        Node(package='polinasi_nav', executable={'mapping': 'mapping_navigation', 'identification': 'identification_navigation'}.get(mission, 'navigation'), parameters=[params, {'mode': mode,
             'map_log_dir': LaunchConfiguration('map_log_dir').perform(context),
             'separate_mapping': isolated,
             'autostart': LaunchConfiguration('autostart').perform(context) == 'true'}], output='screen'),
    ]


def generate_launch_description():
    share = get_package_share_directory('polinasi_nav')
    return LaunchDescription([
        DeclareLaunchArgument('localisation', default_value='ground_truth'),
        DeclareLaunchArgument('mission', default_value='inspection'),
        DeclareLaunchArgument('config', default_value=os.path.join(share, 'config/navigation.json')),
        DeclareLaunchArgument('autostart', default_value='false'),
        DeclareLaunchArgument('map_log_dir', default_value=''),
        DeclareLaunchArgument('drop_after', default_value='-1.0'), OpaqueFunction(function=nodes)])
