"""Gazebo mission: orbit one SDF tree, then one homeward virtual tree."""

import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def _launch_defaults(package_share):
    config_path = os.path.join(
        package_share, 'config', 'virtual_tree.yaml')
    with open(config_path, encoding='utf-8') as config_file:
        document = yaml.safe_load(config_file) or {}
    values = document.get('virtual_tree_mission', {}).get(
        'launch_arguments', {})

    required = {
        'auto_start', 'tree_source', 'tree_world', 'source_tree_limit',
        'mission_type', 'mission_mode', 'max_trees', 'virtual_tree_offset',
        'require_tree_ahead', 'expected_tree_count',
        'report_output_directory',
    }
    missing = sorted(required.difference(values))
    if missing:
        raise RuntimeError(
            'virtual_tree.yaml kehilangan launch argument: ' +
            ', '.join(missing))

    # Launch arguments selalu berupa teks. Konversi bool dibuat lowercase agar
    # dapat diproses ParameterValue(..., value_type=bool).
    return {
        key: str(value).lower() if isinstance(value, bool) else str(value)
        for key, value in values.items()
    }


def generate_launch_description():
    package_share = get_package_share_directory('beehive_drone')
    defaults = _launch_defaults(package_share)
    stack = os.path.join(package_share, 'launch', 'gazebo_stack.launch.py')
    return LaunchDescription([
        DeclareLaunchArgument(
            'auto_start', default_value=defaults['auto_start']),
        DeclareLaunchArgument(
            'tree_world', default_value=defaults['tree_world']),
        DeclareLaunchArgument(
            'virtual_tree_offset',
            default_value=defaults['virtual_tree_offset']),
        DeclareLaunchArgument(
            'report_output_directory',
            default_value=defaults['report_output_directory']),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(stack),
            launch_arguments={
                'auto_start': LaunchConfiguration('auto_start'),
                'tree_source': defaults['tree_source'],
                'tree_world': LaunchConfiguration('tree_world'),
                # Satu pohon fisik dari SDF; target kedua dibuat setelah
                # pohon pertama selesai diorbit.
                'source_tree_limit': defaults['source_tree_limit'],
                'mission_mode': defaults['mission_mode'],
                'mission_type': defaults['mission_type'],
                'max_trees': defaults['max_trees'],
                'virtual_tree_offset':
                    LaunchConfiguration('virtual_tree_offset'),
                'require_tree_ahead': defaults['require_tree_ahead'],
                'expected_tree_count': defaults['expected_tree_count'],
                'report_output_directory':
                    LaunchConfiguration('report_output_directory'),
            }.items()),
    ])
