"""Backward-compatible alias for the simulation mission."""
from pathlib import Path
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    simulation = Path(__file__).with_name('simulation_mission.launch.py')
    return LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(simulation)))
    ])
