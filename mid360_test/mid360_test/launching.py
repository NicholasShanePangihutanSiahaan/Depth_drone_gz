"""Shared hardware/replay launch; driver startup is deliberately operator-owned."""
import datetime
import hashlib
import json
import os
from pathlib import Path
import platform
import subprocess
import uuid

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_description(mode):
    share = Path(get_package_share_directory('mid360_test'))

    def nodes(context):
        def arg(name):
            return LaunchConfiguration(name).perform(context)
        config = Path(arg('config')).expanduser().resolve()
        if not config.is_file():
            raise ValueError('configuration file does not exist: '+str(config))
        output = Path(arg('output_dir')).expanduser().resolve()
        # Refuse reused directories: old records must never contaminate a new session.
        output.mkdir(parents=True, exist_ok=False)
        try:
            repo = Path(__file__).resolve().parents[2]
            revision = subprocess.run(['git', '-C', str(repo), 'rev-parse', 'HEAD'],
                capture_output=True, text=True, timeout=2, check=True).stdout.strip()
            dirty = subprocess.run(['git', '-C', str(repo), 'status', '--porcelain', '--untracked-files=no'],
                capture_output=True, text=True, timeout=2, check=True).stdout.strip()
        except (OSError, subprocess.SubprocessError):
            revision, dirty = 'UNKNOWN_installed_without_git', 'UNKNOWN'
        manifest = dict(mode=mode, created_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),
            revision=revision, tracked_changes_present=bool(dirty), tracked_changes=dirty,
            config_path=str(config), config_sha256=hashlib.sha256(config.read_bytes()).hexdigest(),
            machine=platform.machine(), kernel=platform.release(), python=platform.python_version(),
            ros_distro=os.environ.get('ROS_DISTRO', 'UNKNOWN'),
            ros_domain_id=os.environ.get('ROS_DOMAIN_ID', '0'))
        source_root = Path(__file__).resolve().parent
        manifest['python_source_sha256'] = {
            str(p.relative_to(source_root)): hashlib.sha256(p.read_bytes()).hexdigest()
            for p in sorted(source_root.rglob('*.py'))}
        try:
            import xml.etree.ElementTree as ET
            driver_share = Path(get_package_share_directory('livox_ros_driver2'))
            manifest['driver_package_version'] = ET.parse(driver_share/'package.xml').findtext('version')
        except Exception:
            manifest['driver_package_version'] = 'UNKNOWN_driver_overlay_not_available'
        (output/'launch_session.json').write_text(json.dumps(manifest, indent=2)+'\n')
        print('MID-360 '+mode+' logs: '+str(output), flush=True)
        params = [str(config), dict(mode=mode, use_sim_time=mode == 'replay', output_dir=str(output))]
        actions = []
        for executable in ('cloud_adapter', 'directional_distance', 'imu_integration',
                           'health_monitor', 'test_logger', 'raw_recorder'):
            node = Node(package='mid360_test', executable=executable, output='screen', parameters=params)
            # Recorder/analysis failure ends this session; do not leave misleading live logs.
            actions.append(RegisterEventHandler(OnProcessExit(target_action=node,
                on_exit=[EmitEvent(event=Shutdown(reason=executable+' exited'))])))
            actions.append(node)
        if arg('rviz').lower() == 'true':
            actions.append(Node(package='rviz2', executable='rviz2', output='screen',
                arguments=['-d', str(share/'config/mid360.rviz')],
                parameters=[dict(use_sim_time=mode == 'replay')]))
        return actions

    default_output = str(Path.cwd()/'reports/lidar_test'/(
        mode+'_'+datetime.datetime.now().strftime('%Y%m%d_%H%M%S')+'_'+uuid.uuid4().hex[:6]))
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=str(share/'config'/f'{mode}.yaml')),
        DeclareLaunchArgument('output_dir', default_value=default_output),
        DeclareLaunchArgument('rviz', default_value='false', choices=['true', 'false']),
        OpaqueFunction(function=nodes),
    ])
