"""New sessions only; never start drivers, arm, or resume old simulation loops."""
import datetime
import hashlib
import json
from pathlib import Path
import subprocess
import uuid
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def description(mode):
    share = Path(get_package_share_directory('mid360_slam_test'))
    def setup(context):
        def arg(key): return LaunchConfiguration(key).perform(context)
        config = Path(arg('config')).expanduser().resolve()
        data = yaml.safe_load(config.read_text())['/**']['ros__parameters']
        if mode == 'hardware' and not data.get('mounting_confirmed',False):
            raise ValueError('Periksa satuan IMU, frame, dan rotasi IMU-ke-LiDAR; '
                             'lalu set mounting_confirmed: true pada salinan konfigurasi.')
        output = Path(arg('output_dir')).expanduser().resolve()
        output.mkdir(parents=True,exist_ok=False)
        (output/'configuration.yaml').write_text(config.read_text())
        source = Path(__file__).resolve().parent
        try:
            revision = subprocess.run(['git','-C',str(source),'rev-parse','HEAD'],
                capture_output=True,text=True,check=True,timeout=2).stdout.strip()
        except (OSError,subprocess.SubprocessError):
            revision = 'UNKNOWN'
        (output/'code_manifest.json').write_text(json.dumps(dict(revision=revision,mode=mode,
            utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),
            python_sha256={str(p.relative_to(source)):hashlib.sha256(p.read_bytes()).hexdigest()
                           for p in source.rglob('*.py')}),indent=2)+'\n')
        print('SLAM test session: '+str(output),flush=True)
        print('Terminal kedua: ros2 run mid360_slam_test test_controller',flush=True)
        params=[str(config),dict(mode=mode,use_sim_time=mode=='replay',output_dir=str(output))]
        actions=[]
        for package,executable in [('mid360_test','cloud_adapter'),
            ('mid360_slam_test','cloud_preprocessor'),('mid360_slam_test','lidar_imu_estimator'),
            ('mid360_slam_test','test_evaluator'),('mid360_slam_test','test_monitor'),
            ('mid360_slam_test','slam_recorder')]:
            node=Node(package=package,executable=executable,parameters=params,output='screen',
                      sigterm_timeout='30',sigkill_timeout='10')
            actions.append(RegisterEventHandler(OnProcessExit(target_action=node,
                on_exit=[EmitEvent(event=Shutdown(reason=executable+' exited'))])))
            actions.append(node)
        if arg('rviz')=='true':
            actions.append(Node(package='rviz2',executable='rviz2',
                arguments=['-d',str(share/'config/slam.rviz')],
                parameters=[dict(use_sim_time=mode=='replay')],output='screen'))
        return actions
    output=str(Path.cwd()/'reports/slam_test'/(
        mode+'_'+datetime.datetime.now().strftime('%Y%m%d_%H%M%S')+'_'+uuid.uuid4().hex[:6]))
    return LaunchDescription([
        DeclareLaunchArgument('config',default_value=str(share/'config'/f'{mode}.yaml')),
        DeclareLaunchArgument('output_dir',default_value=output),
        DeclareLaunchArgument('rviz',default_value='true',choices=['true','false']),
        OpaqueFunction(function=setup)])
