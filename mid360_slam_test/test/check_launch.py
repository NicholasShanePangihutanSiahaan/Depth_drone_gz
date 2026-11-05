"""Installed-package synthetic replay smoke test, localhost ROS domain 196.

Source ROS + install_slam_test first. Requires domain 196 to be unused.
No driver, hardware or interactive operator is used. Outputs go to /tmp.
"""
import json
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time
import uuid
import numpy as np
import yaml
import rclpy
from sensor_msgs.msg import Imu,PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import String
from rosgraph_msgs.msg import Clock
from mid360_test.nodes import sensor_qos
from mid360_slam_test.nodes import latched


def main():
    root=Path(tempfile.mkdtemp(prefix='mid360_slam_launch_'))
    session=root/'session';config=root/'fixture.yaml'
    params=dict(mode='replay',cloud_type='sensor_msgs/msg/PointCloud2',
        calibration_seconds=.1,calibration_min_samples=5,imu_receipt_timeout_s=1.,
        estimator_timeout_s=2.,settle_window_s=.4,minimum_settle_samples=3,
        max_stationary_drift_m=.03,max_stationary_orientation_deg=1.,
        max_scan_gap_s=1.,loop_min_separation=4,loop_min_travel_m=.3,
        loop_min_seconds=.3,loop_cooldown_s=.2,keyframe_translation_m=.04,
        loop_radius_m=.15,voxel_m=.04,map_publish_period_s=.3)
    config.write_text(yaml.safe_dump({'/**':{'ros__parameters':params}}))
    env=dict(os.environ,ROS_DOMAIN_ID='196',ROS_LOCALHOST_ONLY='1',
        ROS_LOG_DIR=str(root/'ros_logs'),PYTHONDONTWRITEBYTECODE='1',OPENBLAS_NUM_THREADS='1')
    os.environ.update({k:env[k] for k in ('ROS_DOMAIN_ID','ROS_LOCALHOST_ONLY','ROS_LOG_DIR')})
    with (root/'launch.log').open('w') as log:
        proc=subprocess.Popen(['ros2','launch','mid360_slam_test','replay_test.launch.py',
            'rviz:=false','config:='+str(config),'output_dir:='+str(session)],
            env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        node=None
        try:
            deadline=time.monotonic()+20.
            while not (session/'recorder_status.json').exists():
                assert proc.poll() is None,(root/'launch.log').read_text()
                assert time.monotonic()<deadline,'startup timeout'
                time.sleep(.1)
            rclpy.init()
            node=rclpy.create_node('synthetic_slam_replay_fixture')
            imus=node.create_publisher(Imu,'/livox/imu',sensor_qos())
            clouds=node.create_publisher(PointCloud2,'/livox/lidar',sensor_qos())
            clocks=node.create_publisher(Clock,'/clock',10)
            commands=node.create_publisher(String,'/slam_test/commands',10)
            state={};status={}
            node.create_subscription(String,'/slam_test/test_state',lambda msg:state.update(json.loads(msg.data)),latched())
            node.create_subscription(String,'/slam_test/estimator_status',lambda msg:status.update(json.loads(msg.data)),20)
            world=np.random.default_rng(22).uniform([-3,-3,-1],[3,3,2],(700,3))
            count=0
            def spin(seconds):
                until=time.monotonic()+seconds
                while time.monotonic()<until:
                    assert proc.poll() is None,(root/'launch.log').read_text()
                    rclpy.spin_once(node,timeout_sec=.005)
            spin(1.)
            def publish(x=0.):
                nonlocal count
                for _ in range(5):
                    count+=1;stamp=100.+count*.02
                    clock=Clock();clock.clock.sec=int(stamp);clock.clock.nanosec=round((stamp-int(stamp))*1e9)
                    clocks.publish(clock)
                    imu=Imu();imu.header.frame_id='livox_frame';imu.header.stamp=clock.clock
                    imu.linear_acceleration.z=1.;imus.publish(imu);spin(.02)
                p=world-np.array([x,0.,0.])
                clouds.publish(point_cloud2.create_cloud_xyz32(imu.header,p.tolist()))
                spin(.03)
            for _ in range(40):
                publish()
                if state.get('can_start'):
                    break
            assert state.get('can_start'),(state,status)
            def command(action,**kwargs):
                commands.publish(String(data=json.dumps(dict(id=uuid.uuid4().hex,action=action,**kwargs))))
                spin(.1)
            command('reference',duration_s=.4)
            command('start',reference_confirmed=True)
            for _ in range(10):publish()
            assert state.get('phase')=='REVIEW',state
            assert state['results'][-1]['status']=='PASS',state
            for x in [0,.05,.1,.15,.2,.25,.25,.2,.15,.1,.05,0]:publish(x)
            for _ in range(3):publish()
            assert status.get('valid'),status
            assert status.get('loops_accepted',0)>0,status
            command('quit')
            node.destroy_node();node=None;rclpy.shutdown()
        finally:
            if node is not None:
                node.destroy_node()
                if rclpy.ok():rclpy.shutdown()
            if proc.poll() is None:
                proc.send_signal(signal.SIGINT);proc.wait(timeout=45)
    rec=json.loads((session/'recorder_status.json').read_text())
    assert rec['closed'] and rec['dropped_by_recorder']==0,rec
    assert rec['written_by_topic']['/slam_test/loop_events']>0,rec
    assert (session/'raw_bag/metadata.yaml').exists()
    assert (session/'keyframe_poses.npz').exists()
    result=dict(status='PASS',scope='installed_multi_process_synthetic_replay_only',
                session=str(session),recorder=rec,estimator=status)
    (root/'result.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(dict(status=result['status'],scope=result['scope'],session=str(session),
        messages=rec['written_messages'],loops_accepted=status['loops_accepted']),indent=2))


if __name__=='__main__':
    main()
