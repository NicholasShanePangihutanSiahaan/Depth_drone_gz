"""Installed-package smoke test. Synthetic replay only, localhost domain 194.

Run after sourcing ROS and the mid360_test install overlay. Uses /tmp outputs.
Never run in a domain occupied by a driver or a hardware session.
"""
import json
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time

import rclpy
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Imu, PointCloud2
from sensor_msgs_py import point_cloud2

root = Path(tempfile.mkdtemp(prefix='mid360_multiprocess_'))
session = root/'session'
env = dict(os.environ, ROS_DOMAIN_ID='194', ROS_LOCALHOST_ONLY='1',
           ROS_LOG_DIR=str(root/'ros_logs'), PYTHONDONTWRITEBYTECODE='1')
os.environ.update({k: env[k] for k in ('ROS_DOMAIN_ID', 'ROS_LOCALHOST_ONLY', 'ROS_LOG_DIR')})
with (root/'launch.log').open('w') as log:
    proc = subprocess.Popen(['ros2', 'launch', 'mid360_test', 'replay_test.launch.py',
        'output_dir:='+str(session)], env=env, stdout=log, stderr=subprocess.STDOUT,
        start_new_session=True)
    try:
        deadline=time.monotonic()+15
        while not (session/'recorder_status.json').exists():
            assert proc.poll() is None, (root/'launch.log').read_text()
            assert time.monotonic()<deadline, 'startup timeout'
            time.sleep(.1)
        rclpy.init()
        node = rclpy.create_node('synthetic_replay_fixture')
        cloud_pub = node.create_publisher(PointCloud2, '/livox/lidar', qos_profile_sensor_data)
        imu_pub = node.create_publisher(Imu, '/livox/imu', qos_profile_sensor_data)
        clock_pub = node.create_publisher(Clock, '/clock', 10)
        time.sleep(1.)
        for i in range(200):
            assert proc.poll() is None, (root/'launch.log').read_text()
            sec, nanosec = 100+i//50, (i%50)*20000000
            clock=Clock()
            clock.clock.sec, clock.clock.nanosec=sec,nanosec
            clock_pub.publish(clock)
            imu=Imu()
            imu.header.frame_id='livox_frame'
            imu.header.stamp.sec,imu.header.stamp.nanosec=sec,nanosec
            imu.linear_acceleration.z=1.
            imu_pub.publish(imu)
            if i%5==0:
                cloud_pub.publish(point_cloud2.create_cloud_xyz32(imu.header,
                    [[1.,0.,0.],[0.,-2.,0.],[0.,3.,0.],[-4.,0.,0.],[0.,0.,0.]]))
            rclpy.spin_once(node, timeout_sec=.02)
        node.destroy_node()
        rclpy.shutdown()
    finally:
        if proc.poll() is None:
            proc.send_signal(signal.SIGINT)
            proc.wait(timeout=25)
rec=json.loads((session/'recorder_status.json').read_text())
summary=json.loads((session/'summary.json').read_text())
text=(session/'readings.log').read_text()
assert rec['closed'] and rec['status']=='PASS', rec
assert rec['written_by_topic']['/livox/lidar']>0 and rec['written_by_topic']['/livox/imu']>0
assert 'front = 100.0 cm, right = 200.0 cm, left = 300.0 cm, back = 400.0 cm' in text, text
assert 'Yaw_est = 0.00 deg | Speed_est = 0.000 m/s' in text, text
print(json.dumps(dict(status='PASS', scope='multi_process_synthetic_replay_only',
    session=str(session), recorder=rec, summary_status=summary['status']), indent=2))
