#!/usr/bin/env python3
"""Headless sensor bench in plantation_simple; no SITL, MAVROS or arming.

An ephemeral stationary model keeps the drone geometry and top sensors, but
removes actuator plugins and other sensors. Source models/world stay intact.
"""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time
import uuid
import xml.etree.ElementTree as ET
import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2,Imu
from rosgraph_msgs.msg import Clock
from tf2_ros import Buffer,TransformListener,TransformException
from rclpy.time import Time

ROOT=Path(__file__).resolve().parents[1]
PACKAGE=ROOT/'src/uav_plantation_sim'


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--timeout',type=float,default=90.)
    args=parser.parse_args()
    args.output.parent.mkdir(parents=True,exist_ok=True)
    # Isolate Gazebo discovery; caller chooses a separate ROS domain as well.
    os.environ['GZ_PARTITION']='top_livox_bench_'+uuid.uuid4().hex
    os.environ['GZ_SIM_RESOURCE_PATH']=str(PACKAGE/'models')+':'+os.environ.get('GZ_SIM_RESOURCE_PATH','')
    processes=[];handles=[]
    report=dict(scope='stationary_sensor_bench_not_flight_validation',
        world='plantation_simple.sdf',cloud_count=0,imu_count=0,
        finite_points=0,cloud_stamps=[],imu_stamps=[],frames=[],passed=False)
    def stamp(header):return header.stamp.sec+header.stamp.nanosec*1e-9
    def start(command,log):
        handle=open(args.output.parent/log,'w');handles.append(handle)
        p=subprocess.Popen(command,stdout=handle,stderr=subprocess.STDOUT,start_new_session=True)
        processes.append(p)
    def cloud(msg):
        report['cloud_count']+=1
        report['cloud_stamps'].append(stamp(msg.header))
        report['frames'].append(msg.header.frame_id)
        if msg.height!=32 or msg.width!=360:
            report['shape_error']=[msg.width,msg.height]
        fields={f.name:f for f in msg.fields}
        if not all(k in fields and fields[k].datatype==7 for k in ('x','y','z')):
            report['field_error']=True;return
        points=np.ndarray((msg.height,msg.width),dtype=np.dtype(dict(
            names=['x','y','z'],formats=[('>' if msg.is_bigendian else '<')+'f4']*3,
            offsets=[fields[k].offset for k in ('x','y','z')],itemsize=msg.point_step)),
            buffer=msg.data,strides=(msg.row_step,msg.point_step))
        report['finite_points']=max(report['finite_points'],int(np.count_nonzero(
            np.isfinite(points['x'])&np.isfinite(points['y'])&np.isfinite(points['z']))))
    def imu(msg):
        report['imu_count']+=1;report['imu_stamps'].append(stamp(msg.header))
        if msg.header.frame_id!='livox_top_link':report['imu_frame_error']=msg.header.frame_id
    node=None
    try:
        with tempfile.TemporaryDirectory(prefix='livox_top_bench_') as directory:
            world=ET.parse(PACKAGE/'worlds/plantation_simple.sdf')
            model=ET.parse(PACKAGE/'models/plantation_quadrotor/model.sdf').find('model')
            for plugin in model.findall('plugin'):model.remove(plugin)
            for link in model.findall('link'):
                if link.get('name')!='livox_top_link':
                    for sensor in link.findall('sensor'):link.remove(sensor)
            ET.SubElement(model,'static').text='true'
            ET.SubElement(model,'pose').text='0 -3 .12 0 0 0'
            world.find('world').append(model)
            path=Path(directory)/'plantation_simple_sensor_bench.sdf'
            world.write(path,encoding='unicode',xml_declaration=True)
            start(['gz','sim','-s','-r','--headless-rendering','-v','2',str(path)],'gazebo.log')
            start(['/opt/ros/humble/lib/ros_gz_bridge/parameter_bridge','--ros-args',
                '-p','config_file:='+str(PACKAGE/'config/bridge_config.yaml')],'bridge.log')
            start(['/opt/ros/humble/lib/tf2_ros/static_transform_publisher',
                '--x','0','--y','0','--z','.18','--roll','0','--pitch','0','--yaw','0',
                '--frame-id','base_link','--child-frame-id','livox_top_link'],'mount_tf.log')
            rclpy.init();node=rclpy.create_node('top_livox_sensor_check')
            buffer=Buffer();listener=TransformListener(buffer,node)
            node.create_subscription(PointCloud2,'/livox/top/points',cloud,qos_profile_sensor_data)
            node.create_subscription(Imu,'/livox/top/imu',imu,qos_profile_sensor_data)
            received_clock=[]
            node.create_subscription(Clock,'/clock',lambda m:received_clock.append(
                m.clock.sec+m.clock.nanosec*1e-9),qos_profile_sensor_data)
            deadline=time.monotonic()+args.timeout
            while time.monotonic()<deadline:
                rclpy.spin_once(node,timeout_sec=.1)
                if any(p.poll() is not None for p in processes):break
                try:
                    tf=buffer.lookup_transform('base_link','livox_top_link',Time())
                    report['mount_tf_correct']=bool(abs(tf.transform.translation.z-.18)<1e-8
                        and abs(tf.transform.translation.x)+abs(tf.transform.translation.y)<1e-8
                        and abs(tf.transform.rotation.w-1.)<1e-8)
                except TransformException:pass
                if report['cloud_count']>=10 and report['imu_count']>=40 and received_clock and report.get('mount_tf_correct'):break
            report['clock_received']=bool(received_clock)
            report['passed']=bool(report['cloud_count']>=10 and report['imu_count']>=40
                and report['finite_points']>0 and report['clock_received']
                and report.get('mount_tf_correct',False)
                and set(report['frames'])=={'livox_top_link'}
                and not any(k.endswith('_error') for k in report)
                and all(b>a for a,b in zip(report['cloud_stamps'],report['cloud_stamps'][1:]))
                and all(b>a for a,b in zip(report['imu_stamps'],report['imu_stamps'][1:])))
    finally:
        if node is not None:node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
        for p in processes:
            if p.poll() is None:os.killpg(p.pid,signal.SIGTERM)
        for p in processes:
            try:p.wait(timeout=10.)
            except subprocess.TimeoutExpired:
                os.killpg(p.pid,signal.SIGKILL);p.wait()
        for handle in handles:handle.close()
        report['processes_stopped']=all(p.poll() is not None for p in processes)
        args.output.write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps({k:v for k,v in report.items() if not k.endswith('_stamps') and k!='frames'},indent=2))
    raise SystemExit(0 if report['passed'] else 1)


if __name__=='__main__':main()
