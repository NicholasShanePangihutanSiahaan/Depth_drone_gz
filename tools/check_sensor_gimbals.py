"""Unarmed SITL-only mount test. Never publishes arming or flight commands."""
import argparse
import hashlib
import json
import time
from pathlib import Path
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from rclpy.time import Time
from std_msgs.msg import Float64,String
from sensor_msgs.msg import Image,PointCloud2
from nav_msgs.msg import Odometry
from mavros_msgs.msg import State
from tf2_ros import Buffer,TransformListener,TransformException
from polinasi_nav.sensor_mounts import mount_rotation
from polinasi_nav.ros_nodes import rotation,vector

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output',type=Path,default=Path('reports/sensor_gimbals_check.json'))
    args = parser.parse_args()
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = Node('unarmed_simulation_gimbal_check')
    node.set_parameters([rclpy.parameter.Parameter('use_sim_time',value=True)])
    state,status,pose = {},{},{}
    counts = {'cloud':0,'image':0}
    hashes = []
    def receive_state(msg):
        state.update(armed=msg.armed,connected=msg.connected,wall=time.monotonic())
    def image(msg):
        counts['image'] += 1
        hashes[:] = [hashlib.sha256(bytes(msg.data)).hexdigest()]
    def odom(msg):
        pose.update(position=vector(msg.pose.pose.position).tolist(),matrix=rotation(msg.pose.pose.orientation).tolist())
    node.create_subscription(State,'/mavros/state',receive_state,qos_profile_sensor_data)
    node.create_subscription(String,'/mounts/status',lambda m:(status.clear(),status.update(json.loads(m.data))),1)
    node.create_subscription(Image,'/zed/image',image,qos_profile_sensor_data)
    node.create_subscription(Odometry,'/localisation/odometry',odom,qos_profile_sensor_data)
    node.create_subscription(PointCloud2,'/livox/lidar',lambda m:counts.update(cloud=counts['cloud']+1),qos_profile_sensor_data)
    buffer = Buffer(); listener = TransformListener(buffer,node)
    outputs = {j:node.create_publisher(Float64,'/mounts/'+j.replace('_','/')+'_target',1)
               for j in ('lidar_pitch','camera_yaw','camera_pitch')}
    result = dict(scope='unarmed_ground_truth_SITL_mount_test_not_flight_or_lidar_localisation',steps=[],passed=False)
    def wait(predicate,timeout):
        started = time.monotonic()
        while time.monotonic()-started<timeout:
            rclpy.spin_once(node,timeout_sec=.1)
            if state.get('armed'):
                raise RuntimeError('refusing mount test on armed vehicle')
            if predicate():
                return
        raise TimeoutError('mount feedback / sensor acquisition timed out')
    try:
        wait(lambda:state.get('connected') and state.get('armed') is False and
             status.get('feedback_fresh') and bool(pose) and counts['cloud']>5 and counts['image']>5,180.)
        initial = dict(pose)
        poses = [(0.,0.,0.),(.45,1.2,.65),(.2,-1.2,.3),(0.,0.,0.)]
        for lidar,yaw,pitch in poses:
            targets = dict(lidar_pitch=lidar,camera_yaw=yaw,camera_pitch=pitch)
            initial_counts = dict(counts)
            for j,value in targets.items():
                outputs[j].publish(Float64(data=value))
            wait(lambda:status.get('feedback_fresh') and all(
                abs(status.get('measured',{}).get(j,99.)-v)<.03 for j,v in targets.items()) and
                all(abs(status.get('targets',{}).get(j,99.)-v)<1e-6 for j,v in targets.items()) and
                counts['image']>initial_counts['image']+5 and counts['cloud']>initial_counts['cloud']+5,90.)
            wait(lambda:buffer.can_transform('base_link','mid360',Time()) and
                 buffer.can_transform('base_link','zed_camera_link',Time()),20.)
            lidar_tf = buffer.lookup_transform('base_link','mid360',Time())
            camera_tf = buffer.lookup_transform('base_link','zed_camera_link',Time())
            actual_lidar = rotation(lidar_tf.transform.rotation)
            actual_camera = rotation(camera_tf.transform.rotation)
            expected_lidar = mount_rotation(pitch=lidar)
            expected_camera = mount_rotation(yaw=yaw,pitch=pitch)
            position_change = float(np.linalg.norm(np.asarray(pose['position'])-initial['position']))
            rotation_change = float(np.linalg.norm(np.asarray(pose['matrix'])-initial['matrix']))
            step = dict(targets=targets,measured=dict(status['measured']),
                        lidar_tf_error=float(np.linalg.norm(actual_lidar-expected_lidar)),
                        camera_tf_error=float(np.linalg.norm(actual_camera-expected_camera)),
                        body_position_change_m=position_change,body_rotation_matrix_change=rotation_change,
                        image_hash=hashes[0],sensor_counts=dict(counts))
            step['passed'] = step['lidar_tf_error']<.08 and step['camera_tf_error']<.1 and position_change<.05 and rotation_change<.05
            result['steps'].append(step)
            print(json.dumps(step),flush=True)
            if not step['passed']:
                raise RuntimeError('mount TF or body isolation check failed')
        result['camera_images_changed'] = len({s['image_hash'] for s in result['steps']})>1
        result['passed'] = all(s['passed'] for s in result['steps']) and result['camera_images_changed']
    except Exception as exc:
        result['error'] = str(exc)
    finally:
        for pub in outputs.values():
            pub.publish(Float64(data=0.))
        args.output.parent.mkdir(parents=True,exist_ok=True)
        args.output.write_text(json.dumps(result,indent=2)+'\n')
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    if not result['passed']:
        raise SystemExit(1)

if __name__=='__main__':
    main()
