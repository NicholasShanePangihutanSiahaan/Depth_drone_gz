"""Unarmed empty-arena A/B tilt audit. Publishes mount targets only."""
import argparse
import io
import json
import time
from pathlib import Path
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import Float64,String,UInt8MultiArray
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from nav_msgs.msg import Odometry
from mavros_msgs.msg import State
from tf2_ros import Buffer,TransformListener,TransformException
from polinasi_nav.ros_nodes import rotation,vector,seconds

def main():
    p = argparse.ArgumentParser()
    p.add_argument('--config',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--lidar-only',action='store_true')
    args = p.parse_args()
    config = json.loads(args.config.read_text())
    if config.get('active_tree_inspection') or config.get('automatic_tilt_ascent'):
        raise ValueError('use the dedicated empty unarmed test config')
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = Node('empty_mount_AB_audit')
    node.set_parameters([rclpy.parameter.Parameter('use_sim_time',value=True)])
    buffer = Buffer(); listener = TransformListener(buffer,node)
    state,status,pose = {},{},{}
    collecting = [None]
    latest_map = {}
    outputs = {j:node.create_publisher(Float64,'/mounts/'+j.replace('_','/')+'_target',1)
        for j in ('lidar_pitch','camera_yaw','camera_pitch')}
    def fcu(msg): state.update(armed=msg.armed,connected=msg.connected)
    def odom(msg): pose.update(xyz=vector(msg.pose.pose.position),matrix=rotation(msg.pose.pose.orientation))
    def cloud(msg):
        step = collecting[0]
        if step is None or seconds(msg.header.stamp)<=step['start_sim']:
            return
        try:
            body_tf = buffer.lookup_transform('base_link',msg.header.frame_id,Time.from_msg(msg.header.stamp))
            world_tf = buffer.lookup_transform('map',msg.header.frame_id,Time.from_msg(msg.header.stamp))
            points = np.array([tuple(x) for x in point_cloud2.read_points(msg,field_names=('x','y','z'),skip_nans=True)])
            if not len(points): return
            points = points[np.all(np.isfinite(points),axis=1)]
            body = points@rotation(body_tf.transform.rotation).T+vector(body_tf.transform.translation)
            world = points@rotation(world_tf.transform.rotation).T+vector(world_tf.transform.translation)
            near = (np.linalg.norm(body,axis=1)<.9)&(world[:,2]>.1)
            outside = np.any(np.abs(body)>np.asarray(config['drone_dimensions'])/2+config['self_filter_padding'],axis=1)
            step['clouds'] += 1
            step['near_above_floor_points'] += int(near.sum())
            step['outside_body_filter_points'] += int((near&outside).sum())
            step['outside_filter_peak_per_scan'] = max(step['outside_filter_peak_per_scan'],int((near&outside).sum()))
            far = np.linalg.norm(body,axis=1)>=.9
            step['far_above_floor_points'] += int((far&(world[:,2]>.1)).sum())
            if np.any(far):
                step['maximum_far_point_height_m'] = max(step['maximum_far_point_height_m'],float(world[far,2].max()))
            fixture = config.get('simulation_lidar_validation_box')
            if fixture:
                expected = np.all(np.abs(world-np.asarray(fixture['centre']))<=np.asarray(fixture['size'])/2+.05,axis=1)
                step['fixture_surface_points'] += int(expected.sum())
            if np.any(near&outside) and not step['examples_body_xyz']:
                step['examples_body_xyz'] = body[near&outside][:12].tolist()
        except TransformException:
            pass
    def snapshot(msg):
        with np.load(io.BytesIO(bytes(msg.data)),allow_pickle=False) as data:
            metadata = json.loads(data['metadata'].tobytes())
            ids = np.argwhere(data['state']==1)
            centres = np.asarray(metadata['origin'])+(ids+.5)*metadata['resolution']
        if pose:
            near = (centres[:,2]>.15)&(np.linalg.norm(centres-pose['xyz'],axis=1)<.9)
            latest_map.update(stamp=metadata['observation_stamp'],near_occupied_voxels=int(near.sum()),examples=centres[near][:12].tolist(),
                above_floor_occupied_voxels=int((centres[:,2]>.15).sum()),floor_occupied_voxels=int((centres[:,2]<=.15).sum()))
            fixture = config.get('simulation_lidar_validation_box')
            if fixture:
                inside = np.all(np.abs(centres-np.asarray(fixture['centre']))<=np.asarray(fixture['size'])/2+metadata['resolution'],axis=1)
                latest_map['fixture_occupied_voxels'] = int(inside.sum())
                latest_map['unexpected_above_floor_voxels'] = int(((centres[:,2]>.15)&~inside).sum())
    node.create_subscription(State,'/mavros/state',fcu,qos_profile_sensor_data)
    node.create_subscription(String,'/mounts/status',lambda m:(status.clear(),status.update(json.loads(m.data))),1)
    node.create_subscription(Odometry,'/localisation/odometry',odom,qos_profile_sensor_data)
    node.create_subscription(PointCloud2,'/livox/lidar',cloud,qos_profile_sensor_data)
    node.create_subscription(UInt8MultiArray,'/mapping/navigation_snapshot',snapshot,1)
    def now(): return node.get_clock().now().nanoseconds*1e-9
    def wait(predicate,timeout=90.):
        start = time.monotonic()
        while time.monotonic()-start<timeout:
            rclpy.spin_once(node,timeout_sec=.1)
            if state.get('armed'): raise RuntimeError('test refuses an armed vehicle')
            if predicate(): return
        raise TimeoutError('sensor/mount test timed out')
    result = dict(scope='unarmed_empty_arena_not_flight',hide_self=config['simulation_lidar_hide_self'],
                  floor_z=0.,floor_exclusion_point_z=.1,floor_exclusion_voxel_z=.15,steps=[],completed=False)
    try:
        wait(lambda:state.get('connected') and state.get('armed') is False and pose and status.get('feedback_fresh') and latest_map,180.)
        initial_xyz,initial_matrix = pose['xyz'].copy(),pose['matrix'].copy()
        sequence = [(0.,0.,0.),(.25,0.,0.),(.5,0.,0.),(.7,0.,0.),(.7,1.2,.65),(.7,-1.2,.3),(0.,0.,0.)]
        if args.lidar_only: sequence = [(0.,0.,0.),(.7,0.,0.),(0.,0.,0.)]
        for lidar,yaw,pitch in sequence:
            targets = dict(lidar_pitch=lidar,camera_yaw=yaw,camera_pitch=pitch)
            step = dict(targets=targets,start_sim=now(),clouds=0,near_above_floor_points=0,
                        outside_body_filter_points=0,outside_filter_peak_per_scan=0,examples_body_xyz=[],
                        far_above_floor_points=0,maximum_far_point_height_m=-1e9,fixture_surface_points=0)
            collecting[0] = step  # Include transition: artefacts can be transient.
            for j,value in targets.items(): outputs[j].publish(Float64(data=value))
            wait(lambda:status.get('feedback_fresh') and all(abs(status['measured'].get(j,99)-v)<.025 for j,v in targets.items()) and all(abs(status['targets'].get(j,99)-v)<1e-6 for j,v in targets.items()))
            step['settled_sim'] = now()
            wait(lambda:now()-step['settled_sim']>=2. and step['clouds']>=15 and latest_map.get('stamp',-1)>step['settled_sim']+1.)
            collecting[0] = None
            step.update(end_sim=now(),measured=dict(status['measured']),map=dict(latest_map),
                        body_position_change_m=float(np.linalg.norm(pose['xyz']-initial_xyz)),
                        body_rotation_matrix_change=float(np.linalg.norm(pose['matrix']-initial_matrix)))
            result['steps'].append(step)
            print(json.dumps(step),flush=True)
            if step['body_position_change_m']>.05 or step['body_rotation_matrix_change']>.05:
                raise RuntimeError('body moved during mount-only experiment')
        result['completed'] = True
    except Exception as exc:
        result['error'] = str(exc)
    finally:
        for pub in outputs.values(): pub.publish(Float64(data=0.))
        args.output.parent.mkdir(parents=True,exist_ok=True)
        args.output.write_text(json.dumps(result,indent=2)+'\n')
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
    if not result['completed']: raise SystemExit(1)

if __name__=='__main__': main()
