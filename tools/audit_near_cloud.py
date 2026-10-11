"""Read-only, bounded near-body cloud diagnostics for simulation audits."""
import argparse
import json
import time
from pathlib import Path
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from tf2_ros import Buffer, TransformListener, TransformException
from polinasi_nav.ros_nodes import rotation, vector, seconds

def main():
    p = argparse.ArgumentParser()
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--seconds',type=float,default=600.)
    args = p.parse_args()
    rclpy.init()
    node = Node('simulation_near_cloud_audit')
    node.set_parameters([rclpy.parameter.Parameter('use_sim_time',value=True)])
    buffer = Buffer(); listener = TransformListener(buffer,node)
    args.output.parent.mkdir(parents=True,exist_ok=True)
    with args.output.open('w') as output:
        def cloud(msg):
            try:
                t = buffer.lookup_transform('base_link',msg.header.frame_id,Time.from_msg(msg.header.stamp))
                points = np.array([tuple(x) for x in point_cloud2.read_points(msg,field_names=('x','y','z'),skip_nans=True)])
                if not len(points): return
                points = points[np.all(np.isfinite(points),axis=1)]
                body = points@rotation(t.transform.rotation).T+vector(t.transform.translation)
                selected = np.flatnonzero(np.linalg.norm(body,axis=1)<.8)
                output.write(json.dumps(dict(stamp=seconds(msg.header.stamp),sensor_points=points[selected][:64].tolist(),body_points=body[selected][:64].tolist(),count=len(selected)))+'\n')
                output.flush()
            except TransformException:
                pass
        node.create_subscription(PointCloud2,'/livox/lidar',cloud,qos_profile_sensor_data)
        started = time.monotonic()
        while rclpy.ok() and time.monotonic()-started<args.seconds:
            rclpy.spin_once(node,timeout_sec=.1)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__': main()
