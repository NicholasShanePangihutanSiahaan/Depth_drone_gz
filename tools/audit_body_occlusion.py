"""Passive Gazebo-only check: solid core body must block rays behind it."""
import argparse,json,time
from pathlib import Path
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from tf2_ros import Buffer,TransformListener,TransformException
from polinasi_nav.simulated_rays import directions
from polinasi_nav.ros_nodes import rotation,vector
from polinasi_nav.self_filter import self_mask


def main():
    p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True)
    p.add_argument('--seconds',type=float,default=90.)
    p.add_argument('--topic',default='/simulation/lidar')
    args=p.parse_args();rclpy.init();node=Node('body_occlusion_passive_audit')
    config=json.loads((Path(__file__).resolve().parents[1]/'reports/visual50_hover/config.json').read_text())
    node.set_parameters([rclpy.parameter.Parameter('use_sim_time',value=True)])
    buffer=Buffer();listener=TransformListener(buffer,node)
    result=dict(scope='Gazebo_main_body_core_only_not_all_drone_geometry',scans=0,
                blocked_body_rays=0,blocked_finite=0,blocked_negative_infinite_or_nan=0,
                blocked_positive_no_return=0,tf_failures=0)
    result.update(blocked_finite_beyond_body_exit=0,blocked_valid_self_returns_not_filtered=0)
    expected=directions()
    def cloud(msg):
        try:
            tf=buffer.lookup_transform('base_link','mid360',Time.from_msg(msg.header.stamp))
            raw=np.asarray([tuple(p) for p in point_cloud2.read_points(msg,
                field_names=('x','y','z'),skip_nans=False)],float)
            if raw.shape!=expected.shape:return
            rays=expected@rotation(tf.transform.rotation).T
            origin=vector(tf.transform.translation)
            # Shrink the actual visual body by 2 mm to avoid grazing ambiguity.
            extent=np.asarray([.13,.09,.0475])-.002
            with np.errstate(divide='ignore',invalid='ignore'):
                a=(-extent-origin)/rays;b=(extent-origin)/rays
            entry=np.max(np.minimum(a,b),axis=1);exit=np.min(np.maximum(a,b),axis=1)
            blocked=(exit>np.maximum(entry,0))&(entry>0)&(entry<1.)
            positive=np.all(np.isinf(raw),axis=1)&np.all(np.sign(raw)==np.sign(expected),axis=1)
            finite=np.all(np.isfinite(raw),axis=1)
            distance=np.full(len(raw),np.inf)
            distance[finite]=np.linalg.norm(raw[finite],axis=1)
            beyond=blocked&finite&(distance>exit+.06)
            body_points=raw[finite]@rotation(tf.transform.rotation).T+origin
            filtered=np.zeros(len(raw),dtype=bool)
            filtered[finite]=self_mask(body_points,config)
            leaked=blocked&finite&(distance>=config['lidar_min_range'])&~filtered
            result['scans']+=1
            result['blocked_body_rays']+=int(blocked.sum())
            result['blocked_finite']+=int((blocked&finite).sum())
            result['blocked_positive_no_return']+=int((blocked&positive).sum())
            result['blocked_negative_infinite_or_nan']+=int((blocked&~finite&~positive).sum())
            result['blocked_finite_beyond_body_exit']+=int(beyond.sum())
            result['blocked_valid_self_returns_not_filtered']+=int(leaked.sum())
        except TransformException:result['tf_failures']+=1
    node.create_subscription(PointCloud2,args.topic,cloud,qos_profile_sensor_data)
    end=time.monotonic()+args.seconds
    try:
        while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.1)
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
    result['passed']=(result['blocked_body_rays']>0 and result['blocked_positive_no_return']==0
                      and result['blocked_finite_beyond_body_exit']==0
                      and result['blocked_valid_self_returns_not_filtered']==0)
    result['topic']=args.topic
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))


if __name__=='__main__':main()
