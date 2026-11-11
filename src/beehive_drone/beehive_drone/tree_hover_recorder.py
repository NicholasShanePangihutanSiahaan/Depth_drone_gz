"""Per-tree hover telemetry and actual LiDAR point-cloud snapshots."""
import csv
import json
import math
import time
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String, Int32, Float32, Float64MultiArray
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import PointCloud2, Range
from sensor_msgs_py import point_cloud2
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


class TreeHoverRecorder(Node):
    def __init__(self):
        super().__init__('tree_hover_recorder')
        self.declare_parameter('output_directory','~/beehive_mission_reports/multi_tree')
        self.directory=Path(self.get_parameter('output_directory').value).expanduser()/time.strftime('%Y%m%d_%H%M%S')
        self.directory.mkdir(parents=True)
        self.lidar_estimate_ids=set()
        self.tree_id=-1;self.state='';self.values={};self.estimates={};self.cloud=None;self.cloud_time=0.;self.saved=set()
        self.columns=['wall_time','tree_id','state','world_z','local_z','rangefinder','filtered_agl','setpoint_z','observed_base_z','observed_height','estimate_age_s','height_source']
        self.file=(self.directory/'hover_altitudes.csv').open('w',newline='');self.writer=csv.DictWriter(self.file,fieldnames=self.columns);self.writer.writeheader()
        self.create_subscription(Int32,'/mission/active_tree_id',self.target,10)
        self.create_subscription(String,'/mission/fsm_state',self.status,10)
        self.create_subscription(Float64MultiArray,'/map/trunk_estimates',self.estimate,10)
        self.create_subscription(Float64MultiArray,'/perception/lidar_verified_tree',self.lidar_estimate,10)
        def sub(cls,topic,key,extract):
            self.create_subscription(cls,topic,lambda m:self.put(key,extract(m)),qos_profile_sensor_data)
        sub(PoseStamped,'/simulation/local_position/pose','world_z',lambda m:m.pose.position.z)
        sub(PoseStamped,'/mavros/local_position/pose','local_z',lambda m:m.pose.position.z)
        sub(PoseStamped,'/mavros/setpoint_position/local','setpoint_z',lambda m:m.pose.position.z)
        sub(Range,'/simulation/rangefinder','rangefinder',lambda m:m.range)
        sub(Float32,'/control/measured_agl','filtered_agl',lambda m:m.data)
        self.create_subscription(PointCloud2,'/livox/top/points',self.lidar,qos_profile_sensor_data)
        self.create_timer(0.2,self.record)
        self.get_logger().info(f'Per-tree hover/LiDAR output: {self.directory}')

    def put(self,key,value): self.values[key]=(float(value),time.monotonic())
    def target(self,msg):
        if msg.data>=0:self.tree_id=msg.data
    def status(self,msg): self.state=msg.data
    def lidar_estimate(self,msg):
        if len(msg.data)==6:
            self.lidar_estimate_ids.add(int(msg.data[0]))
            self.estimates[int(msg.data[0])]=(msg.data[3],msg.data[4],time.monotonic())

    def estimate(self,msg):
        if len(msg.data)==3 and int(msg.data[0]) not in self.lidar_estimate_ids:self.estimates[int(msg.data[0])]=(msg.data[1],msg.data[2],time.monotonic())
    def lidar(self,msg):self.cloud=msg;self.cloud_time=time.monotonic()

    def record(self):
        if self.tree_id<0 or self.state not in ('VERIFY_TREE','START_ORBIT','POST_ORBIT_HOVER'):return
        now=time.monotonic();estimate=self.estimates.get(self.tree_id,(math.nan,math.nan,math.nan))
        row={k:(v[0] if now-v[1]<1.0 else math.nan) for k,v in self.values.items()}
        row.update(height_source='lidar' if self.tree_id in self.lidar_estimate_ids else 'camera',wall_time=time.time(),tree_id=self.tree_id,state=self.state,observed_base_z=estimate[0],observed_height=estimate[1],estimate_age_s=now-estimate[2])
        self.writer.writerow({k:row.get(k,math.nan) for k in self.columns});self.file.flush()
        key=(self.tree_id,self.state)
        if key in self.saved or self.cloud is None or now-self.cloud_time>1.0:return
        raw=point_cloud2.read_points(self.cloud,field_names=('x','y','z'),skip_nans=True)
        if isinstance(raw,np.ndarray) and raw.dtype.names:
            points=np.column_stack([raw[name].reshape(-1) for name in ('x','y','z')]).astype(float)
        else:
            points=np.asarray(list(raw),dtype=float)
        if points.size==0:return
        self.saved.add(key);folder=self.directory/f'tree_{self.tree_id:04d}';folder.mkdir(exist_ok=True)
        np.savez_compressed(folder/f'{self.state}_lidar.npz',xyz=points)
        (folder/f'{self.state}_metadata.json').write_text(json.dumps(dict(frame_id=self.cloud.header.frame_id,stamp_sec=self.cloud.header.stamp.sec,stamp_nanosec=self.cloud.header.stamp.nanosec,observed_base_z=estimate[0],observed_height=estimate[1],estimate_age_s=now-estimate[2],height_source='lidar' if self.tree_id in self.lidar_estimate_ids else 'camera',note='Raw LiDAR scene in sensor frame; not an isolated tree image.'),indent=2))
        display=points[::max(1,len(points)//12000)]
        fig,axes=plt.subplots(1,2,figsize=(12,5))
        for ax,indices,labels in [(axes[0],(0,1),('X','Y')),(axes[1],(0,2),('X','Z'))]:
            ax.scatter(display[:,indices[0]],display[:,indices[1]],c=display[:,2],s=1,cmap='viridis');ax.set(xlabel=labels[0]+' (m)',ylabel=labels[1]+' (m)');ax.axis('equal');ax.grid(alpha=.3)
        fig.suptitle(f'Tree {self.tree_id}: {self.state}, LiDAR frame {self.cloud.header.frame_id}');fig.tight_layout();fig.savefig(folder/f'{self.state}_lidar.png',dpi=150);plt.close(fig)


def main():
    rclpy.init();node=TreeHoverRecorder()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:node.file.close();node.destroy_node();rclpy.shutdown()
