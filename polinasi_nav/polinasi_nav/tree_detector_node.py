"""Read-only perception process. It owns no flight/control publishers."""
import json
import math
import multiprocessing
from concurrent.futures import ProcessPoolExecutor
import numpy as np
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener, TransformException
from rclpy.time import Time
from .ros_nodes import ConfigNode, run, cloud_points, rotation, vector, seconds
from .tree_perception import StemDetector
from .depth_queue import DepthTransformQueue

DETECTOR = None

def initialize():
    global DETECTOR
    DETECTOR = StemDetector()

def detect(points, stamp):
    return DETECTOR.update(points,stamp)

class TreeDetectorNode(ConfigNode):
    def __init__(self):
        super().__init__('polinasi_tree_detector')
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer,self)
        self.pending = DepthTransformQueue(6)
        self.last_submitted_stamp = -math.inf
        self.last_queue_log = -math.inf
        self.future = None
        self.last_submit = -math.inf
        self.worker = ProcessPoolExecutor(max_workers=1,mp_context=multiprocessing.get_context('spawn'),initializer=initialize)
        self.pub = self.create_publisher(String,'/perception/trees',1)
        self.create_subscription(PointCloud2,'/livox/lidar',self.receive,qos_profile_sensor_data)
        self.create_timer(.2,self.tick)

    def receive(self,msg):
        if msg.header.frame_id == 'mid360':
            self.pending.push(seconds(msg.header.stamp),msg,self.now())

    def exact_transform(self,msg):
        try:
            return self.buffer.lookup_transform('map','mid360',Time.from_msg(msg.header.stamp))
        except TransformException:
            return None

    def tick(self):
        if self.future is not None:
            if not self.future.done():
                return
            try:
                result=self.future.result()
                result['transform_queue']=self.pending.status()
                self.pub.publish(String(data=json.dumps(result,allow_nan=False)))
            except Exception as exc:
                self.get_logger().error('Tree detection failed: '+str(exc))
            self.future = None
        now=self.now()
        if now-self.last_submit < 1.:
            return
        selected=self.pending.select(now,self.c['cloud_timeout'],self.last_submitted_stamp,self.exact_transform)
        if now-self.last_queue_log>=1.:
            self.get_logger().info('Tree scan transform queue: '+json.dumps(self.pending.status()))
            self.last_queue_log=now
        if selected is None:
            return
        msg,tf=selected
        stamp=seconds(msg.header.stamp)
        points = cloud_points(msg)
        ranges = np.linalg.norm(points,axis=1)
        points = points[np.all(np.isfinite(points),axis=1) & (ranges > 1.) & (ranges < self.c['lidar_range']-.05)]
        world = points@rotation(tf.transform.rotation).T+vector(tf.transform.translation)
        self.future = self.worker.submit(detect,world,stamp)
        self.last_submit = now
        self.last_submitted_stamp=stamp
        self.pending.acknowledge(stamp)

    def destroy_node(self):
        self.worker.shutdown(wait=True,cancel_futures=True)
        return super().destroy_node()

def main(args=None):
    run(TreeDetectorNode,args)
