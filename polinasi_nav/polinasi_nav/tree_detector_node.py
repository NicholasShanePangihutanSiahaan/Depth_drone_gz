"""Read-only perception process. It owns no flight/control publishers."""
import json
import math
import multiprocessing
from concurrent.futures import ProcessPoolExecutor
from collections import deque
import numpy as np
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener, TransformException
from rclpy.time import Time
from .ros_nodes import ConfigNode, run, cloud_points, rotation, vector, seconds
from .tree_perception import StemDetector

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
        self.pending = deque(maxlen=1)
        self.future = None
        self.last_submit = -math.inf
        self.worker = ProcessPoolExecutor(max_workers=1,mp_context=multiprocessing.get_context('spawn'),initializer=initialize)
        self.pub = self.create_publisher(String,'/perception/trees',1)
        self.create_subscription(PointCloud2,'/livox/lidar',self.pending.append,qos_profile_sensor_data)
        self.create_timer(.2,self.tick)

    def tick(self):
        if self.future is not None:
            if not self.future.done():
                return
            try:
                self.pub.publish(String(data=json.dumps(self.future.result(),allow_nan=False)))
            except Exception as exc:
                self.get_logger().error('Tree detection failed: '+str(exc))
            self.future = None
        if not self.pending or self.now()-self.last_submit < 1.:
            return
        msg = self.pending[-1]
        stamp = seconds(msg.header.stamp)
        if msg.header.frame_id != 'mid360' or not 0 <= self.now()-stamp < self.c['cloud_timeout']:
            return
        try:
            tf = self.buffer.lookup_transform('map','mid360',Time.from_msg(msg.header.stamp))
        except TransformException:
            return
        points = cloud_points(msg)
        ranges = np.linalg.norm(points,axis=1)
        points = points[np.all(np.isfinite(points),axis=1) & (ranges > 1.) & (ranges < self.c['lidar_range']-.05)]
        world = points@rotation(tf.transform.rotation).T+vector(tf.transform.translation)
        self.future = self.worker.submit(detect,world,stamp)
        self.last_submit = self.now()

    def destroy_node(self):
        self.worker.shutdown(wait=True,cancel_futures=True)
        return super().destroy_node()

def main(args=None):
    run(TreeDetectorNode,args)
