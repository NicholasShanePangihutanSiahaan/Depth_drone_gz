#!/usr/bin/env python3
"""Sensor freshness and rangefinder watchdog; never commands the vehicle."""

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from sensor_msgs.msg import PointCloud2, Range
from std_msgs.msg import Bool, String


class MissionSafetyMonitor(Node):
    def __init__(self):
        super().__init__('mission_safety_monitor')
        defaults = {'pointcloud_topic': '/zed2i/depth/points',
                    'range_topic': '/mavros/rangefinder/rangefinder',
                    'require_rangefinder': False, 'pose_timeout': 1.0,
                    'cloud_timeout': 3.0, 'state_timeout': 1.0,
                    'range_timeout': 1.0, 'min_range': 0.08, 'max_range': 20.0}
        for key, value in defaults.items():
            self.declare_parameter(key, value)
        self.last = {'pose': None, 'cloud': None, 'state': None, 'range': None}
        self.range_valid = False
        self.connected = False
        cloud_topic = str(self.get_parameter('pointcloud_topic').value)
        range_topic = str(self.get_parameter('range_topic').value)
        self.create_subscription(PoseStamped, '/mavros/local_position/pose',
                                 lambda _: self.touch('pose'), qos_profile_sensor_data)
        self.create_subscription(PointCloud2, cloud_topic,
                                 lambda _: self.touch('cloud'), qos_profile_sensor_data)
        self.create_subscription(State, '/mavros/state', self.state_cb, 10)
        self.create_subscription(Range, range_topic, self.range_cb, qos_profile_sensor_data)
        self.ok_pub = self.create_publisher(Bool, '/mission/safety_ok', 10)
        self.reason_pub = self.create_publisher(String, '/mission/safety_reason', 10)
        self.create_timer(0.1, self.loop)

    def touch(self, name):
        self.last[name] = self.get_clock().now()

    def state_cb(self, msg):
        self.connected = msg.connected
        self.touch('state')

    def range_cb(self, msg):
        low = float(self.get_parameter('min_range').value)
        high = float(self.get_parameter('max_range').value)
        self.range_valid = math.isfinite(msg.range) and low <= msg.range <= high
        self.touch('range')

    def fresh(self, name, timeout):
        return self.last[name] is not None and \
            (self.get_clock().now() - self.last[name]).nanoseconds * 1e-9 <= timeout

    def loop(self):
        failures = []
        for name in ('pose', 'cloud', 'state'):
            timeout = float(self.get_parameter(f'{name}_timeout').value)
            if not self.fresh(name, timeout):
                failures.append(f'{name}_stale')
        if not self.connected:
            failures.append('fcu_disconnected')
        if bool(self.get_parameter('require_rangefinder').value):
            if not self.fresh('range', float(self.get_parameter('range_timeout').value)):
                failures.append('range_stale')
            elif not self.range_valid:
                failures.append('range_invalid')
        ok = Bool(); ok.data = not failures
        reason = String(); reason.data = 'OK' if not failures else ','.join(failures)
        self.ok_pub.publish(ok)
        self.reason_pub.publish(reason)


def main(args=None):
    rclpy.init(args=args)
    node = MissionSafetyMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
