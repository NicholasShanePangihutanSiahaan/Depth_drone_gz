#!/usr/bin/env python3
"""SITL only: after manual takeoff, follow the alley to x=-50 and hold."""
import copy
import csv
import math
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from std_msgs.msg import Bool, Float32, String


class StraightTest(Node):
    def __init__(self):
        super().__init__('terrain_straight_test')
        self.pose = self.state = None
        self.pose_time = self.range_time = 0.0
        self.agl = math.nan
        self.status = ''
        self.start = None
        self.finished = False
        self.create_subscription(PoseStamped, '/simulation/local_position/pose', self.on_pose, qos_profile_sensor_data)
        self.create_subscription(State, '/mavros/state', self.on_state, 10)
        self.create_subscription(Float32, '/control/terrain/measured_agl', self.on_agl, 10)
        self.create_subscription(String, '/control/terrain/status', self.on_status, 10)
        self.target_pub = self.create_publisher(PoseStamped, '/control/safe_target_pose', 10)
        self.agl_pub = self.create_publisher(Float32, '/control/target_agl', 10)
        self.enable_pub = self.create_publisher(Bool, '/control/setpoint_enabled', 10)
        path = Path('/tmp') / time.strftime('terrain_straight_%Y%m%d_%H%M%S.csv')
        self.file = path.open('w', newline='')
        self.writer = csv.writer(self.file)
        self.writer.writerow(['wall_time', 'x', 'y', 'z', 'agl', 'agl_error', 'terrain_status'])
        self.get_logger().info(f'SITL ONLY. Waiting for GUIDED, armed, and takeoff. CSV: {path}')
        self.create_timer(0.1, self.loop)

    def on_pose(self, msg):
        self.pose, self.pose_time = msg, time.monotonic()

    def on_state(self, msg):
        self.state = msg

    def on_agl(self, msg):
        self.agl, self.range_time = msg.data, time.monotonic()

    def on_status(self, msg):
        self.status = msg.data

    def loop(self):
        now = time.monotonic()
        if self.pose is None or self.state is None or now-self.pose_time > 1.0:
            return  # Controller target timeout provides hold.
        if not self.state.armed or self.state.mode != 'GUIDED':
            return
        p = self.pose.pose.position
        if self.start is None:
            if p.z < 1.2:
                return
            if abs(p.x) > 2 or abs(p.y+3) > 1:
                self.get_logger().error('Start must be near (0, -3); no target sent.', once=True)
                return
            self.start = copy.deepcopy(self.pose)
            self.get_logger().info('Flying toward x=-50, y=-3. Holding at endpoint; land manually.')
        target = copy.deepcopy(self.start)
        target.header.stamp = self.get_clock().now().to_msg()
        # Small lookahead from actual position: progress pauses while vehicle holds.
        target.pose.position.x = max(-50.0, p.x-0.3)
        target.pose.orientation.x = 0.0
        target.pose.orientation.y = 0.0
        target.pose.orientation.z = 1.0
        target.pose.orientation.w = 0.0
        target.pose.position.y = -3.0
        target.pose.position.z = 1.5
        if p.x <= -49.7:
            target.pose.position.x = -50.0
            if not self.finished:
                self.finished = True
                self.get_logger().info('Endpoint reached. Holding; use LAND when ready.')
        self.agl_pub.publish(Float32(data=1.5))
        self.enable_pub.publish(Bool(data=True))
        self.target_pub.publish(target)
        agl = self.agl if now-self.range_time < 1 else math.nan
        self.writer.writerow([time.time(), p.x, p.y, p.z, agl, 1.5-agl, self.status])
        self.file.flush()


def main():
    rclpy.init()
    node = StraightTest()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.file.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
