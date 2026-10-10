#!/usr/bin/env python3
"""Opt-in SITL integration experiment; starts/stops only its own process groups.

Run after sourcing ROS and this workspace. Never run beside another SITL using
ports 5760/9002. Ground truth and fixture geometry are scoring inputs only.
"""
import argparse
import csv
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time
import xml.etree.ElementTree as ET

import rclpy
from nav_msgs.msg import Odometry
from mavros_msgs.srv import StreamRate
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String
from uav_interfaces.msg import TreeArray


class Monitor(Node):
    def __init__(self, folder, world):
        super().__init__('depth_avoidance_sitl_test')
        self.state = 'INIT'
        self.states = []
        self.status = {}
        self.inspected = set()
        self.start_time = time.monotonic()
        self.min_fixture_surface = float('inf')
        self.min_trunk_surface = float('inf')
        self.fixture_encounters = 0
        self.last_record = 0.0
        self.latest_pose = None
        self.stream_client = self.create_client(StreamRate, '/mavros/set_stream_rate')
        self.stream_requested = False
        self.tree_xy = []
        self.finished = False
        root = ET.parse(world).getroot()
        for item in root.findall('./world/include'):
            if (item.findtext('name') or '').startswith('tree_'):
                xy = list(map(float, item.findtext('pose').split()[:2]))
                self.tree_xy.append(xy)
        self.csv_file = (folder/'trajectory.csv').open('w')
        self.writer = csv.writer(self.csv_file)
        self.writer.writerow(['elapsed', 'state', 'avoidance', 'x', 'y', 'z',
                              'fixture_surface_distance', 'nearest_trunk_surface_distance'])
        self.create_subscription(Odometry, '/simulation/ground_truth/odom',
                                 self.pose_cb, qos_profile_sensor_data)
        self.create_subscription(String, '/mission/fsm_state', self.state_cb, 10)
        self.create_subscription(String, '/avoidance/status', self.status_cb, 10)
        self.create_subscription(TreeArray, '/map/trees', self.trees_cb, 10)

    def state_cb(self, msg):
        if msg.data != self.state:
            self.states.append((round(time.monotonic()-self.start_time, 2), msg.data))
        self.state = msg.data

    def status_cb(self, msg):
        self.status = json.loads(msg.data)

    def trees_cb(self, msg):
        self.inspected.update(t.id for t in msg.trees if t.inspected)

    def pose_cb(self, msg):
        now = time.monotonic()
        p = msg.pose.pose.position
        self.latest_pose = (p.x, p.y, p.z)
        if now-self.last_record < 0.1:
            return
        self.last_record = now
        distances = []
        for x, y in ((7, -2.6), (10.5, -8.66)):
            # Signed distance to horizontal rectangle, valid at test altitude.
            dx, dy = abs(p.x-x)-0.09, abs(p.y-y)-0.5
            distances.append(math.hypot(max(dx, 0), max(dy, 0))
                             + min(max(dx, dy), 0))
        fixture_distance = min(distances)
        trunk_distance = min(math.hypot(p.x-x, p.y-y)-0.55 for x, y in self.tree_xy)
        if p.z > 0.6:
            self.min_fixture_surface = min(self.min_fixture_surface, fixture_distance)
            self.min_trunk_surface = min(self.min_trunk_surface, trunk_distance)
        self.writer.writerow([round(now-self.start_time, 3), self.state,
                              self.status.get('status', ''), p.x, p.y, p.z,
                              fixture_distance, trunk_distance])
        self.csv_file.flush()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', required=True)
    parser.add_argument('--timeout', type=float, default=1000.0)
    parser.add_argument('--trees', type=int, default=3)
    parser.add_argument('--workspace', default='/home/shane/ProjekAtaka/gazebo_sim')
    parser.add_argument('--ardupilot', default='/home/shane/ardupilot')
    args = parser.parse_args()
    folder = Path(args.output).resolve()
    folder.mkdir(parents=True, exist_ok=False)
    workspace, ardupilot = Path(args.workspace), Path(args.ardupilot)
    env = dict(os.environ, ROS_LOG_DIR=str(folder/'ros'),
               GZ_HOMEDIR=str(folder/'gz'))
    processes = []
    handles = []

    def start(name, command):
        handle = (folder/f'{name}.log').open('w')
        handles.append(handle)
        process = subprocess.Popen(command, cwd=folder, env=env, stdout=handle,
                                   stderr=subprocess.STDOUT, start_new_session=True)
        processes.append(process)
        return process

    rclpy.init()
    node = Monitor(folder, workspace/'src/uav_plantation_sim/worlds/plantation_737c519.sdf')
    try:
        start('gazebo', ['ros2', 'launch', 'uav_plantation_sim', 'plantation_sim.launch.py',
                        'world:=plantation_737c519.sdf', 'avoidance_fixture:=true',
                        'gz_args:=-s -r -v 1'])
        defaults = ','.join(map(str, [ardupilot/'Tools/autotest/default_params/copter.parm',
                                      ardupilot/'Tools/autotest/default_params/gazebo-iris.parm',
                                      workspace/'src/beehive_drone/config/sitl_sim.parm']))
        start('sitl', [str(ardupilot/'build/sitl/bin/arducopter'), '--model', 'JSON',
                       '--defaults', defaults, '-w', '--speedup', '1',
                       '--sim-address=127.0.0.1', '-I0'])
        start('mavros', ['ros2', 'launch', 'mavros', 'apm.launch',
                         'fcu_url:=tcp://127.0.0.1:5760'])
        start('mission', ['ros2', 'launch', 'beehive_drone', 'gazebo_stack.launch.py',
                          'tree_source:=sdf', 'tree_world:=plantation_737c519.sdf',
                          f'source_tree_limit:={args.trees}', 'mission_mode:=multi_tree',
                          f'max_trees:={args.trees}', 'expected_tree_count:=1',
                          'auto_start:=true', 'enable_depth_avoidance:=true',
                          f'report_output_directory:={folder}/mission_report'])
        last_print = 0.0
        while time.monotonic()-node.start_time < args.timeout:
            rclpy.spin_once(node, timeout_sec=0.1)
            now = time.monotonic()
            if (not node.stream_requested and now-node.start_time > 10
                    and node.stream_client.service_is_ready()):
                node.stream_client.call_async(StreamRate.Request(
                    stream_id=0, message_rate=20, on_off=True))
                node.stream_requested = True
            if now-last_print > 10:
                print(json.dumps({'elapsed': round(now-node.start_time), 'state': node.state,
                                  'position': node.latest_pose, 'inspected': len(node.inspected),
                                  'avoidance': node.status}), flush=True)
                last_print = now
            if node.state in ('DONE', 'ABORT', 'MANUAL_OVERRIDE'):
                break
        result = {'state': node.state, 'states': node.states,
                  'trees_inspected': sorted(node.inspected),
                  'minimum_fixture_surface_distance_m': node.min_fixture_surface,
                  'minimum_trunk_surface_distance_m': node.min_trunk_surface,
                  'vehicle_radius_m': 0.4, 'last_avoidance': node.status,
                  'passed': (node.state == 'DONE' and len(node.inspected) == args.trees
                             and node.min_fixture_surface > 0.4
                             and node.min_trunk_surface > 0.4)}
        (folder/'result.json').write_text(json.dumps(result, indent=2)+'\n')
        print(json.dumps(result), flush=True)
    finally:
        node.csv_file.close()
        for process in reversed(processes):
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
        for process in reversed(processes):
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGTERM)
        for handle in handles:
            handle.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
