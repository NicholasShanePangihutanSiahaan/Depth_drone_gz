"""Depth-based local navigation supervisor; a single safe-target publisher.

Conservative fixed-altitude navigation: A* candidate paths use remembered depth
hits, then each commanded segment must be seen by the forward camera. Cannot
guarantee coverage in an enclosed region: unavailable routes result in HOLD.
"""

from collections import deque
from copy import deepcopy
import json
import math
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import Path
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Bool, String

from beehive_drone.depth_navigation import (
    DepthView, LocalGrid, ObstacleMemory, decode_depth, rotation, wrap,
)


def stamp_seconds(stamp):
    return stamp.sec+stamp.nanosec*1e-9


class DepthAvoidanceController(Node):
    def __init__(self):
        super().__init__('depth_avoidance_controller')
        defaults = {
            'depth_topic': '/zed2i/depth/depth_registered',
            'camera_info_topic': '/zed2i/depth/camera_info',
            'depth_timeout': 0.8,
            'pose_timeout': 0.5,
            'goal_timeout': 1.0,
            'max_pose_skew': 0.15,
            'use_receipt_time': False,
            'inf_is_clear': False,
            'camera_translation': [0.15, 0.0, 0.02],
            'camera_rpy_degrees': [0.0, 0.0, 0.0],
            'vehicle_radius': 0.40,
            'clearance_margin': 0.35,
            'vehicle_half_height': 0.30,
            'grid_resolution': 0.20,
            'planning_radius': 7.0,
            'max_depth': 10.0,
            'lookahead': 0.65,
            'yaw_tolerance_degrees': 12.0,
            'blocked_timeout': 30.0,
            'orbit_radius': 2.6,
            'pose_jump_distance': 1.0,
            'pose_jump_yaw_degrees': 45.0,
        }
        for name, default in defaults.items():
            self.declare_parameter(name, default)
        self.p = {name: self.get_parameter(name).value for name in defaults}
        if not (self.p['vehicle_radius'] > 0 and self.p['clearance_margin'] > 0
                and self.p['grid_resolution'] > 0 and self.p['lookahead'] > 0
                and self.p['planning_radius'] > 2*self.p['lookahead']):
            raise ValueError('invalid avoidance geometry parameters')
        roll, pitch, yaw = np.radians(self.p['camera_rpy_degrees'])
        cr, sr = math.cos(roll/2), math.sin(roll/2)
        cp, sp = math.cos(pitch/2), math.sin(pitch/2)
        cy, sy = math.cos(yaw/2), math.sin(yaw/2)
        self.mount = rotation((sr*cp*cy-cr*sp*sy, cr*sp*cy+sr*cp*sy,
                               cr*cp*sy-sr*sp*cy, cr*cp*cy+sr*sp*sy))
        self.memory = ObstacleMemory(radius=self.p['planning_radius']+4.0)
        self.poses = deque(maxlen=120)
        self.pose = None
        self.pose_time = -float('inf')
        self.depth_time = -float('inf')
        self.last_stamp = None
        self.info = None
        self.view = None
        self.state = 'INIT'
        self.goals = {}
        self.goal_times = {}
        self.orbit_centre = None
        self.waypoint = None
        self.detour_goal = None
        self.hold_pose = None
        self.last_status = None
        self.last_status_log = 0.0
        self.blocked_since = None
        self.latched_fault = None
        self.frame = None
        self.events = 0
        self.obstacle_points = 0
        self.last_camera_error = ''
        self.create_subscription(PoseStamped, '/mavros/local_position/pose',
                                 self.pose_cb, qos_profile_sensor_data)
        self.create_subscription(Image, self.p['depth_topic'], self.depth_cb,
                                 QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.create_subscription(CameraInfo, self.p['camera_info_topic'],
                                 self.info_cb, qos_profile_sensor_data)
        self.create_subscription(String, '/mission/fsm_state', self.state_cb, 10)
        self.create_subscription(PoseStamped, '/navigation/local_goal',
                                 lambda m: self.goal_cb('fsm', m), 10)
        self.create_subscription(PoseStamped, '/control/dynamic_target',
                                 lambda m: self.goal_cb('orbit', m), 10)
        self.create_subscription(Point, '/control/orbit_target',
                                 self.orbit_cb, 10)
        self.target_pub = self.create_publisher(PoseStamped, '/control/safe_target_pose', 10)
        self.hold_pub = self.create_publisher(Bool, '/avoidance/hold', 10)
        self.pause_pub = self.create_publisher(Bool, '/avoidance/orbit_pause', 10)
        self.status_pub = self.create_publisher(String, '/avoidance/status', 10)
        self.ready_pub = self.create_publisher(Bool, '/avoidance/ready', 10)
        self.path_pub = self.create_publisher(Path, '/avoidance/path', 10)
        self.create_timer(0.1, self.tick)
        self.get_logger().info('Depth navigation: inflated A*, observe-before-move, fixed local Z.')

    def pose_cb(self, msg):
        now = time.monotonic()
        if self.frame is not None and msg.header.frame_id != self.frame:
            self.latched_fault = 'POSE_FRAME_CHANGED'
        self.frame = msg.header.frame_id
        if self.pose is not None and now-self.pose_time < 0.5:
            a, b = self.pose.pose.position, msg.pose.position
            qa, qb = self.pose.pose.orientation, msg.pose.orientation
            dyaw = wrap(self.yaw(qb)-self.yaw(qa))
            if (math.dist((a.x, a.y, a.z), (b.x, b.y, b.z)) > self.p['pose_jump_distance']
                    or abs(dyaw) > math.radians(self.p['pose_jump_yaw_degrees'])):
                self.latched_fault = 'POSE_RESET_RESTART_REQUIRED'
        self.pose, self.pose_time = msg, now
        self.poses.append((stamp_seconds(msg.header.stamp), now, deepcopy(msg.pose)))

    @staticmethod
    def yaw(q):
        return math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))

    def info_cb(self, msg):
        self.info = msg

    def state_cb(self, msg):
        if msg.data != self.state:
            self.waypoint = None
            self.detour_goal = None
            self.hold_pose = None
            self.blocked_since = None
        self.state = msg.data

    def goal_cb(self, key, msg):
        self.goals[key] = msg
        self.goal_times[key] = time.monotonic()

    def orbit_cb(self, msg):
        self.orbit_centre = np.array([msg.x, msg.y])
        self.detour_goal = None

    def depth_cb(self, msg):
        if not self.poses or self.info is None:
            return
        stamp = stamp_seconds(msg.header.stamp)
        if self.last_stamp is not None and stamp <= self.last_stamp:
            if stamp < self.last_stamp:
                self.latched_fault = 'DEPTH_CLOCK_RESET_RESTART_REQUIRED'
            return  # Repeated frames cannot refresh the watchdog.
        now = time.monotonic()
        if self.p['use_receipt_time']:
            match = self.poses[-1]
            skew = now-match[1]
        else:
            match = min(self.poses, key=lambda item: abs(item[0]-stamp))
            skew = abs(match[0]-stamp)
        if skew > self.p['max_pose_skew']:
            self.last_camera_error = 'DEPTH_POSE_UNSYNCHRONIZED'
            return
        try:
            if (msg.width, msg.height) != (self.info.width, self.info.height):
                raise ValueError('CameraInfo/depth dimensions mismatch')
            depth = decode_depth(msg.data, msg.width, msg.height, msg.step,
                                 msg.encoding, msg.is_bigendian)
            valid = np.isfinite(depth) & (depth >= 0.2)
            if self.p['inf_is_clear']:
                valid |= np.isposinf(depth)
            if np.count_nonzero(valid) < depth.size*0.05:
                raise ValueError('depth invalid/insufficient coverage')
            pose = match[2]
            pos = np.array([pose.position.x, pose.position.y, pose.position.z])
            q = pose.orientation
            body = rotation((q.x, q.y, q.z, q.w))
            origin = pos+body @ np.asarray(self.p['camera_translation'])
            k = self.info.k
            view = DepthView(depth, (k[0], k[4], k[2], k[5]), origin,
                             body @ self.mount, max_range=self.p['max_depth'],
                             inf_is_clear=self.p['inf_is_clear'])
            self.memory.update(view, pos)
            self.view = view
            self.last_stamp, self.depth_time = stamp, now
            self.last_camera_error = ''
        except (ValueError, TypeError, IndexError) as exc:
            self.last_camera_error = str(exc)

    def output(self, goal, status, hold, paused, path=None):
        now = time.monotonic()
        if hold:
            if self.hold_pose is None:
                self.hold_pose = deepcopy(self.pose)
            target = deepcopy(self.hold_pose)
            target.pose.orientation = deepcopy(goal.pose.orientation)
        else:
            self.hold_pose = None
            target = deepcopy(goal)
        target.header.stamp = self.get_clock().now().to_msg()
        target.header.frame_id = self.frame or 'map'
        self.hold_pub.publish(Bool(data=hold))
        self.pause_pub.publish(Bool(data=paused))
        self.target_pub.publish(target)
        if status != self.last_status:
            if status == 'DETOUR':
                self.events += 1
            if now-self.last_status_log > 1.0:
                self.get_logger().info(f'{status}: state={self.state}, obstacles={self.obstacle_points}')
                self.last_status_log = now
            self.last_status = status
        self.status_pub.publish(String(data=json.dumps({
            'status': status, 'holding': hold, 'orbit_paused': paused,
            'depth_age_s': round(now-self.depth_time, 3) if self.view else None,
            'obstacle_voxels': self.obstacle_points, 'detour_events': self.events,
            'fsm_state': self.state, 'camera_error': self.last_camera_error,
        })))
        if path is not None:
            message = Path()
            message.header = target.header
            for xy in path:
                point = deepcopy(target)
                point.pose.position.x, point.pose.position.y = map(float, xy)
                message.poses.append(point)
            self.path_pub.publish(message)

    def tick(self):
        if self.pose is None:
            self.ready_pub.publish(Bool(data=False))
            return
        now = time.monotonic()
        healthy = (self.latched_fault is None
                   and now-self.pose_time <= self.p['pose_timeout']
                   and now-self.depth_time <= self.p['depth_timeout'])
        self.ready_pub.publish(Bool(data=healthy))
        orbit = self.state in ('START_ORBIT', 'WAIT_ORBIT')
        key = 'orbit' if orbit else 'fsm'
        goal = deepcopy(self.goals.get(key, self.pose))
        if self.state in ('INIT', 'WAIT_START', 'WAIT_GUIDED', 'WAIT_ARM',
                          'WAIT_TAKEOFF', 'LANDING', 'DONE', 'ABORT', 'MANUAL_OVERRIDE'):
            self.output(goal, 'STANDBY', True, True)
            return
        fault = self.latched_fault
        if now-self.pose_time > self.p['pose_timeout']:
            fault = 'POSE_STALE'
        elif now-self.depth_time > self.p['depth_timeout']:
            fault = 'DEPTH_STALE'
        elif now-self.goal_times.get(key, -float('inf')) > self.p['goal_timeout']:
            fault = 'GOAL_STALE'
        if fault:
            self.output(goal, fault, True, True)
            return
        p = self.pose.pose.position
        start = np.array([p.x, p.y])
        desired = np.array([goal.pose.position.x, goal.pose.position.y])
        obstacles = self.memory.slice(p.z, self.p['vehicle_half_height'])
        self.obstacle_points = len(obstacles)
        clearance = self.p['vehicle_radius']+self.p['clearance_margin']
        grid = LocalGrid(start, obstacles, self.p['grid_resolution'],
                         self.p['planning_radius'], clearance)
        # Preserve the nominal orbit's inner boundary; a shortcut through the
        # selected trunk never constitutes a successful avoidance manoeuvre.
        if orbit and self.orbit_centre is not None:
            xs = grid.origin[0]+np.arange(grid.size)*grid.resolution
            ys = grid.origin[1]+np.arange(grid.size)*grid.resolution
            inner = self.p['orbit_radius']-0.45
            grid.blocked |= ((xs[:, None]-self.orbit_centre[0])**2
                             + (ys[None, :]-self.orbit_centre[1])**2 < inner**2)
        if not grid.free(grid.cell(start)):
            self.output(goal, 'CLEARANCE_HOLD', True, True)
            return
        if np.linalg.norm(desired-start) < 0.12 and not orbit:
            self.waypoint = None
            self.output(goal, 'AT_GOAL', False, False)
            return
        if self.detour_goal is not None:
            if np.linalg.norm(start-self.detour_goal) < 0.3:
                self.detour_goal = None
                self.waypoint = None
            else:
                desired = self.detour_goal
        delta = desired-start
        horizon = self.p['planning_radius']-1.0
        if np.linalg.norm(delta) > horizon:
            desired = start+delta/np.linalg.norm(delta)*horizon
        path = grid.plan(start, desired)
        # A nominal orbital point can itself be occupied by a frond. Rejoin
        # the circle farther CCW, after planning every leg around the obstacle.
        if not path and orbit and self.orbit_centre is not None:
            angle = math.atan2(*(start-self.orbit_centre)[::-1])
            for advance in (30, 50, 70, 90):
                theta = angle+math.radians(advance)
                candidate = self.orbit_centre+self.p['orbit_radius']*np.array([
                    math.cos(theta), math.sin(theta)])
                path = grid.plan(start, candidate)
                if path:
                    self.detour_goal = candidate
                    desired = candidate
                    break
        if not path:
            if self.blocked_since is None:
                self.blocked_since = now
            status = 'NO_PATH_HOLD'
            if now-self.blocked_since > self.p['blocked_timeout']:
                self.latched_fault = status = 'BLOCKED_RESTART_REQUIRED'
            self.output(goal, status, True, True, [])
            return
        self.blocked_since = None
        if (self.waypoint is None or np.linalg.norm(start-self.waypoint) < 0.20
                or not grid.segment_free(start, self.waypoint)
                or np.linalg.norm(desired-start) < 0.8):
            self.waypoint = grid.next_waypoint(start, path, self.p['lookahead'])
        direction = self.waypoint-start
        heading = math.atan2(direction[1], direction[0])
        goal.pose.orientation.x = goal.pose.orientation.y = 0.0
        goal.pose.orientation.z = math.sin(heading/2)
        goal.pose.orientation.w = math.cos(heading/2)
        yaw_error = abs(wrap(heading-self.yaw(self.pose.pose.orientation)))
        if yaw_error > math.radians(self.p['yaw_tolerance_degrees']):
            self.output(goal, 'LOOK_BEFORE_MOVE', True, True, path)
            return
        endpoint = np.array([*self.waypoint, p.z])
        if not self.view.corridor_seen(np.array([p.x, p.y, p.z]), endpoint,
                                       self.p['vehicle_radius'],
                                       self.p['vehicle_half_height']):
            self.waypoint = None
            self.output(goal, 'UNOBSERVED_HOLD', True, True, path)
            return
        goal.pose.position.x, goal.pose.position.y = map(float, self.waypoint)
        detour = len(path) > 2 or self.detour_goal is not None
        self.output(goal, 'DETOUR' if detour else 'TRACKING', False,
                    self.detour_goal is not None, path)


def main(args=None):
    rclpy.init(args=args)
    node = DepthAvoidanceController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
