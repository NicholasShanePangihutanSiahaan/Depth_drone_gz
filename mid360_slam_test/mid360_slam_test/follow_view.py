"""Gravity-referenced visualization using matched, healthy local SLAM poses.

No new localization or control: the existing estimator initializes roll/pitch
from stationary IMU gravity and estimates subsequent movement using ICP/IMU.
"""
from collections import deque
import json
import math
import time

import numpy as np
from scipy.spatial.transform import Rotation
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header, String
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray

from mid360_test.nodes import xyz, stamp_s, sensor_qos, json_message, run
from mid360_test.core.recording import atomic_json
from .geometry import unpack, transform
from .nodes import SlamNode, latched, vector
from .self_filter import BodySphere


def follow_transforms(local_pose, center_lidar_m, pitch_down_deg):
    """World-aligned camera target and physical drone orientation, same origin."""
    follow = np.eye(4)
    follow[:3, 3] = transform(local_pose, np.asarray(center_lidar_m).reshape(1, 3))[0]
    body = follow.copy()
    body[:3, :3] = local_pose[:3, :3] @ Rotation.from_euler(
        'y', -pitch_down_deg, degrees=True).as_matrix()
    return follow, body


class FollowView(SlamNode):
    def __init__(self):
        super().__init__('follow_view')
        self.sphere = BodySphere(self.c['lidar_mount_pitch_down_deg'], self.c['drone_center_a_cm'],
            self.c['drone_center_b_cm'], self.c['body_sphere_radius_cm'],
            self.c['body_sphere_filter_enabled'], center_mode=self.c['body_sphere_center_mode'],
            center_lidar_cm=self.c['body_sphere_center_lidar_cm'])
        self.clouds, self.poses = deque(maxlen=16), deque(maxlen=16)
        self.last_stamp, self.last_receipt = -math.inf, None
        self.center = np.zeros(3)
        self.active = False
        self.tf = TransformBroadcaster(self)
        self.points_pub = self.create_publisher(PointCloud2, '/slam_test/follow_points', sensor_qos(2))
        self.markers_pub = self.create_publisher(MarkerArray, '/slam_test/follow_markers', latched())
        self.state_pub = self.create_publisher(String, '/slam_test/follow_status', latched())
        self.create_subscription(PointCloud2, '/slam_test/points', self.cloud, sensor_qos(2))
        self.create_subscription(String, '/slam_test/estimator_status', self.status, 10)
        self.create_timer(.1, self.check_freshness, clock=self.wall_clock)
        self.set_state(False, 'waiting_for_calibrated_SLAM_and_matching_cloud')

    def cloud(self, msg):
        if msg.header.frame_id != self.c['expected_frame']:
            self.invalidate('unexpected_cloud_frame')
            return
        self.clouds.append((stamp_s(msg.header.stamp), time.monotonic(), msg))
        self.match()

    def status(self, msg):
        try:
            status = json.loads(msg.data)
            if not isinstance(status, dict):
                raise ValueError('status must be an object')
            if not status.get('valid') or status.get('local') is None:
                self.invalidate(status.get('reason', 'tracking_invalid'))
                return
            stamp = float(status['stamp'])
            local = unpack(status['local'])
            if not math.isfinite(stamp) or stamp <= 0:
                raise ValueError('invalid stamp')
        except (KeyError, TypeError, ValueError):
            self.invalidate('malformed_estimator_status')
            return
        self.poses.append((stamp, time.monotonic(), local))
        self.match()

    def match(self):
        if self.guard_reason:
            self.invalidate(self.guard_reason)
            return
        now = time.monotonic()
        for stamp, pose_receipt, local in list(self.poses):
            if stamp <= self.last_stamp:
                continue
            for source_stamp, receipt, cloud in list(self.clouds):
                # JSON timestamp rounding at Unix epochs may differ by <1 us.
                if abs(source_stamp-stamp) > 1e-6:
                    continue
                if max(now-receipt, now-pose_receipt) > self.c['estimator_timeout_s']:
                    continue
                follow, body = follow_transforms(local, self.sphere.center,
                                                 self.c['lidar_mount_pitch_down_deg'])
                self.center = follow[:3, 3].copy()
                for child, matrix in [('slam_test_follow', follow), ('slam_test_drone_body', body)]:
                    tf = TransformStamped()
                    tf.header = Header(stamp=cloud.header.stamp, frame_id='slam_odom')
                    tf.child_frame_id = child
                    vector(tf.transform.translation, matrix[:3, 3])
                    q = Rotation.from_matrix(matrix[:3, :3]).as_quat()
                    tf.transform.rotation.x, tf.transform.rotation.y, tf.transform.rotation.z, tf.transform.rotation.w = map(float, q)
                    self.tf.sendTransform(tf)
                points, _ = xyz(cloud)
                self.points_pub.publish(point_cloud2.create_cloud_xyz32(
                    Header(stamp=cloud.header.stamp, frame_id='slam_odom'), transform(local, points).tolist()))
                self.last_stamp, self.last_receipt = stamp, now
                self.set_state(True, 'tracking_local_SLAM_gravity_initialized',
                               source_stamp=source_stamp,
                               cloud_to_view_ms=(time.monotonic()-receipt)*1000.,
                               processing_ms=(time.monotonic()-now)*1000.)
                break

    def invalidate(self, reason):
        self.clouds.clear(); self.poses.clear()
        self.set_state(False, reason)

    def check_freshness(self):
        if self.guard_reason:
            self.invalidate(self.guard_reason)
        elif self.active and self.last_receipt is not None and time.monotonic()-self.last_receipt > self.c['estimator_timeout_s']:
            self.invalidate('matched_cloud_or_pose_stale')

    def set_state(self, valid, reason, **extra):
        if not valid and self.active:
            # Clear old geometry instead of presenting a frozen scan as live.
            self.points_pub.publish(point_cloud2.create_cloud_xyz32(Header(frame_id='slam_odom'), []))
        changed = getattr(self, 'state_reason', None) != (valid, reason)
        self.active, self.state_reason = valid, (valid, reason)
        if changed:
            self.get_logger().info('Follow view: '+('TRACKING' if valid else 'NOT READY')+' | '+reason)
        state = dict(valid=valid, reason=reason, world_frame='slam_odom',
            camera_target='slam_test_follow', rotation_source='existing_ICP_IMU_local_pose',
            gravity_reference='stationary_IMU_initialization_not_continuous_gravity_fusion',
            sensor_to_screen_latency_measured=False,
            hardware_validated=False, **extra)
        self.state_pub.publish(json_message(state))
        atomic_json(self.directory/'follow_view_status.json', state)
        markers = []
        text = Marker()
        text.header.frame_id = 'slam_odom'
        text.ns, text.id, text.type = 'follow_status', 0, Marker.TEXT_VIEW_FACING
        text.pose.orientation.w = 1.
        vector(text.pose.position, self.center+[0., 0., .65])
        text.scale.z = .12
        text.color.r, text.color.g, text.color.a = (0., 1., 1.) if valid else (1., .15, 1.)
        text.text = ('TRACKING' if valid else 'NOT READY')+' | '+reason
        markers.append(text)
        sphere = Marker()
        sphere.header.frame_id = 'slam_odom'
        sphere.ns, sphere.id, sphere.type = 'follow_sphere', 0, Marker.SPHERE
        sphere.action = Marker.ADD if valid and self.sphere.enabled else Marker.DELETE
        sphere.pose.orientation.w = 1.
        vector(sphere.pose.position, self.center)
        sphere.scale.x = sphere.scale.y = sphere.scale.z = max(2*self.sphere.radius_m, .001)
        sphere.color.r, sphere.color.g, sphere.color.a = 1., .5, .15
        markers.append(sphere)
        self.markers_pub.publish(MarkerArray(markers=markers))


def main(args=None):
    run(FollowView, args)
