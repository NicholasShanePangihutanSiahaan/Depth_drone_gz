"""Passive ROS adapters for experimental manual SLAM testing."""
from collections import deque
from concurrent.futures import ThreadPoolExecutor
import csv
import datetime
import json
import math
import os
from pathlib import Path
import time

import numpy as np
from scipy.spatial.transform import Rotation
import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy
from rclpy.time import Time
from nav_msgs.msg import Odometry, Path as PathMsg
from sensor_msgs.msg import PointCloud2, Imu
from sensor_msgs_py import point_cloud2
from geometry_msgs.msg import PoseStamped, TransformStamped, Point
from std_msgs.msg import String, Header
from visualization_msgs.msg import Marker, MarkerArray
from tf2_ros import TransformBroadcaster

from mid360_test.nodes import BenchNode, RawRecorder, sensor_qos, xyz, stamp_s, json_message, run
from mid360_test.core.integration import ImuIntegrator
from mid360_test.core.recording import atomic_json
from mid360_test.core.health import stats
from .estimator import SlamEstimator, DEFAULTS
from .geometry import downsample, pose, inverse, transform, pack, unpack
from .evaluation import Evaluation

EXTRAS = dict(
    imu_lidar_rotation_rpy_deg=[0., 0., 0.], mounting_confirmed=False,
    exclusion_boxes_json='[]', preprocess_voxel_m=.06, preprocess_max_points=12000,
    max_imu_cloud_offset_s=.06, estimator_timeout_s=1., imu_receipt_timeout_s=.5,
    settle_window_s=1., settle_position_span_m=.03, settle_angle_span_deg=2.,
    minimum_settle_samples=5, loop_wait_s=20.,
    max_position_error_m=-1., max_angle_error_deg=-1., max_stationary_drift_m=-1.,
    max_stationary_orientation_deg=-1., reference_uncertainty_m=-1., reference_uncertainty_deg=-1.,
    map_publish_period_s=2., visual_max_points=60000,
)


def latched():
    return QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)


def vector(message, values):
    message.x, message.y, message.z = map(float, values)


def fill_pose(message, t):
    vector(message.position, t[:3, 3])
    q = Rotation.from_matrix(t[:3, :3]).as_quat()
    message.orientation.x, message.orientation.y, message.orientation.z, message.orientation.w = map(float, q)


class SlamNode(BenchNode):
    def __init__(self, name):
        super().__init__(name)
        for key, value in dict(EXTRAS, **DEFAULTS).items():
            self.c[key] = self.declare_parameter(key, value).value
        for key in ('estimator_timeout_s', 'imu_receipt_timeout_s', 'max_imu_cloud_offset_s',
                    'settle_window_s', 'settle_position_span_m', 'settle_angle_span_deg',
                    'map_publish_period_s', 'preprocess_voxel_m', 'loop_wait_s'):
            if not math.isfinite(self.c[key]) or self.c[key] <= 0:
                raise ValueError('invalid '+key)

    def guard_update(self):
        super().guard_update()
        # No two estimator/evaluator sessions may share one ROS domain.
        names = self.get_node_names()
        duplicates = [name for name in ('lidar_imu_estimator','test_evaluator','cloud_preprocessor','test_monitor')
                      if names.count(name)>1]
        if duplicates:
            self.guard_reason = (self.guard_reason+';duplicate_test_nodes:'+','.join(duplicates)).strip(';')


class CloudPreprocessor(SlamNode):
    def __init__(self):
        super().__init__('cloud_preprocessor')
        self.boxes = json.loads(self.c['exclusion_boxes_json'])
        for box in self.boxes:
            if set(box) != {'min', 'max'}:
                raise ValueError('each exclusion box needs min/max in sensor metres')
            lo, hi = np.asarray(box['min']), np.asarray(box['max'])
            if lo.shape != (3,) or hi.shape != (3,) or not np.all(np.isfinite(np.r_[lo,hi])) or np.any(lo >= hi):
                raise ValueError('invalid exclusion box')
        self.pub = self.create_publisher(PointCloud2, '/slam_test/points', sensor_qos(2))
        self.stats = self.create_publisher(String, '/slam_test/preprocessing', 10)
        self.create_subscription(PointCloud2, '/lidar_test/points', self.cloud, sensor_qos(2))

    def cloud(self, msg):
        started = time.monotonic()
        if self.guard_reason or msg.header.frame_id != self.c['expected_frame']:
            return
        points, tags = xyz(msg)
        radius = np.linalg.norm(points, axis=1)
        keep = np.all(np.isfinite(points), axis=1) & (radius >= self.c['minimum_range_m']) & (radius <= self.c['maximum_range_m'])
        if self.c['reject_tag_mask']:
            if tags is None:
                self.get_logger().error('configured tag filter but tag missing')
                return
            keep &= (tags & self.c['reject_tag_mask']) == 0
        for box in self.boxes:
            keep &= ~np.all((points >= box['min']) & (points <= box['max']), axis=1)
        selected = downsample(points[keep], self.c['preprocess_voxel_m'], self.c['preprocess_max_points'])
        self.pub.publish(point_cloud2.create_cloud_xyz32(msg.header, selected.tolist()))
        self.stats.publish(json_message(dict(stamp=stamp_s(msg.header.stamp), raw_points=len(points),
            selected_points=len(selected), removed_points=int((~keep).sum()),
            processing_ms=(time.monotonic()-started)*1000., deskewed=False,
            filtered_frame=msg.header.frame_id, source_fields_preserved_in_raw_bag=True)))


class EstimatorNode(SlamNode):
    def __init__(self):
        super().__init__('lidar_imu_estimator')
        self.backend = SlamEstimator({k: self.c[k] for k in DEFAULTS})
        self.integrator = ImuIntegrator(self.c['acceleration_unit'], self.c['calibration_seconds'],
            self.c['calibration_min_samples'], self.c['imu_max_dt_s'],
            self.c['calibration_gyro_limit_rad_s'], self.c['calibration_accel_std_m_s2'])
        # Constant measured rotation from IMU coordinates to LiDAR coordinates.
        self.imu_to_lidar = Rotation.from_euler('xyz', self.c['imu_lidar_rotation_rpy_deg'], degrees=True).as_matrix()
        self.imu_history = deque(maxlen=4000)
        self.imu_received = None
        self.pending = None
        self.future = None
        self.worker = ThreadPoolExecutor(max_workers=1, thread_name_prefix='bench_slam')
        self.local_pub = self.create_publisher(Odometry, '/slam_test/odometry_local', 10)
        self.pose_pub = self.create_publisher(PoseStamped, '/slam_test/pose_map', 10)
        self.status_pub = self.create_publisher(String, '/slam_test/estimator_status', 10)
        self.loop_pub = self.create_publisher(String, '/slam_test/loop_events', 50)
        self.local_path = self.create_publisher(PathMsg, '/slam_test/path_local', latched())
        self.map_path = self.create_publisher(PathMsg, '/slam_test/path_map', latched())
        self.map_cloud = self.create_publisher(PointCloud2, '/slam_test/map_points', sensor_qos(1))
        self.live_cloud = self.create_publisher(PointCloud2, '/slam_test/registered_points', sensor_qos(2))
        self.tf = TransformBroadcaster(self)
        self.local_samples = deque(maxlen=20000)
        self.last_visual = -math.inf
        self.last_status, self.completed_receipt = None, None
        self.fatal_reason = ''
        self.last_health_emit = 0.
        self.received_clouds = self.replaced_clouds = 0
        self.create_subscription(Imu, self.c['imu_topic'], self.imu, sensor_qos(400))
        self.create_subscription(PointCloud2, '/slam_test/points', self.cloud, sensor_qos(2))
        self.create_timer(.02, self.poll, clock=self.wall_clock)

    def imu(self, msg):
        if not self.accepts(self.c['imu_topic']):
            return
        stamp = stamp_s(msg.header.stamp)
        self.imu_received = time.monotonic()
        if msg.header.frame_id != self.c['expected_imu_frame']:
            self.fatal_reason = 'unexpected_imu_frame'
            return
        gyro = self.imu_to_lidar@np.array([msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z])
        acc = self.imu_to_lidar@np.array([msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z])
        result = self.integrator.update(stamp, gyro, acc)
        if result['valid']:
            self.imu_history.append((stamp, Rotation.from_quat(result['quaternion_xyzw']).as_matrix()))
        elif result['state'] == 'INVALID':
            self.fatal_reason = result['reason']

    def cloud(self, msg):
        self.received_clouds += 1
        if self.pending is not None:
            self.replaced_clouds += 1
        self.pending = (msg, time.monotonic())

    def job(self, msg, imu_rotation, receipt):
        started = time.monotonic()
        points, _ = xyz(msg)
        status = self.backend.scan(points, stamp_s(msg.header.stamp), imu_rotation)
        status.update(queue_ms=(started-receipt)*1000.,
                      source_to_result_receipt_ms=(time.monotonic()-receipt)*1000.,
                      sensor_stamp_latency_measured=False)
        visual = None
        if self.backend.keyframes and time.monotonic()-self.last_visual >= self.c['map_publish_period_s']:
            self.last_visual = time.monotonic()
            mapped = [transform(t, f['cloud']) for t, f in zip(self.backend.map_poses, self.backend.keyframes)]
            cloud = downsample(np.vstack(mapped), self.c['voxel_m'], self.c['visual_max_points'])
            visual = (cloud, [(f['stamp'], f['local'].copy()) for f in self.backend.keyframes],
                      [(f['stamp'], t.copy()) for f, t in zip(self.backend.keyframes, self.backend.map_poses)])
        live = transform(unpack(status['mapped']), downsample(points,self.c['voxel_m'],self.c['max_points'])) if status['valid'] else None
        return status, visual, live

    def invalid_status(self, reason):
        result = dict(self.last_status or {}, valid=False, tracking='LOST' if self.fatal_reason else 'INITIALIZING',
                      reason=reason, mode=self.c['mode'])
        result.update(local=None, mapped=None, velocity_mps=None)
        self.status_pub.publish(json_message(result))

    def poll(self):
        now = time.monotonic()
        reason = self.guard_reason
        if self.c['mode'] == 'hardware' and not self.c['mounting_confirmed']:
            reason = reason or 'confirm_measured_mount_and_IMU_units_in_configuration'
        if self.imu_received is not None and now-self.imu_received > self.c['imu_receipt_timeout_s']:
            if self.last_status and self.last_status.get('valid'):
                self.fatal_reason = 'imu_receipt_timeout_restart_required'
            reason = reason or 'waiting_for_imu'
        reason = self.fatal_reason or reason
        if self.future is not None and self.future.done():
            try:
                status, visual, live = self.future.result()
                if not reason:
                    self.publish(status, visual, live)
            except Exception as exc:
                self.fatal_reason = 'worker_exception:'+str(exc)
                reason = self.fatal_reason
            self.future = None
        if self.completed_receipt is not None and now-self.completed_receipt > self.c['estimator_timeout_s']:
            reason = reason or 'estimator_output_stale'
        if reason:
            if now-self.last_health_emit > .2:
                self.invalid_status(reason)
                self.last_health_emit = now
            return
        if self.future is not None:
            return
        if self.pending is None or not self.imu_history:
            if now-self.last_health_emit > .5 and not self.last_status:
                self.invalid_status(self.integrator.reason)
                self.last_health_emit = now
            return
        msg, receipt = self.pending
        stamp = stamp_s(msg.header.stamp)
        if msg.header.frame_id != self.c['expected_frame']:
            self.fatal_reason = 'unexpected_cloud_frame'
            return
        if now-receipt > self.c['cloud_timeout_s']:
            self.pending = None
            return
        # Only IMU states at/before acquisition time are eligible; no future pose relabel.
        before = [(s, r) for s, r in self.imu_history if s <= stamp]
        if not before or stamp-before[-1][0] > self.c['max_imu_cloud_offset_s']:
            if now-self.last_health_emit > .2:
                self.invalid_status('waiting_for_matching_IMU_clock_or_calibration')
                self.last_health_emit = now
            return
        if not self.c['imu_cloud_same_clock']:
            self.invalid_status('source_clock_relationship_not_confirmed')
            return
        if self.c['timestamps_synchronized']:
            age = self.get_clock().now().nanoseconds*1e-9-stamp
            if not 0 <= age <= self.c['cloud_timeout_s']:
                self.fatal_reason = 'source_cloud_age_invalid'
                return
        self.pending = None
        self.future = self.worker.submit(self.job, msg, before[-1][1].copy(), receipt)

    def publish(self, status, visual, live=None):
        self.completed_receipt = time.monotonic()
        status.update(mode=self.c['mode'], replaced_clouds=self.replaced_clouds,
                      received_clouds=self.received_clouds,
                      mounting_confirmed=self.c['mounting_confirmed'])
        self.last_status = status
        self.status_pub.publish(json_message(status))
        for event in status['loop_events']:
            self.loop_pub.publish(json_message(event))
        if not status['valid']:
            self.fatal_reason = status['reason']
            return
        stamp = Time(nanoseconds=int(status['stamp']*1e9)).to_msg()
        local, mapped = unpack(status['local']), unpack(status['mapped'])
        odom = Odometry()
        odom.header = Header(stamp=stamp, frame_id='slam_odom')
        odom.child_frame_id = 'slam_lidar'
        fill_pose(odom.pose.pose, local)
        vector(odom.twist.twist.linear, local[:3, :3].T@np.array(status['velocity_mps']))
        # Negative diagonal is an explicit project convention for UNKNOWN covariance.
        # Consumer must use status.covariance_available=false, not treat this as calibrated.
        for index in range(0, 36, 7):
            odom.pose.covariance[index] = odom.twist.covariance[index] = -1.
        self.local_pub.publish(odom)
        mapped_msg = PoseStamped(header=Header(stamp=stamp, frame_id='slam_map'))
        fill_pose(mapped_msg.pose, mapped)
        self.pose_pub.publish(mapped_msg)
        if live is not None:
            self.live_cloud.publish(point_cloud2.create_cloud_xyz32(mapped_msg.header,live.tolist()))
        transforms = []
        for parent, child, t in [('slam_map', 'slam_odom', mapped@inverse(local)),
                                 ('slam_odom', 'slam_lidar', local)]:
            tf = TransformStamped()
            tf.header = Header(stamp=stamp, frame_id=parent)
            tf.child_frame_id = child
            vector(tf.transform.translation, t[:3, 3])
            q = Rotation.from_matrix(t[:3, :3]).as_quat()
            tf.transform.rotation.x, tf.transform.rotation.y, tf.transform.rotation.z, tf.transform.rotation.w = map(float,q)
            transforms.append(tf)
        self.tf.sendTransform(transforms)
        self.local_samples.append((status['stamp'], local))
        if visual is not None:
            cloud, _, mapped_rows = visual
            self.map_cloud.publish(point_cloud2.create_cloud_xyz32(mapped_msg.header, cloud.tolist()))
            self.local_path.publish(self.path(self.local_samples, 'slam_odom', stamp))
            self.map_path.publish(self.path(mapped_rows, 'slam_map', stamp))

    def path(self, rows, frame, stamp):
        out = PathMsg(header=Header(stamp=stamp, frame_id=frame))
        for source_stamp, t in rows:
            msg = PoseStamped(header=Header(stamp=Time(nanoseconds=int(source_stamp*1e9)).to_msg(), frame_id=frame))
            fill_pose(msg.pose, t)
            out.poses.append(msg)
        return out

    def destroy_node(self):
        self.worker.shutdown(wait=True, cancel_futures=True)
        if self.backend.keyframes:
            local = np.array([f['local'] for f in self.backend.keyframes])
            mapped = np.array(self.backend.map_poses)
            np.savez(self.directory/'keyframe_poses.npz', local=local, mapped=mapped,
                     stamps=np.array([f['stamp'] for f in self.backend.keyframes]))
            np.savez_compressed(self.directory/'keyframe_clouds.npz',
                                **{f'cloud_{i}': f['cloud'] for i,f in enumerate(self.backend.keyframes)})
            points = downsample(np.vstack([transform(t,f['cloud']) for t,f in zip(mapped,self.backend.keyframes)]),
                                self.c['voxel_m'], self.c['visual_max_points'])
            np.save(self.directory/'map_points.npy', points)
        atomic_json(self.directory/'estimator_final.json', self.last_status or dict(valid=False, reason='NO_DATA'))
        return super().destroy_node()


class EvaluatorNode(SlamNode):
    def __init__(self):
        super().__init__('test_evaluator')
        names = ('settle_window_s', 'settle_position_span_m', 'settle_angle_span_deg',
            'minimum_settle_samples', 'loop_wait_s', 'max_position_error_m', 'max_angle_error_deg',
            'max_stationary_drift_m', 'max_stationary_orientation_deg',
            'reference_uncertainty_m', 'reference_uncertainty_deg')
        settings = {key: self.c[key] for key in names}
        settings['status_timeout_s'] = self.c['estimator_timeout_s']
        self.engine = Evaluation(settings)
        self.events = (self.directory/'test_events.jsonl').open('x', buffering=1)
        self.state_pub = self.create_publisher(String, '/slam_test/test_state', latched())
        self.ack_pub = self.create_publisher(String, '/slam_test/command_ack', 20)
        self.marker_pub = self.create_publisher(MarkerArray, '/slam_test/reference_markers', latched())
        self.create_subscription(String, '/slam_test/estimator_status', self.status, 50)
        self.create_subscription(String, '/slam_test/loop_events', self.loop, 50)
        self.create_subscription(String, '/slam_test/commands', self.command, 20)
        self.create_timer(.2, self.tick, clock=self.wall_clock)
        self.last_result_count = 0

    def status(self, msg):
        self.engine.receive(json.loads(msg.data), time.monotonic())

    def loop(self, msg):
        self.engine.receive_loop(json.loads(msg.data), time.monotonic())
        self.events.write(json.dumps(dict(type='loop_event', payload=json.loads(msg.data)))+'\n')

    def command(self, msg):
        try:
            command = json.loads(msg.data)
            if not isinstance(command, dict):
                raise ValueError('command must be object')
            result = self.engine.command(command, time.monotonic())
            self.events.write(json.dumps(dict(type='command', command=command, response=result))+'\n')
            self.ack_pub.publish(json_message(result))
            self.tick()
        except (ValueError, TypeError) as exc:
            self.get_logger().error(str(exc))

    def tick(self):
        if self.guard_reason and self.engine.active and self.engine.phase != 'REVIEW':
            self.engine.active['invalid_reasons'].add(self.guard_reason)
            self.engine.complete(time.monotonic())
        self.engine.tick(time.monotonic())
        view = self.engine.view(time.monotonic())
        self.state_pub.publish(json_message(view))
        if len(self.engine.results) != self.last_result_count:
            self.events.write(json.dumps(dict(type='result', result=self.engine.results[-1]))+'\n')
            self.last_result_count = len(self.engine.results)
        self.save(view)
        if self.engine.active:
            a = self.engine.active
            start = a['start_local'][:3, 3]
            reference = np.asarray(a['spec'].get('reference_xyz_m', [0.,0.,0.]))
            target = start+a['start_local'][:3,:3]@reference
            markers = []
            for i, p in enumerate((start, target)):
                m = Marker()
                m.header.frame_id = 'slam_odom'
                m.ns, m.id, m.type, m.action = 'operator_reference', i, Marker.SPHERE, Marker.ADD
                m.pose.orientation.w = 1.
                vector(m.pose.position, p)
                m.scale.x = m.scale.y = m.scale.z = .15
                m.color.r, m.color.g, m.color.b, m.color.a = (1., .8 if i else 0., 0., .8)
                markers.append(m)
            self.marker_pub.publish(MarkerArray(markers=markers))

    def save(self, view):
        results = self.engine.results
        latest = {r['stage']: r for r in results}
        status = ('FAIL' if any(r['status'] == 'FAIL' for r in latest.values()) else
            'PASS' if len(latest) == len(self.engine.specs) and all(r['status'] == 'PASS' for r in latest.values())
            else 'NOT TESTED')
        atomic_json(self.directory/'test_results.json', dict(status=status, mode=self.c['mode'],
            state=view, results=results, acceptance_configuration={k:self.c[k] for k in EXTRAS},
            untested_stages=[s['name'] for s in self.engine.specs if s['name'] not in latest],
            accuracy_scope='operator_endpoint_references_no_continuous_ground_truth',
            backend_scope='experimental_ICP_IMU_rotation_seed_pose_graph',
            hardware_validation='NOT TESTED_until_operator_runs_and_reviews',
            repeated_attempt_policy='all_attempts_preserved_overall_uses_latest_attempt_per_stage'))

    def destroy_node(self):
        if self.engine.active and self.engine.phase not in ('REVIEW', 'COMPLETE'):
            self.engine.active['invalid_reasons'].add('session_shutdown_during_stage')
            self.engine.complete(time.monotonic())
        self.save(self.engine.view(time.monotonic()))
        self.events.close()
        return super().destroy_node()


class MonitorNode(SlamNode):
    def __init__(self):
        super().__init__('test_monitor')
        self.status_data, self.stage, self.pre = {}, {}, {}
        self.receipt = None
        self.processing_samples = deque(maxlen=4096)
        self.queue_samples = deque(maxlen=4096)
        self.pipeline_samples = deque(maxlen=4096)
        self.status_count = 0
        self.last_source_stamp = None
        self.tracking_fault_count = 0
        self.last_cpu_ticks = None
        self.started = time.monotonic()
        self.text = (self.directory/'slam_readings.log').open('x', buffering=1)
        self.csvfile = (self.directory/'slam_readings.csv').open('x', buffering=1, newline='')
        self.csv = csv.writer(self.csvfile)
        self.csv.writerow(['elapsed_wall_s','source_stamp','stage','phase','tracking',
            'local_x_m','local_y_m','local_z_m','roll_deg','pitch_deg','yaw_deg',
            'map_x_m','map_y_m','map_z_m','vx_mps','vy_mps','vz_mps',
            'stage_displacement_m','stage_accumulated_yaw_deg','processing_ms',
            'queue_ms','preprocess_ms','loops_accepted','reason'])
        self.loops_file = (self.directory/'loop_events.jsonl').open('x', buffering=1)
        self.create_subscription(String, '/slam_test/estimator_status', self.status, 50)
        self.create_subscription(String, '/slam_test/test_state', lambda msg:setattr(self,'stage',json.loads(msg.data)), latched())
        self.create_subscription(String, '/slam_test/preprocessing', lambda msg:setattr(self,'pre',json.loads(msg.data)), 10)
        self.create_subscription(String, '/slam_test/loop_events', self.loop, 50)
        self.resources = (self.directory/'resources.jsonl').open('x', buffering=1)
        self.create_timer(.2, self.tick, clock=self.wall_clock)
        self.create_timer(1., self.resource_tick, clock=self.wall_clock)
        atomic_json(self.directory/'slam_session.json', dict(configuration=self.c,
            created_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),
            sensor_pose_not_drone_body_pose=True, mode=self.c['mode']))

    def status(self, msg):
        self.status_data, self.receipt = json.loads(msg.data), time.monotonic()
        s = self.status_data
        self.status_count += 1
        self.tracking_fault_count += int(not s.get('valid',False))
        if s.get('valid') and s.get('stamp') != self.last_source_stamp:
            for key, samples in [('processing_ms',self.processing_samples),('queue_ms',self.queue_samples),
                                 ('source_to_result_receipt_ms',self.pipeline_samples)]:
                if s.get(key) is not None:
                    samples.append(s[key])
            self.last_source_stamp = s.get('stamp')

    def loop(self, msg):
        self.loops_file.write(msg.data+'\n')
        event = json.loads(msg.data)
        print(f"Loop = {event['state']} | Keyframes = {event['from_keyframe']}->{event['to_keyframe']} | Reason = {event['reason']}", flush=True)

    def tick(self):
        s, stage = self.status_data, self.stage
        fresh = self.receipt is not None and time.monotonic()-self.receipt <= self.c['estimator_timeout_s']
        valid = s.get('valid') and fresh and not self.guard_reason
        tracking = s.get('tracking', 'NO_DATA') if fresh else 'STALE_OR_NO_DATA'
        elapsed = time.monotonic()-self.started
        p = s['local']['position'] if valid else [None]*3
        mapped = s['mapped']['position'] if valid else [None]*3
        rpy = Rotation.from_quat(s['local']['quaternion']).as_euler('xyz', degrees=True).tolist() if valid else [None]*3
        velocity = s['velocity_mps'] if valid else [None]*3
        pose_text = ', '.join(f'{v:.3f}' for v in p) if valid else 'N/A'
        angles = ', '.join(f'{v:.1f}' for v in rpy) if valid else 'N/A'
        speed = f"{np.linalg.norm(velocity):.3f} m/s" if valid else 'N/A'
        displacement = stage.get('live_displacement_m')
        displacement_text = f'{displacement:.3f} m' if displacement is not None and valid else 'N/A'
        turn = stage.get('accumulated_yaw_deg')
        turn_text = f'{turn:.1f} deg' if turn is not None and valid else 'N/A'
        line = (f"Time = {elapsed:.1f} s | Test = {stage.get('stage','WAIT_CONTROLLER')} | Phase = {stage.get('phase','READY')}"
            f" | Tracking = {tracking} | Position = [{pose_text}] m | RPY = [{angles}] deg"
            f" | Displacement = {displacement_text} | Turn = {turn_text} | Speed_est = {speed}"
            f" | Loop count = {s.get('loops_accepted',0)} | Reason = {self.guard_reason or s.get('reason','waiting')}")
        print(line, flush=True)
        self.text.write(line+'\n')
        self.csv.writerow([elapsed,s.get('stamp'),stage.get('stage'),stage.get('phase'),tracking,
            *p,*rpy,*mapped,*velocity,stage.get('live_displacement_m'),stage.get('accumulated_yaw_deg'),
            s.get('processing_ms'),s.get('queue_ms'),self.pre.get('processing_ms'),s.get('loops_accepted'),s.get('reason')])

    def resource_tick(self):
        temps = {}
        for path in Path('/sys/class/thermal').glob('thermal_zone*/temp'):
            try:
                temps[path.parent.name] = float(path.read_text())/1000.
            except (OSError, ValueError):
                pass
        memory = {}
        try:
            for line in Path('/proc/meminfo').read_text().splitlines():
                key, value = line.split(':',1)
                if key in ('MemTotal','MemAvailable'):
                    memory[key+'_kB'] = int(value.split()[0])
            cpu_ticks = [int(v) for v in Path('/proc/stat').read_text().splitlines()[0].split()[1:]]
        except (OSError, ValueError):
            cpu_ticks = []
        cpu_percent = None
        if self.last_cpu_ticks is not None and cpu_ticks:
            # Guest ticks are already included in user/nice; sum first eight.
            previous,total = self.last_cpu_ticks,cpu_ticks
            delta=sum(total[:8])-sum(previous[:8])
            idle=(total[3]+total[4])-(previous[3]+previous[4])
            if delta>0:cpu_percent=100.*(delta-idle)/delta
        self.last_cpu_ticks = cpu_ticks if cpu_ticks else self.last_cpu_ticks
        self.resources.write(json.dumps(dict(elapsed_wall_s=time.monotonic()-self.started,
            host_load_average=list(os.getloadavg()), host_memory=memory, thermal_c=temps,
            host_cpu_ticks=cpu_ticks,host_cpu_utilization_percent=cpu_percent,cpu_scope='host_aggregate_ticks_not_per_node',
            processing_scope='preprocessing_and_registration_only_no_flight_pipeline'))+'\n')
        self.save_runtime()

    def save_runtime(self):
        atomic_json(self.directory/'runtime_summary.json',dict(mode=self.c['mode'],
            received_status_messages=self.status_count,invalid_status_messages=self.tracking_fault_count,
            timing_sample_count=len(self.processing_samples),statistics_window=4096,
            registration_graph_ms=stats(self.processing_samples),queue_ms=stats(self.queue_samples),
            processed_cloud_receive_to_backend_result_ms=stats(self.pipeline_samples),
            excludes='sensor_acquisition;driver;preprocessing_before_estimator_receive;ROS_publish_delivery;GUI',
            drift_accuracy='NOT TESTED_without_independent_measurement',
            flight_pipeline='NOT TESTED_no_planner_control_FCU',hardware_performance='NOT TESTED_until_run_on_Jetson'))

    def destroy_node(self):
        self.save_runtime()
        for file in (self.text,self.csvfile,self.loops_file,self.resources):
            file.close()
        return super().destroy_node()


class SlamRecorder(RawRecorder):
    def __init__(self):
        super().__init__()
        import rosbag2_py
        topics = [('/slam_test/points','sensor_msgs/msg/PointCloud2'),
            ('/slam_test/registered_points','sensor_msgs/msg/PointCloud2'),
            ('/slam_test/odometry_local','nav_msgs/msg/Odometry'),
            ('/slam_test/pose_map','geometry_msgs/msg/PoseStamped'),
            ('/slam_test/path_local','nav_msgs/msg/Path'),('/slam_test/path_map','nav_msgs/msg/Path'),
            ('/slam_test/map_points','sensor_msgs/msg/PointCloud2'),
            ('/slam_test/estimator_status','std_msgs/msg/String'),('/slam_test/loop_events','std_msgs/msg/String'),
            ('/slam_test/commands','std_msgs/msg/String'),('/slam_test/command_ack','std_msgs/msg/String'),
            ('/slam_test/test_state','std_msgs/msg/String'),('/slam_test/preprocessing','std_msgs/msg/String'),
            ('/tf','tf2_msgs/msg/TFMessage')]
        from rosidl_runtime_py.utilities import get_message
        for topic, kind in topics:
            self.writer.create_topic(rosbag2_py.TopicMetadata(name=topic, type=kind, serialization_format='cdr'))
            with self.lock:
                self.received_by_topic[topic] = self.written_by_topic[topic] = 0
            qos = latched() if topic in ('/slam_test/test_state','/slam_test/path_local','/slam_test/path_map') else sensor_qos(100)
            self.create_subscription(get_message(kind), topic, lambda data,t=topic:self.receive(t,data), qos, raw=True)

    def accepts(self, topic):
        if topic in (self.c['cloud_topic'], self.c['imu_topic']):
            return super().accepts(topic)
        return not self.guard_reason

    def report(self):
        super().report()
        # Some topics (e.g. loop event) may correctly have zero messages.
        path = self.directory/'recorder_status.json'
        data = json.loads(path.read_text())
        if data['status'] != 'FAIL':
            data['status'] = ('PASS' if all(data['written_by_topic'].get(t,0) > 0
                for t in (self.c['cloud_topic'],self.c['imu_topic'])) else 'NOT TESTED')
        data['scope'] = 'raw_sources_and_test_outputs_zero_loop_events_is_not_recording_failure'
        atomic_json(path,data)


def cloud_preprocessor_main(args=None): run(CloudPreprocessor,args)
def lidar_imu_estimator_main(args=None): run(EstimatorNode,args)
def test_evaluator_main(args=None): run(EvaluatorNode,args)
def test_monitor_main(args=None): run(MonitorNode,args)
def slam_recorder_main(args=None): run(SlamRecorder,args)
