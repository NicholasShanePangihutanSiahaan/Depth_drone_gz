"""ROS 2 bench adapters. Only /lidar_test/* outputs; no vehicle commands or TF."""
from collections import deque
import datetime
import gc
import json
import math
import queue
import threading
import time
from pathlib import Path

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rosidl_runtime_py.utilities import get_message
from sensor_msgs.msg import PointCloud2, PointField, Imu
from sensor_msgs_py import point_cloud2
from geometry_msgs.msg import QuaternionStamped, Vector3Stamped
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from std_msgs.msg import String

from .core.sectors import measure
from .core.integration import ImuIntegrator
from .core.health import StreamHealth
from .core.recording import ReadingFiles, atomic_json

DEFAULTS = dict(
    mode='hardware', output_dir='/tmp/mid360_test_single_node',
    cloud_topic='/livox/lidar', imu_topic='/livox/imu',
    cloud_type='sensor_msgs/msg/PointCloud2', expected_frame='livox_frame',
    expected_imu_frame='livox_frame', allowed_publishers=['livox_lidar_publisher'],
    sector_half_width_deg=5., elevation_min_deg=-5., elevation_max_deg=5.,
    minimum_range_m=.2, maximum_range_m=70., reject_tag_mask=0,
    acceleration_unit='g', calibration_seconds=2., calibration_min_samples=100,
    calibration_gyro_limit_rad_s=.03, calibration_accel_std_m_s2=.15,
    imu_max_dt_s=.1, cloud_timeout_s=.5, imu_timeout_s=.1,
    timestamps_synchronized=False, imu_cloud_same_clock=True, sync_tolerance_s=.05,
    expected_cloud_hz=10., expected_imu_hz=0., logger_hz=5.,
    recorder_max_queue_bytes=134217728, recorder_queue_messages=512,
    operator_notes='', device_serial='UNKNOWN', mounting_notes='UNCONFIRMED',
)


def stamp_s(stamp):
    return stamp.sec+stamp.nanosec*1e-9


def sensor_qos(depth=100):
    return QoSProfile(depth=depth, reliability=ReliabilityPolicy.BEST_EFFORT,
                      durability=DurabilityPolicy.VOLATILE)


def json_message(data):
    return String(data=json.dumps(data, allow_nan=False))


def xyz(message):
    rows = point_cloud2.read_points(message, field_names=('x', 'y', 'z'), skip_nans=False)
    if getattr(rows.dtype, 'names', None):
        points = np.column_stack([rows[name].reshape(-1) for name in ('x', 'y', 'z')])
    else:
        points = np.asarray(list(rows), dtype=float).reshape(-1, 3)
    tags = None
    if any(field.name == 'tag' for field in message.fields):
        data = point_cloud2.read_points(message, field_names=('tag',), skip_nans=False)
        tags = data['tag'].reshape(-1) if getattr(data.dtype, 'names', None) else np.asarray(list(data)).reshape(-1)
    return points, tags


class BenchNode(Node):
    def __init__(self, name):
        super().__init__(name)
        self.c = {k: self.declare_parameter(k, value).value for k, value in DEFAULTS.items()}
        mode = self.c['mode']
        if mode not in ('hardware', 'replay'):
            raise ValueError('mode must be hardware or replay')
        if mode == 'hardware' and self.get_parameter('use_sim_time').value:
            raise ValueError('hardware refuses use_sim_time=true')
        if self.c['cloud_type'] not in ('sensor_msgs/msg/PointCloud2', 'livox_ros_driver2/msg/CustomMsg'):
            raise ValueError('unsupported cloud_type')
        for key in ('cloud_topic', 'imu_topic'):
            topic = self.c[key]
            if not topic.startswith('/') or topic.startswith('/lidar_test/'):
                raise ValueError('input topics must be absolute and outside /lidar_test')
            if mode == 'hardware' and any(s in topic.lower() for s in ('simulation', 'ground_truth', 'mock')):
                raise ValueError('hardware refuses simulator topic names')
        if mode == 'hardware' and not self.c['allowed_publishers']:
            raise ValueError('hardware requires an explicit driver publisher allowlist')
        for key in ('logger_hz', 'cloud_timeout_s', 'imu_timeout_s', 'sync_tolerance_s'):
            if not math.isfinite(self.c[key]) or self.c[key] <= 0:
                raise ValueError(key+' must be positive and finite')
        if self.c['recorder_max_queue_bytes'] < 1024 or self.c['recorder_queue_messages'] < 1:
            raise ValueError('invalid recorder queue bounds')
        if not 0 <= self.c['reject_tag_mask'] <= 255:
            raise ValueError('reject_tag_mask must be 0..255')
        self.directory = Path(self.c['output_dir']).expanduser()
        self.directory.mkdir(parents=True, exist_ok=True)
        self.guard_reason = ''
        self.graph_snapshot = {}
        self.guard_update()
        self.wall_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.create_timer(.5, self.guard_update, clock=self.wall_clock)

    def guard_update(self):
        if self.c['mode'] == 'replay':
            self.guard_reason = ''
            return
        errors = []
        if self.get_parameter('use_sim_time').value:
            errors.append('simulation_time_forbidden')
        if '/clock' in dict(self.get_topic_names_and_types()):
            errors.append('clock_topic_present_use_isolated_hardware_domain')
        graph = {}
        for topic in (self.c['cloud_topic'], self.c['imu_topic']):
            endpoints = self.get_publishers_info_by_topic(topic)
            names = [e.node_name for e in endpoints]
            graph[topic] = [dict(node=e.node_name, namespace=e.node_namespace,
                                 type=e.topic_type) for e in endpoints]
            if len(endpoints) > 1:
                errors.append('multiple_source_publishers:'+topic)
            if any(name not in self.c['allowed_publishers'] for name in names):
                errors.append('unapproved_source_publisher:'+topic)
            expected = self.c['cloud_type'] if topic == self.c['cloud_topic'] else 'sensor_msgs/msg/Imu'
            if any(e.topic_type != expected for e in endpoints):
                errors.append('source_type_mismatch:'+topic)
        self.graph_snapshot = graph
        reason = ';'.join(errors)
        if reason and reason != self.guard_reason:
            self.get_logger().error(reason)
        self.guard_reason = reason

    def accepts(self, topic):
        if self.c['mode'] == 'replay':
            return True
        # Refresh discovery at first messages; timer continues checking /clock.
        endpoints = self.get_publishers_info_by_topic(topic)
        return (not self.guard_reason and len(endpoints) == 1
                and endpoints[0].node_name in self.c['allowed_publishers'])


class CloudAdapter(BenchNode):
    def __init__(self):
        super().__init__('cloud_adapter')
        self.pub = self.create_publisher(PointCloud2, '/lidar_test/points', sensor_qos(5))
        self.create_subscription(get_message(self.c['cloud_type']), self.c['cloud_topic'],
                                 self.receive, sensor_qos(10))

    def receive(self, msg):
        if not self.accepts(self.c['cloud_topic']):
            return
        if self.c['cloud_type'] == 'sensor_msgs/msg/PointCloud2':
            self.pub.publish(msg)  # Preserve header, every field, invalid returns and bytes.
            return
        # Analysis copy only. Recorder retains original CustomMsg incl. timebase/lidar_id.
        if msg.point_num != len(msg.points):
            self.get_logger().error('CustomMsg point_num mismatch; analysis copy rejected')
            return
        fields = [PointField(name=name, offset=offset, datatype=kind, count=1)
                  for name, offset, kind in (
                      ('x', 0, PointField.FLOAT32), ('y', 4, PointField.FLOAT32),
                      ('z', 8, PointField.FLOAT32), ('intensity', 12, PointField.FLOAT32),
                      ('tag', 16, PointField.UINT8), ('line', 17, PointField.UINT8),
                      ('offset_time', 18, PointField.UINT32))]
        points = [(p.x, p.y, p.z, float(p.reflectivity), p.tag, p.line, p.offset_time)
                  for p in msg.points]
        out = point_cloud2.create_cloud(msg.header, fields, points)
        out.is_dense = False
        self.pub.publish(out)


class DirectionalDistance(BenchNode):
    def __init__(self):
        super().__init__('directional_distance')
        # Validate geometric limits even if no sensor is connected.
        self.calculate(np.empty((0, 3)), np.empty(0, dtype=np.uint8))
        self.pub = self.create_publisher(String, '/lidar_test/distances', 20)
        self.health = StreamHealth(self.c['cloud_timeout_s'], self.c['expected_cloud_hz'])
        self.create_subscription(PointCloud2, '/lidar_test/points', self.receive, sensor_qos(5))

    def calculate(self, points, tags):
        return measure(points, self.c['sector_half_width_deg'], self.c['elevation_min_deg'],
                       self.c['elevation_max_deg'], self.c['minimum_range_m'],
                       self.c['maximum_range_m'], tags, self.c['reject_tag_mask'])

    def receive(self, msg):
        started = time.monotonic()
        stamp = stamp_s(msg.header.stamp)
        error = ''
        try:
            if self.guard_reason:
                raise ValueError(self.guard_reason)
            if msg.header.frame_id != self.c['expected_frame']:
                raise ValueError('unexpected_cloud_frame')
            if stamp <= 0 or (self.health.stamp is not None and stamp <= self.health.stamp):
                raise ValueError('invalid_or_unordered_cloud_stamp')
            points, tags = xyz(msg)
            result = self.calculate(points, tags)
        except (ValueError, KeyError, AssertionError, TypeError) as exc:
            error = str(exc)
            result = dict(sectors={}, missing=['front', 'right', 'left', 'back'])
        age = (self.get_clock().now().nanoseconds*1e-9-stamp
               if self.c['timestamps_synchronized'] else None)
        if age is not None and not 0 <= age <= self.c['cloud_timeout_s']:
            error = error or 'cloud_source_age_outside_limits'
            result.update(sectors={}, missing=['front', 'right', 'left', 'back'])
        self.health.receive(started, stamp, not error, age, (time.monotonic()-started)*1000.)
        result.update(stamp=stamp, frame=msg.header.frame_id, error=error,
                      mode=self.c['mode'], health=self.health.report(time.monotonic()))
        self.pub.publish(json_message(result))


class ImuIntegration(BenchNode):
    def __init__(self):
        super().__init__('imu_integration')
        self.integrator = ImuIntegrator(self.c['acceleration_unit'], self.c['calibration_seconds'],
            self.c['calibration_min_samples'], self.c['imu_max_dt_s'],
            self.c['calibration_gyro_limit_rad_s'], self.c['calibration_accel_std_m_s2'])
        self.pub = self.create_publisher(String, '/lidar_test/imu_estimate', 100)
        self.orientation = self.create_publisher(QuaternionStamped, '/lidar_test/orientation_est', 10)
        self.velocity = self.create_publisher(Vector3Stamped, '/lidar_test/velocity_est', 10)
        self.last_receipt = None
        self.last_watch_reason = None
        self.create_subscription(Imu, self.c['imu_topic'], self.receive, sensor_qos(200))
        self.create_timer(.05, self.watch, clock=self.wall_clock)

    def watch(self):
        reason = self.guard_reason
        if self.last_receipt is not None and time.monotonic()-self.last_receipt > self.c['imu_timeout_s']:
            reason = reason or 'imu_receipt_timeout_restart_required'
        if reason:
            self.integrator.invalidate(reason)
            if reason != self.last_watch_reason:
                self.pub.publish(json_message(self.integrator.snapshot(self.integrator.last_stamp)))
                self.last_watch_reason = reason

    def receive(self, msg):
        if not self.accepts(self.c['imu_topic']):
            return
        self.last_receipt = time.monotonic()
        stamp = stamp_s(msg.header.stamp)
        if msg.header.frame_id != self.c['expected_imu_frame']:
            self.integrator.invalidate('unexpected_imu_frame')
        if self.c['timestamps_synchronized']:
            age = self.get_clock().now().nanoseconds*1e-9-stamp
            if not 0 <= age <= self.c['imu_timeout_s']:
                self.integrator.invalidate('imu_source_age_outside_limits')
        result = self.integrator.update(stamp,
            [msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z],
            [msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z])
        self.pub.publish(json_message(result))
        if result['valid']:
            q = QuaternionStamped()
            q.header.stamp, q.header.frame_id = msg.header.stamp, 'lidar_test_initial'
            q.quaternion.x, q.quaternion.y, q.quaternion.z, q.quaternion.w = result['quaternion_xyzw']
            v = Vector3Stamped(header=q.header)
            v.vector.x, v.vector.y, v.vector.z = result['velocity_mps']
            self.orientation.publish(q)
            self.velocity.publish(v)


class HealthMonitor(BenchNode):
    def __init__(self):
        super().__init__('health_monitor')
        self.streams = {key: StreamHealth(self.c[key+'_timeout_s'], self.c['expected_'+key+'_hz'])
                        for key in ('cloud', 'imu')}
        self.frames = {}
        self.estimate_state, self.estimate_receipt = {}, None
        self.pub = self.create_publisher(String, '/lidar_test/health', 10)
        self.diag = self.create_publisher(DiagnosticArray, '/lidar_test/diagnostics', 10)
        self.create_subscription(get_message(self.c['cloud_type']), self.c['cloud_topic'],
                                 lambda msg: self.receive('cloud', msg), sensor_qos(20))
        self.create_subscription(Imu, self.c['imu_topic'], lambda msg: self.receive('imu', msg), sensor_qos(200))
        self.create_subscription(String, '/lidar_test/imu_estimate', self.estimate_cb, 20)
        self.create_timer(.5, self.report, clock=self.wall_clock)

    def estimate_cb(self, msg):
        estimate = json.loads(msg.data)
        self.estimate_state = {key: estimate.get(key) for key in ('state', 'reason', 'valid')}
        self.estimate_receipt = time.monotonic()

    def receive(self, key, msg):
        topic = self.c[key+'_topic']
        if not self.accepts(topic):
            return
        stamp = stamp_s(msg.header.stamp)
        self.frames[key] = msg.header.frame_id
        expected = self.c['expected_frame'] if key == 'cloud' else self.c['expected_imu_frame']
        valid = stamp > 0 and msg.header.frame_id == expected
        if key == 'imu':
            valid = valid and np.all(np.isfinite([msg.angular_velocity.x, msg.angular_velocity.y,
                msg.angular_velocity.z, msg.linear_acceleration.x, msg.linear_acceleration.y,
                msg.linear_acceleration.z]))
        age = self.get_clock().now().nanoseconds*1e-9-stamp if self.c['timestamps_synchronized'] else None
        valid = valid and (age is None or 0 <= age <= self.c[key+'_timeout_s'])
        self.streams[key].receive(time.monotonic(), stamp, valid, age)

    def report(self):
        data = dict(mode=self.c['mode'], guard_error=self.guard_reason,
                    sources=self.graph_snapshot, frames=self.frames,
                    imu_integration=dict(self.estimate_state,
                        receipt_age_s=None if self.estimate_receipt is None else time.monotonic()-self.estimate_receipt),
                    streams={k: v.report(time.monotonic()) for k, v in self.streams.items()})
        self.pub.publish(json_message(data))
        statuses = []
        for name, stream in data['streams'].items():
            status = DiagnosticStatus(name='mid360_test/'+name, hardware_id=self.c['device_serial'])
            status.message = self.guard_reason or stream['status']
            status.level = (DiagnosticStatus.ERROR if self.guard_reason or stream['status'] == 'FAIL'
                            else DiagnosticStatus.STALE if stream['status'] == 'NOT TESTED' else DiagnosticStatus.OK)
            status.values = [KeyValue(key=k, value=json.dumps(v)) for k, v in stream.items()]
            statuses.append(status)
        integration = DiagnosticStatus(name='mid360_test/imu_integration', hardware_id=self.c['device_serial'])
        integration.message = self.estimate_state.get('reason') or 'NO_ESTIMATE'
        integration.level = (DiagnosticStatus.STALE if self.estimate_receipt is None
            or time.monotonic()-self.estimate_receipt > self.c['imu_timeout_s'] else
            DiagnosticStatus.ERROR if self.estimate_state.get('state') == 'INVALID' else DiagnosticStatus.WARN)
        # Even a valid estimate remains WARN: pure IMU integration has unbounded drift.
        statuses.append(integration)
        out = DiagnosticArray(status=statuses)
        out.header.stamp = self.get_clock().now().to_msg()
        self.diag.publish(out)
        atomic_json(self.directory/'health.json', data)


class TestLogger(BenchNode):
    def __init__(self):
        super().__init__('test_logger')
        self.files = ReadingFiles(self.directory)
        self.start_wall = time.monotonic()
        self.cloud = {}
        self.cloud_received = self.imu_received = None
        self.estimates = deque(maxlen=2000)
        self.latest_estimate = {}
        self.health = {}
        self.rows = 0
        self.last_event = None
        self.event_file = (self.directory/'events.jsonl').open('x', buffering=1)
        atomic_json(self.directory/'session.json', dict(
            created_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),
            configuration=self.c, use_sim_time=self.get_parameter('use_sim_time').value,
            hardware_test_executed=False,
            note='Set hardware evidence only after reviewing actual received data; source identity is not authentication.'))
        self.create_subscription(String, '/lidar_test/distances', self.distance_cb, 20)
        self.create_subscription(String, '/lidar_test/imu_estimate', self.imu_cb, 200)
        self.create_subscription(String, '/lidar_test/health', self.health_cb, 10)
        self.create_timer(1./self.c['logger_hz'], self.tick, clock=self.wall_clock)

    def distance_cb(self, msg):
        self.cloud, self.cloud_received = json.loads(msg.data), time.monotonic()

    def imu_cb(self, msg):
        self.latest_estimate = json.loads(msg.data)
        self.estimates.append((time.monotonic(), self.latest_estimate))
        self.imu_received = time.monotonic()

    def health_cb(self, msg):
        self.health = json.loads(msg.data)

    def matched_estimate(self, now):
        if (not self.latest_estimate.get('valid') or self.imu_received is None
                or now-self.imu_received > self.c['imu_timeout_s']):
            return dict(valid=False, state=self.latest_estimate.get('state', 'NO_IMU'))
        if not self.c['imu_cloud_same_clock'] or self.cloud.get('stamp') is None:
            return dict(valid=False, state='IMU_CLOUD_CLOCK_UNCONFIRMED')
        stamp = self.cloud['stamp']
        candidates = [(abs(e['stamp']-stamp), e) for receipt, e in self.estimates
                      if e.get('valid') and e.get('stamp') is not None
                      and now-receipt <= self.c['cloud_timeout_s']+self.c['sync_tolerance_s']]
        if not candidates:
            return dict(valid=False, state='NO_MATCHING_IMU')
        offset, result = min(candidates, key=lambda item: item[0])
        if offset > self.c['sync_tolerance_s']:
            return dict(valid=False, state='IMU_CLOUD_TIME_MISMATCH')
        return dict(result, sync_offset_s=result['stamp']-stamp)

    def tick(self):
        now = time.monotonic()
        cloud = self.cloud
        if self.cloud_received is None or now-self.cloud_received > self.c['cloud_timeout_s']:
            cloud = dict(stamp=cloud.get('stamp'), sectors={}, missing=['front', 'right', 'left', 'back'])
            status = 'NO_DATA' if self.cloud_received is None else 'STALE_CLOUD'
        else:
            status = 'INVALID_CLOUD' if cloud.get('error') else 'OK' if not cloud['missing'] else 'MISSING_RETURNS'
        estimate = self.matched_estimate(now)
        if status in ('NO_DATA', 'STALE_CLOUD', 'INVALID_CLOUD'):
            estimate = dict(valid=False, state='CLOUD_UNAVAILABLE')
        if self.guard_reason or self.health.get('guard_error'):
            status = 'SOURCE_REJECTED'
            cloud = dict(stamp=cloud.get('stamp'), sectors={}, missing=['front', 'right', 'left', 'back'])
            estimate = dict(valid=False, state='SOURCE_REJECTED')
        recorder_path = self.directory/'recorder_status.json'
        if recorder_path.exists():
            try:
                if json.loads(recorder_path.read_text()).get('status') == 'FAIL':
                    status += ';RECORDER_ERROR'
            except (OSError, json.JSONDecodeError):
                status += ';RECORDER_STATUS_ERROR'
        elapsed = now-self.start_wall
        line = self.files.write(elapsed, cloud, estimate, status)
        print(line, flush=True)
        self.rows += 1
        event = (status, estimate.get('state'), self.guard_reason)
        if event != self.last_event:
            self.event_file.write(json.dumps(dict(elapsed_wall_s=elapsed, event=event,
                imu_reason=self.latest_estimate.get('reason')), allow_nan=False)+'\n')
            self.last_event = event
        self.save_summary()

    def save_summary(self):
        recorder = {}
        path = self.directory/'recorder_status.json'
        if path.exists():
            try:
                recorder = json.loads(path.read_text())
            except (OSError, json.JSONDecodeError):
                recorder = dict(status='FAIL', error='recorder_status_unreadable')
        streams = self.health.get('streams', {})
        states = [v['status'] for v in streams.values()]
        status = ('FAIL' if self.guard_reason or self.health.get('guard_error') or 'FAIL' in states
                  or recorder.get('status') == 'FAIL' else 'PASS'
                  if len(states) == 2 and all(s == 'PASS' for s in states) and recorder.get('status') == 'PASS'
                  else 'NOT TESTED')
        atomic_json(self.directory/'summary.json', dict(
            status=status, scope='received_stream_and_recorder_only_not_flight_or_accuracy',
            mode=self.c['mode'], hardware_validation='NOT TESTED_requires_operator_review',
            distance_accuracy='NOT TESTED_no_independent_reference',
            imu_accuracy='NOT TESTED_imu_only_integration_drifts',
            raw_recording=recorder or dict(status='NOT TESTED'), health=self.health,
            distance_processing=self.cloud.get('health', {}),
            guard_error=self.guard_reason, rows=self.rows,
            elapsed_wall_seconds=time.monotonic()-self.start_wall,
            limitations=['No UDP capture or guarantee of lossless acquisition',
                'No point deskew or localisation; moving scans may be distorted',
                'IMU integration assumes zero initial velocity; no automatic zero-velocity updates',
                'Source age unavailable unless clocks explicitly synchronized',
                'Source node allowlist is a guard against mistakes, not authentication'],
            processing_scope='distance callback only; planner/control/FCU stages NOT TESTED'))

    def destroy_node(self):
        self.save_summary()
        self.event_file.close()
        self.files.close()
        return super().destroy_node()


class RawRecorder(BenchNode):
    """Bounded async writer of original CDR bytes, before all analysis filters."""
    def __init__(self):
        super().__init__('raw_recorder')
        import rosbag2_py
        if (self.directory/'raw_bag').exists():
            raise ValueError('raw_bag already exists; use a NEW output directory')
        self.writer = rosbag2_py.SequentialWriter()
        self.writer.open(rosbag2_py.StorageOptions(uri=str(self.directory/'raw_bag'), storage_id='sqlite3'),
                         rosbag2_py.ConverterOptions('', ''))
        self.q = queue.Queue(maxsize=self.c['recorder_queue_messages'])
        self.lock = threading.Lock()
        self.queued_bytes = self.dropped = self.received = self.written = 0
        self.received_by_topic, self.written_by_topic = {}, {}
        self.error = ''
        self.closed = False
        for topic, kind in ((self.c['cloud_topic'], self.c['cloud_type']),
                            (self.c['imu_topic'], 'sensor_msgs/msg/Imu')):
            self.writer.create_topic(rosbag2_py.TopicMetadata(name=topic, type=kind, serialization_format='cdr'))
            self.received_by_topic[topic] = self.written_by_topic[topic] = 0
            self.create_subscription(get_message(kind), topic,
                lambda data, t=topic: self.receive(t, data), sensor_qos(200), raw=True)
        self.worker = threading.Thread(target=self.write_loop, name='mid360_bag_writer', daemon=True)
        self.worker.start()
        self.create_timer(.5, self.report, clock=self.wall_clock)
        self.report()

    def receive(self, topic, data):
        if not self.accepts(topic):
            return
        # This is original serialized data, not reconstructed XYZ or an analysis copy.
        payload = bytes(data)
        with self.lock:
            self.received += 1
            self.received_by_topic[topic] += 1
            if self.error or self.queued_bytes+len(payload) > self.c['recorder_max_queue_bytes']:
                self.dropped += 1
                return
            try:
                self.q.put_nowait((topic, payload, self.get_clock().now().nanoseconds))
            except queue.Full:
                self.dropped += 1
                return
            self.queued_bytes += len(payload)

    def write_loop(self):
        while True:
            item = self.q.get()
            try:
                if item is None:
                    return
                topic, data, stamp = item
                if not self.error:
                    try:
                        self.writer.write(topic, data, stamp)
                        with self.lock:
                            self.written += 1
                            self.written_by_topic[topic] += 1
                    except Exception as exc:
                        self.error = str(exc)
                with self.lock:
                    self.queued_bytes -= len(data)
            finally:
                self.q.task_done()

    def report(self):
        with self.lock:
            result = dict(status='FAIL' if self.error or self.dropped or self.guard_reason else 'PASS'
                          if all(self.written_by_topic.values()) else 'NOT TESTED',
                received_messages=self.received, written_messages=self.written,
                received_by_topic=dict(self.received_by_topic), written_by_topic=dict(self.written_by_topic),
                dropped_by_recorder=self.dropped, queued_bytes=self.queued_bytes,
                error=self.error, guard_error=self.guard_reason, closed=self.closed,
                timestamp_policy='ROS_receipt_time_bag_index_source_stamps_unchanged',
                qos='best_effort_volatile_matches_sensor_sources_no_lossless_guarantee')
        atomic_json(self.directory/'recorder_status.json', result)

    def destroy_node(self):
        # Spin has ended; drain every accepted queued message before final metadata.
        self.q.put(None)
        self.worker.join()
        self.writer = None
        gc.collect()  # Humble writer finalizes metadata in its destructor.
        self.closed = True
        self.report()
        return super().destroy_node()


def run(node_type, args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = node_type()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def cloud_adapter_main(args=None):
    run(CloudAdapter, args)


def directional_distance_main(args=None):
    run(DirectionalDistance, args)


def imu_integration_main(args=None):
    run(ImuIntegration, args)


def health_monitor_main(args=None):
    run(HealthMonitor, args)


def test_logger_main(args=None):
    run(TestLogger, args)


def raw_recorder_main(args=None):
    run(RawRecorder, args)
