"""ROS adapters. One final setpoint publisher, simulation-only MAVROS endpoint."""
import copy
import json
import math
import multiprocessing
import time
from types import SimpleNamespace
from collections import deque
from concurrent.futures import ThreadPoolExecutor, ProcessPoolExecutor
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from rclpy.time import Time
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Odometry, Path as PathMsg
from sensor_msgs.msg import PointCloud2, Imu, Range, LaserScan, JointState
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Bool, Float32, Float64, String, Header, Int32, UInt8MultiArray
from visualization_msgs.msg import Marker, MarkerArray
from mavros_msgs.msg import State, PositionTarget
from tf2_ros import Buffer, TransformBroadcaster, StaticTransformBroadcaster, TransformListener, TransformException
from .config import load_config
from .control import MissionController
from .localisation import LidarImuOdometry
from .mapping import VoxelMap, FREE, OCCUPIED, integrate_snapshot
from .map_log import MapLog, write_snapshot
from .survey import SurveyController
from .frames import FCULocalAlignment
from .takeoff import TakeoffSequence
from .runtime import CallbackTiming, measured, timed_job, PipelineTiming
from .visual_worker import build_visual_snapshot
from .map_transport import decode_window
from .coverage_mapping import (navigation_window, SparseHistory, initialise_rolling_mapper,
    integrate_rolling_snapshot, write_history_snapshot)


def seconds(stamp):
    return stamp.sec+stamp.nanosec*1e-9


def rotation(quaternion):
    values = [quaternion.x, quaternion.y, quaternion.z, quaternion.w]
    if not np.all(np.isfinite(values)) or np.linalg.norm(values) < 1e-8:
        raise ValueError('invalid quaternion')
    return Rotation.from_quat(values).as_matrix()


def vector(message):
    return np.array([message.x, message.y, message.z], float)


def set_vector(message, values):
    message.x, message.y, message.z = map(float, values)


def set_quaternion(message, matrix):
    message.x, message.y, message.z, message.w = map(float, Rotation.from_matrix(matrix).as_quat())


def raw_position_target(position, velocity, acceleration, yaw, alignment, stamp):
    """ROS fields are FCU-local ENU; MAVROS performs ENU->NED exactly once."""
    if alignment is None:
        raise ValueError('raw control requires a confirmed fixed FCU alignment')
    message = PositionTarget()
    message.header.stamp, message.header.frame_id = stamp, 'fcu_local'
    message.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
    # All XYZ position/velocity/acceleration and yaw enabled; force bit clear.
    message.type_mask = PositionTarget.IGNORE_YAW_RATE
    set_vector(message.position, alignment.to_fcu(position))
    set_vector(message.velocity, alignment.rotation@np.asarray(velocity))
    set_vector(message.acceleration_or_force, alignment.rotation@np.asarray(acceleration))
    message.yaw = float(yaw+alignment.yaw)
    return message


def cloud_points(message):
    points = np.array([tuple(row) for row in point_cloud2.read_points(message, field_names=('x', 'y', 'z'), skip_nans=True)], dtype=float).reshape(-1, 3)
    # Raw non-finite measurements are never integrated. Verified Gazebo
    # no-return rays are decoded separately by SensorGate, before mapping.
    return points[np.all(np.isfinite(points), axis=1)]


class ConfigNode(Node):
    def __init__(self, name):
        super().__init__(name)
        self.declare_parameter('config_file', '')
        path = self.get_parameter('config_file').value
        if not path:
            from ament_index_python.packages import get_package_share_directory
            path = str(Path(get_package_share_directory('polinasi_nav'))/'config/navigation.json')
        self.c = load_config(path)
        if not self.get_parameter('use_sim_time').value:
            raise RuntimeError('This prototype requires use_sim_time=true')

    def now(self):
        return self.get_clock().now().nanoseconds*1e-9


class LocalisationNode(ConfigNode):
    """Exclusive selector: truth transform OR a LiDAR+IMU estimator."""
    def __init__(self):
        super().__init__('polinasi_localisation')
        self.declare_parameter('mode', 'ground_truth')
        self.declare_parameter('world_to_odom', [0., 0., 0., 0., 0., 0.])
        self.mode = self.get_parameter('mode').value
        if self.mode not in ('ground_truth', 'sensor'):
            raise ValueError('mode must be ground_truth or sensor')
        transform = self.get_parameter('world_to_odom').value
        self.world_p = np.asarray(transform[:3])
        self.world_r = Rotation.from_euler('xyz', transform[3:]).as_matrix()
        self.pub = self.create_publisher(Odometry, '/localisation/odometry', qos_profile_sensor_data)
        self.status = self.create_publisher(Bool, '/localisation/healthy', 10)
        self.tf = TransformBroadcaster(self)
        self.static = StaticTransformBroadcaster(self)
        self.last_stamp = -math.inf
        self.last_imu = -math.inf
        self.pending_imu = deque(maxlen=500)
        self.estimator = LidarImuOdometry(self.c['lidar_mount'], self.c['initial_pose'])
        self.send_static()
        if self.mode == 'ground_truth':
            self.create_subscription(Odometry, '/simulation/ground_truth/odom', self.truth, qos_profile_sensor_data)
            self.get_logger().warning('GROUND-TRUTH LOCALISATION: planner isolation only')
        else:
            self.create_subscription(Imu, '/livox/imu', self.imu, qos_profile_sensor_data)
            self.create_subscription(PointCloud2, '/livox/lidar', self.scan, qos_profile_sensor_data)
            self.get_logger().warning('SENSOR LOCALISATION: experimental IMU-predicted ICP')
        self.create_timer(0.1, self.health)

    def send_static(self):
        transforms = []
        for parent, child, xyz, rpy in [('map', 'odom', [0., 0., 0.], [0., 0., 0.]),
                ('base_link', 'mid360', self.c['lidar_mount'][:3], self.c['lidar_mount'][3:]),
                ('mid360', 'mid360_imu', [0., 0., 0.], [0., 0., 0.])]:
            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id, t.child_frame_id = parent, child
            set_vector(t.transform.translation, xyz)
            set_quaternion(t.transform.rotation, Rotation.from_euler('xyz', rpy).as_matrix())
            transforms.append(t)
        self.static.sendTransform(transforms)

    def valid_time(self, stamp):
        value = seconds(stamp)
        return value > self.last_stamp and 0 <= self.now()-value <= self.c['pose_timeout']

    def publish(self, stamp, p, matrix, v_world, omega_body, covariance):
        if not np.all(np.isfinite(np.concatenate((p, v_world, omega_body)))):
            return
        out = Odometry()
        out.header.stamp, out.header.frame_id, out.child_frame_id = stamp, 'odom', 'base_link'
        set_vector(out.pose.pose.position, p)
        set_quaternion(out.pose.pose.orientation, matrix)
        # nav_msgs/Odometry twist must be expressed in the CHILD/body frame.
        set_vector(out.twist.twist.linear, matrix.T@v_world)
        set_vector(out.twist.twist.angular, omega_body)
        out.pose.covariance = list(np.asarray(covariance).reshape(-1))
        out.twist.covariance = list((np.eye(6)*0.05).reshape(-1))
        self.pub.publish(out)
        t = TransformStamped()
        t.header = out.header
        t.child_frame_id = 'base_link'
        set_vector(t.transform.translation, p)
        t.transform.rotation = out.pose.pose.orientation
        self.tf.sendTransform(t)
        self.last_stamp = seconds(stamp)

    def truth(self, msg):
        if msg.header.frame_id != 'world' or not self.valid_time(msg.header.stamp):
            return
        try:
            source_r = rotation(msg.pose.pose.orientation)
        except ValueError:
            return
        matrix = self.world_r@source_r
        p = self.world_r@vector(msg.pose.pose.position)+self.world_p
        # Gazebo OdometryPublisher uses child-frame velocity; rotate explicitly.
        v = matrix@vector(msg.twist.twist.linear)
        self.publish(msg.header.stamp, p, matrix, v, vector(msg.twist.twist.angular), np.eye(6)*0.01)

    def imu(self, msg):
        stamp = seconds(msg.header.stamp)
        if msg.header.frame_id != 'mid360_imu' or not 0 <= self.now()-stamp <= 0.1 or stamp <= self.last_imu:
            return
        # Clouds can arrive after newer IMU packets. Integrate only through a
        # scan's measurement time, never stamp a future-predicted pose as old.
        self.pending_imu.append((stamp, vector(msg.angular_velocity), vector(msg.linear_acceleration)))
        self.last_imu = stamp

    def scan(self, msg):
        if msg.header.frame_id != 'mid360' or not self.valid_time(msg.header.stamp):
            return
        stamp = seconds(msg.header.stamp)
        while self.pending_imu and self.pending_imu[0][0] <= stamp:
            imu_stamp, gyro, acceleration = self.pending_imu.popleft()
            self.estimator.imu(gyro, acceleration, imu_stamp)
        points = cloud_points(msg)
        ranges = np.linalg.norm(points, axis=1)
        points = points[(ranges >= self.c['lidar_min_range']) & (ranges < self.c['lidar_range']-0.05)]
        e = self.estimator
        body = points@e.mount_r.T+e.mount_p
        keep = ~np.all(np.abs(body) <= np.asarray(self.c['drone_dimensions'])/2+self.c['self_filter_padding'], axis=1)
        if e.scan(points[keep], seconds(msg.header.stamp)):
            self.publish(msg.header.stamp, e.p, e.r, e.v, [0., 0., 0.], e.covariance)

    def health(self):
        fresh = 0 <= self.now()-self.last_stamp < self.c['pose_timeout']
        if self.mode == 'sensor':
            fresh = fresh and self.estimator.ok and 0 <= self.now()-self.last_imu < 0.1
        self.status.publish(Bool(data=bool(fresh)))


class ExternalNavNode(ConfigNode):
    def __init__(self):
        super().__init__('polinasi_externalnav')
        self.last = -math.inf
        self.healthy = False
        # ROS 2 MAVROS odometry/out is the SUBSCRIBED FCU input. Verify the
        # installed plugin's graph; its in/out names differ from ROS 1 usage.
        self.pub = self.create_publisher(Odometry, '/mavros/odometry/out', 10)
        self.create_subscription(Bool, '/localisation/healthy', self.health, 10)
        self.create_subscription(Odometry, '/localisation/odometry', self.bridge, qos_profile_sensor_data)
        # MAVROS UAS::setup_static_tf owns odom_ned/base_link_frd helper
        # transforms. Do not add duplicate broadcasters for those same children.

    def health(self, msg):
        self.healthy = msg.data

    def bridge(self, msg):
        stamp = seconds(msg.header.stamp)
        diagonal = np.asarray(msg.pose.covariance).reshape(6, 6).diagonal()
        values = np.concatenate((vector(msg.pose.pose.position), vector(msg.twist.twist.linear), vector(msg.twist.twist.angular), msg.pose.covariance, msg.twist.covariance))
        if (not self.healthy or msg.header.frame_id != 'odom' or msg.child_frame_id != 'base_link'
                or not self.last < stamp or not 0 <= self.now()-stamp <= self.c['pose_timeout']
                or not np.all(np.isfinite(values)) or np.any(diagonal <= 0) or np.max(diagonal) > 1.):
            return
        try:
            rotation(msg.pose.pose.orientation)
        except ValueError:
            return
        # MAVROS owns ROS ENU/FLU -> MAVLink NED/FRD conversion. Preserve stamp.
        self.pub.publish(copy.deepcopy(msg))
        self.last = stamp


class NavigationNode(ConfigNode):
    def __init__(self, mission_kind='inspection', identification_empty_arena=False):
        super().__init__('polinasi_mapping_navigation' if mission_kind == 'mapping' else 'polinasi_navigation')
        self.mission_kind = mission_kind
        self.declare_parameter('separate_mapping', False)
        self.separate_mapping = mission_kind == 'mapping' and self.get_parameter('separate_mapping').value
        self.map_statistics = {}
        self.last_path_snapshot = -math.inf
        self.callback_timing = CallbackTiming()
        self.timing_gap_context = None
        self.visual_worker = (ProcessPoolExecutor(max_workers=1,
            mp_context=multiprocessing.get_context('spawn')) if mission_kind == 'mapping' and not self.separate_mapping else None)
        self.visual_futures = {}
        self.fcu_alignment = None
        self.mavros_ready = mission_kind != 'mapping'
        self.declare_parameter('mode', 'ground_truth')
        self.identification_empty_arena = identification_empty_arena
        self.predictive = None
        self.alignment_samples = deque(maxlen=100)
        if identification_empty_arena and not (mission_kind == 'mapping'
                and self.c.get('identification_simulation_only')
                and self.c.get('identification_empty_arena')
                and self.get_parameter('mode').value == 'ground_truth' and self.separate_mapping):
            raise ValueError('empty arena bypass is restricted to simulation identification')
        self.declare_parameter('autostart', False)
        self.declare_parameter('map_log_dir', '')
        log_dir = self.get_parameter('map_log_dir').value
        if self.separate_mapping:
            log_dir = ''  # The mapping I/O process exclusively owns this output.
        self.map_log = MapLog(log_dir, max_points=int(self.c.get('map_log_max_points', 100000))) if log_dir else None
        if log_dir and mission_kind == 'mapping':
            self.log_worker = ProcessPoolExecutor(max_workers=1, mp_context=multiprocessing.get_context('spawn'))
        else:
            self.log_worker = ThreadPoolExecutor(max_workers=1, thread_name_prefix='map-log') if log_dir else None
        self.log_future = None
        self.rolling_map = mission_kind == 'mapping' and bool(self.c.get('navigation_window_size')) and not identification_empty_arena
        self.history = SparseHistory(self.c) if self.rolling_map and not self.separate_mapping else None
        self.grid = navigation_window(self.c, self.c['initial_pose']) if self.rolling_map else VoxelMap(self.c)
        controller_type = SurveyController if mission_kind == 'mapping' else MissionController
        self.controller = controller_type(self.c, self.grid, self.get_parameter('mode').value == 'sensor')
        if self.c.get('predictive_enabled'):
            if self.get_parameter('mode').value != 'ground_truth' or mission_kind != 'mapping':
                raise ValueError('MPC prototype requires ground-truth simulation')
            from .predictive import PredictiveControl
            self.predictive = PredictiveControl(self.c)
        self.pose, self.velocity = None, np.zeros(3)
        self.state = State()
        self.started = bool(self.get_parameter('autostart').value)
        self.flight_stage = 'WAIT_EXTERNALNAV'
        self.last_tick = self.now()
        self.last_timing_gap = None
        self.last_status = {}
        self.last_map = -math.inf
        self.last_integrated_cloud = -math.inf
        self.last_flight_cmd = -math.inf
        self.stage_started = self.now()
        self.range_value, self.range_stamp = None, -math.inf
        self.camera_pitch_measured, self.camera_stamp = 0., -math.inf
        self.fc_pose, self.fc_stamp, self.fc_yaw = None, -math.inf, 0.
        self.estimated_yaw = 0.
        self.health_good_since = None
        self.takeoff_sequence = None
        self.flight_hovering, self.flight_hover_stamp = False, -math.inf
        self.reference_ready = mission_kind != 'mapping'
        self.landing_sent = False
        self.landing_started = None
        self.last_land_cmd = -math.inf
        self.pending = deque(maxlen=3)
        self.map_worker = None if self.separate_mapping else (ProcessPoolExecutor(max_workers=1, mp_context=multiprocessing.get_context('spawn'),
                           initializer=initialise_rolling_mapper if self.rolling_map else None,
                           initargs=(self.c,) if self.rolling_map else ())
                           if mission_kind == 'mapping' else ThreadPoolExecutor(max_workers=1, thread_name_prefix='occupancy'))
        self.map_future = None
        self.executed = deque(maxlen=6000)
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.camera_tf = TransformBroadcaster(self)
        self.fcu_frame_tf = StaticTransformBroadcaster(self) if mission_kind == 'mapping' else None
        self.final_interface = '/mavros/setpoint_raw/local' if mission_kind == 'mapping' else '/mavros/setpoint_position/local'
        self.final_pub = self.create_publisher(PositionTarget if mission_kind == 'mapping' else PoseStamped, self.final_interface, 10)
        self.safe_pub = self.create_publisher(PoseStamped, '/control/safe_target_pose', 10)
        self.status_pub = self.create_publisher(String, '/navigation/status', 10)
        self.buzzer_pub = self.create_publisher(Bool, '/actuators/mock_buzzer', 10)
        self.pitch_pub = self.create_publisher(Float64, '/camera/pitch_cmd', 10)
        self.mode_pub = self.create_publisher(String, '/flight/cmd/set_mode', 10)
        self.arm_pub = self.create_publisher(Bool, '/flight/cmd/set_arm', 10)
        self.takeoff_pub = self.create_publisher(Float32, '/flight/cmd/takeoff', 10)
        self.land_pub = self.create_publisher(Bool, '/flight/cmd/land', 10)
        self.map_pub = self.create_publisher(MarkerArray, '/navigation/occupancy', QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.global_map_pub = self.create_publisher(MarkerArray, '/mapping/global_occupied',
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)) if self.history else None
        self.global_cloud_pub = self.create_publisher(PointCloud2, '/mapping/global_cloud',
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)) if self.history and self.map_log else None
        self.path_pub = self.create_publisher(PathMsg, '/navigation/planned_path', 10)
        self.nominal_pub = self.create_publisher(PathMsg, '/navigation/nominal_orbit', 10)
        self.executed_pub = self.create_publisher(PathMsg, '/navigation/executed_path', 10)
        self.exploration_pub = self.create_publisher(PathMsg, '/navigation/exploration_viewpoints', 10)
        self.survey_pub = self.create_publisher(PathMsg, '/navigation/survey_route', 10) if mission_kind == 'mapping' else None
        if self.separate_mapping:
            # No visual publisher is owned by the flight-control process.
            for attribute in ('map_pub', 'path_pub', 'nominal_pub', 'executed_pub', 'exploration_pub', 'survey_pub'):
                self.destroy_publisher(getattr(self, attribute))
                setattr(self, attribute, None)
            self.flight_paths_pub = self.create_publisher(String, '/mapping/flight_paths', QoSProfile(depth=1))
            if not identification_empty_arena:
                self.create_subscription(UInt8MultiArray, '/mapping/navigation_snapshot', self.navigation_snapshot,
                    QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(Odometry, '/localisation/odometry', self.odom, qos_profile_sensor_data)
        self.create_subscription(Bool, '/localisation/healthy', self.localisation_health, 10)
        if not identification_empty_arena:
            self.create_subscription(PointCloud2, '/livox/lidar', self.cloud, qos_profile_sensor_data)
        self.create_subscription(Imu, '/livox/imu', self.imu, qos_profile_sensor_data)
        if not identification_empty_arena:
            self.create_subscription(Range, '/simulation/rangefinder', self.range_cb, qos_profile_sensor_data)
        self.create_subscription(State, '/mavros/state', self.state_cb, qos_profile_sensor_data)
        self.create_subscription(PoseStamped, '/mavros/local_position/pose', self.fc_pose_cb, qos_profile_sensor_data)
        if self.c['camera_pitch_enabled']:
            self.create_subscription(JointState, '/camera/joint_state', self.camera_feedback, qos_profile_sensor_data)
        self.create_subscription(Bool, '/mission/start', self.start_cb, 10)
        self.create_subscription(Bool, '/flight/telemetry/is_hovering', self.hover_cb, 10)
        self.create_subscription(Int32, '/flight/response/takeoff_result', self.takeoff_ack_cb, 10)
        if mission_kind == 'mapping':
            self.create_subscription(Bool, '/simulation/reference_ready', self.reference_ready_cb,
                                     QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
            self.create_subscription(Bool, '/simulation/mavros_ready', self.mavros_ready_cb,
                                     QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(Bool, '/perception/mock_flower_confirmed', self.flower, 10)
        self.create_timer(0.05, self.tick)
        if not self.separate_mapping:
            self.create_timer(0.5, self.visualise)
        if self.history:
            self.create_timer(2., self.global_visualise)
        if self.map_log:
            self.create_timer(10., self.save_map_log)

    @measured('odometry_receive')
    def odom(self, msg):
        stamp = seconds(msg.header.stamp)
        if msg.header.frame_id != 'odom' or msg.child_frame_id != 'base_link' or stamp <= self.controller.health.pose_stamp:
            return
        try:
            matrix = rotation(msg.pose.pose.orientation)
        except ValueError:
            return
        self.pose = vector(msg.pose.pose.position)
        self.velocity = matrix@vector(msg.twist.twist.linear)
        if not np.all(np.isfinite(np.concatenate((self.pose, self.velocity)))):
            self.pose = None
            return
        self.estimated_yaw = math.atan2(matrix[1, 0], matrix[0, 0])
        self.controller.health.pose_stamp = stamp
        if getattr(self, 'predictive', None) is not None:
            if not hasattr(self, 'predictive_pose_samples'):
                self.predictive_pose_samples = deque(maxlen=400)
            self.predictive_pose_samples.append((stamp, self.pose.copy(), self.velocity.copy()))
        pose = PoseStamped(header=msg.header, pose=msg.pose.pose)
        self.executed.append(pose)
        if self.map_log and (not self.map_log.executed_times or
                stamp-self.map_log.executed_times[-1] >= self.map_log.path_period):
            try:
                # Use the timestamped numerical transform, not a frame relabel.
                transform = self.buffer.lookup_transform('map', msg.header.frame_id, Time.from_msg(msg.header.stamp))
                mapped = rotation(transform.transform.rotation)@self.pose+vector(transform.transform.translation)
                self.map_log.add_pose(mapped, stamp)
            except (TransformException, ValueError):
                pass  # Missing measurement-time TF leaves a gap, never a fake path.

    def localisation_health(self, msg):
        self.controller.health.localisation_ok = msg.data

    def state_cb(self, msg):
        self.state = msg

    def hover_cb(self, msg):
        self.flight_hovering, self.flight_hover_stamp = msg.data, self.now()

    def takeoff_ack_cb(self, msg):
        if self.takeoff_sequence is not None:
            self.takeoff_sequence.acknowledge(msg.data)

    def reference_ready_cb(self, msg):
        if not self.state.armed and self.flight_stage == 'WAIT_EXTERNALNAV':
            self.reference_ready = bool(msg.data)

    def mavros_ready_cb(self, msg):
        self.mavros_ready = msg.data

    def fc_pose_cb(self, msg):
        expected = 'fcu_local' if self.mission_kind == 'mapping' else 'map'
        if msg.header.frame_id != expected:
            return
        self.fc_pose, self.fc_stamp = vector(msg.pose.position), seconds(msg.header.stamp)
        try:
            matrix = rotation(msg.pose.orientation)
            self.fc_yaw = math.atan2(matrix[1, 0], matrix[0, 0])
        except ValueError:
            self.fc_stamp = -math.inf

    def camera_feedback(self, msg):
        if 'camera_pitch' in msg.name:
            index = msg.name.index('camera_pitch')
            if index < len(msg.position) and math.isfinite(msg.position[index]):
                self.camera_pitch_measured = msg.position[index]
                self.camera_stamp = seconds(msg.header.stamp)

    def start_cb(self, msg):
        self.started = msg.data

    def flower(self, msg):
        self.controller.flower_event(msg.data)

    def imu(self, msg):
        if msg.header.frame_id == 'mid360_imu':
            self.controller.health.imu_stamp = seconds(msg.header.stamp)

    def range_cb(self, msg):
        if math.isfinite(msg.range) and msg.min_range <= msg.range <= msg.max_range:
            try:
                transform = self.buffer.lookup_transform('map', 'base_link', Time.from_msg(msg.header.stamp))
                cosine = rotation(transform.transform.rotation)[2, 2]
            except (TransformException, ValueError):
                return
            if cosine > 0.7:
                # Sensor is 5 cm below the base; project slant beam to vertical.
                self.range_value, self.range_stamp = (msg.range+0.05)*cosine, seconds(msg.header.stamp)

    @measured('lidar_receive')
    def cloud(self, msg):
        stamp = seconds(msg.header.stamp)
        if (msg.header.frame_id == 'mid360' and msg.width*msg.height > 0
                and {'x', 'y', 'z'} <= {field.name for field in msg.fields}
                and stamp > self.controller.health.cloud_stamp
                and 0 <= self.now()-stamp < self.c['cloud_timeout']):
            # Packet freshness is distinct from map coverage/processing age.
            # A sparse valid scan is not a dropout and never makes unknown free.
            self.controller.health.cloud_stamp = stamp
            if not self.separate_mapping:
                self.pending.append(msg)
                if not hasattr(self, 'scan_receipts'):
                    self.scan_receipts = {}
                self.scan_receipts[stamp] = time.monotonic_ns()
                # Latest-only scan queue; keep only the in-flight and newest trace.
                if len(self.scan_receipts) > 8:
                    self.scan_receipts.pop(next(iter(self.scan_receipts)))

    @measured('navigation_snapshot_receive')
    def navigation_snapshot(self, msg):
        try:
            grid, statistics = decode_window(bytes(msg.data), self.grid)
            if (grid.last_observation_stamp <= self.last_integrated_cloud or
                    not 0 <= self.now()-grid.last_observation_stamp < self.c['map_timeout']):
                return
            # Numeric origin changes with the rolling window; never relabel frames.
            lo, hi = np.asarray(self.c['bounds_min']), np.asarray(self.c['bounds_max'])
            if np.any(grid.lo < lo-1e-7) or np.any(grid.lo+np.asarray(grid.shape)*grid.res > hi+1e-7):
                raise ValueError('snapshot outside configured map')
            self.grid = self.controller.grid = grid
            self.last_integrated_cloud = grid.last_observation_stamp
            self.map_statistics = statistics
            self.pipeline_trace = dict(statistics.get('pipeline_trace', {}))
            self.pipeline_trace['navigation_received_ns'] = time.monotonic_ns()
        except (ValueError, KeyError, TypeError, OSError) as exc:
            self.get_logger().warning(f'Rejected navigation snapshot: {exc}')

    @measured('map_update_handoff')
    def map_pending(self):
        if self.map_future is not None:
            if not self.map_future.done():
                return
            try:
                result = self.map_future.result()
                updated_grid, stamp = result[:2]
                if self.history is not None:
                    self.history.apply_delta(result[2])
                updated_grid.version = max(updated_grid.version, self.grid.version+1)
                self.grid = updated_grid
                self.controller.grid = self.grid
                self.last_integrated_cloud = stamp
                self.last_map = self.now()
                if isinstance(result[-1], dict) and 'worker_started_ns' in result[-1]:
                    self.pipeline_trace = dict(getattr(self, 'map_job_trace', {}), **result[-1])
                    self.pipeline_trace['integrated_ns'] = time.monotonic_ns()
                    if not self.separate_mapping:
                        self.pipeline_trace['navigation_received_ns'] = time.monotonic_ns()
            except Exception as exc:
                self.get_logger().error(f'map update failed: {exc}')
            self.map_future = None
        if not self.pending:
            return
        msg = self.pending[-1]
        stamp = seconds(msg.header.stamp)
        if not 0 <= self.now()-stamp < self.c['cloud_timeout']:
            self.pending.clear()
            return
        try:
            transform = self.buffer.lookup_transform('map', msg.header.frame_id, Time.from_msg(msg.header.stamp))
            matrix = rotation(transform.transform.rotation)
        except (TransformException, ValueError):
            return
        self.pending.clear()
        points = cloud_points(msg)
        mount = self.c['lidar_mount']
        mr = Rotation.from_euler('xyz', mount[3:]).as_matrix()
        body = points@mr.T+mount[:3]
        keep = ~np.all(np.abs(body) <= np.asarray(self.c['drone_dimensions'])/2+self.c['self_filter_padding'], axis=1)
        ranges = np.linalg.norm(points, axis=1)
        keep &= np.all(np.isfinite(points), axis=1) & (ranges >= self.c['lidar_min_range']) & (ranges <= self.c['lidar_range']+0.01)
        points, ranges = points[keep], ranges[keep]
        if not len(points):
            return
        origin = vector(transform.transform.translation)
        hits = ranges < self.c['lidar_range']-0.05
        # Only finite max-range rays are no-return evidence; invalid rays dropped.
        snapshot = copy.deepcopy(self.grid)
        stride = self.c['mapping_stride']
        # Alternate the retained ray phase instead of permanently omitting rays.
        sequence = getattr(self, 'mapping_ray_sequence', 0)
        self.mapping_ray_sequence = sequence+1
        free_stride = self.c.get('mapping_no_return_stride', stride)
        from .mapping import select_rays
        selected = select_rays(hits, sequence, stride, free_stride)
        world_points, ray_hits = (points@matrix.T+origin)[selected], hits[selected]
        self.mapping_ray_counts = dict(surface_rays=int(ray_hits.sum()),
            no_return_rays=int((~ray_hits).sum()), surface_stride=stride,
            no_return_stride=free_stride, sequence=sequence)
        if self.map_log:
            # Same timestamped transform/filter as mapping, but retain surface
            # hits from all rays for display. No max-range no-return endpoints.
            self.map_log.add_hits((points@matrix.T+origin)[hits])
        pad = None
        if self.pose is not None and self.flight_stage in ('WAIT_EXTERNALNAV', 'GUIDED', 'ARM', 'TAKEOFF') and self.now()-self.range_stamp < self.c['range_timeout']:
            pad_pose = self.pose.copy()
            pad_pose[:2] = self.c['initial_pose'][:2]
            pad = (pad_pose, self.range_value)
        now = self.now()
        submitted = time.monotonic_ns()
        self.map_job_trace = dict(received_ns=getattr(self, 'scan_receipts', {}).pop(stamp, None),
                                 scan_stamp=stamp, submitted_ns=submitted)
        if self.rolling_map:
            roi = self.mapping_region() if hasattr(self, 'mapping_region') else None
            self.map_future = self.map_worker.submit(timed_job, integrate_rolling_snapshot, submitted, snapshot, origin,
                world_points, ray_hits, stamp, pad, now, self.pose.copy(), roi)
        else:
            self.map_future = self.map_worker.submit(timed_job, integrate_snapshot, submitted, snapshot, origin, world_points, ray_hits, stamp, pad, now)

    def publish_camera_tf(self):
        if self.c['camera_pitch_enabled'] and not 0 <= self.now()-self.camera_stamp < 0.3:
            return
        t = TransformStamped()
        stamp = Time(nanoseconds=int(self.camera_stamp*1e9)).to_msg() if self.c['camera_pitch_enabled'] else self.get_clock().now().to_msg()
        t.header.stamp, t.header.frame_id, t.child_frame_id = stamp, 'base_link', 'zed_camera_link'
        set_vector(t.transform.translation, self.c['camera_mount'])
        set_quaternion(t.transform.rotation, Rotation.from_euler('y', -self.camera_pitch_measured).as_matrix())
        optical = TransformStamped()
        optical.header.stamp, optical.header.frame_id, optical.child_frame_id = t.header.stamp, 'zed_camera_link', 'zed_camera_optical'
        set_quaternion(optical.transform.rotation, Rotation.from_euler('xyz', [-math.pi/2, 0., -math.pi/2]).as_matrix())
        self.camera_tf.sendTransform([t, optical])

    def flight_command(self, now):
        if self.mission_kind == 'mapping' and self.flight_stage == 'TAKEOFF':
            # Reuse the original startup sequence, not a repeating takeoff loop.
            if self.takeoff_sequence is None:
                self.takeoff_sequence = TakeoffSequence(now, float(self.pose[2]))
            action = self.takeoff_sequence.step(now, float(self.pose[2]), self.state.armed,
                self.flight_hovering and 0 <= now-self.flight_hover_stamp < .5,
                self.grid.safe(self.pose), float(np.linalg.norm(self.velocity)))
            if action == 'takeoff':
                self.takeoff_pub.publish(Float32(data=float(self.c['takeoff_altitude'])))
            elif action == 'ready':
                self.controller.command = self.pose.copy()
                self.controller.yaw = self.estimated_yaw
                self.controller.state = self.controller.airborne_state
                self.transition('MISSION')
            elif action == 'abort':
                self.controller.command = self.pose.copy()
                self.controller.fail(self.takeoff_sequence.failure)
                self.transition('MISSION')
            return
        if now-self.last_flight_cmd < 1.:
            return
        self.last_flight_cmd = now
        if self.flight_stage == 'GUIDED':
            self.mode_pub.publish(String(data='GUIDED'))
        elif self.flight_stage == 'ARM':
            self.arm_pub.publish(Bool(data=True))
        elif self.flight_stage == 'TAKEOFF' and now-self.stage_started < 5.:
            # Mapping setup confirms home at the fixed simulator origin (world
            # Z=0), not at the initial body-centre height. CommandTOL uses home.
            reference_z = 0. if self.mission_kind == 'mapping' else self.c['initial_pose'][2]
            self.takeoff_pub.publish(Float32(data=float(self.c['takeoff_altitude']-reference_z)))

    def transition(self, stage):
        self.flight_stage, self.stage_started = stage, self.now()

    @measured('control_tick')
    def tick(self):
        now = self.now()
        dt = now-self.last_tick
        self.last_tick = now
        if dt == 0:
            # Duplicate callbacks at the same simulation timestamp do not
            # advance flight. They are not evidence of a backwards clock jump.
            return
        if dt < 0 or dt > 0.2:
            # Time resets and scheduling gaps cannot replay an old trajectory.
            self.last_timing_gap = float(dt)
            self.timing_gap_context = copy.copy(self.callback_timing.last)
            if self.flight_stage == 'MISSION':
                self.get_logger().error(f'Control timing gap {dt:.6f}s; previous callback={self.timing_gap_context}; max wall ms={self.callback_timing.max_ms}')
                self.controller.fail('clock_or_scheduler_gap')
                self.publish_status(now, 'clock_or_scheduler_gap')
            return
        if not self.separate_mapping:
            self.map_pending()
        if self.mission_kind == 'mapping':
            self.grid.expire(now)
        elif now-self.last_map > 0.5:
            self.grid.rebuild(now)
            self.last_map = now
        self.publish_camera_tf()
        if self.pose is None:
            self.buzzer_pub.publish(Bool(data=False))
            self.publish_status(now, 'waiting_for_localisation')
            return
        health_reason = self.controller.health.reason(now, self.c, self.controller.sensor_mode,
            require_lidar=not self.identification_empty_arena)
        if not self.mavros_ready:
            health_reason = health_reason or 'waiting_for_mavros_parameters'
        if not self.reference_ready:
            health_reason = health_reason or 'waiting_for_confirmed_origin_and_home'
        if self.rolling_map and self.flight_stage == 'MISSION' and now-self.last_integrated_cloud > self.c.get('map_timeout', 1.):
            health_reason = health_reason or 'stale_integrated_map'
        if (self.mission_kind == 'mapping' and self.fcu_alignment is None
                and self.flight_stage == 'WAIT_EXTERNALNAV' and not self.state.armed
                and self.fc_pose is not None and not health_reason
                and abs(self.fc_stamp-self.controller.health.pose_stamp) < .05
                and np.linalg.norm(self.velocity) < .05
                and np.linalg.norm(self.fc_pose-self.pose) <= .35
                and abs(math.atan2(math.sin(self.fc_yaw-self.estimated_yaw), math.cos(self.fc_yaw-self.estimated_yaw))) < .2
                and self.alignment_settled(now)):
            self.fcu_alignment = FCULocalAlignment(self.pose, self.estimated_yaw, self.fc_pose, self.fc_yaw)
            transform = TransformStamped()
            transform.header.stamp, transform.header.frame_id, transform.child_frame_id = self.get_clock().now().to_msg(), 'map', 'fcu_local'
            set_vector(transform.transform.translation, self.fcu_alignment.fcu_origin_in_map)
            set_quaternion(transform.transform.rotation, self.fcu_alignment.rotation.T)
            self.fcu_frame_tf.sendTransform(transform)
            self.get_logger().info(f'Fixed map-to-FCU alignment: translation={self.fcu_alignment.translation.tolist()}, yaw={self.fcu_alignment.yaw}')
        fc_map = self.fcu_alignment.to_map(self.fc_pose) if self.fcu_alignment is not None and self.fc_pose is not None else self.fc_pose
        fc_map_yaw = self.fc_yaw-(self.fcu_alignment.yaw if self.fcu_alignment is not None else 0.)
        if self.fc_pose is None or not 0 <= now-self.fc_stamp < self.c['pose_timeout']:
            health_reason = health_reason or 'stale_fcu_pose'
        elif self.mission_kind == 'mapping' and self.fcu_alignment is None:
            health_reason = health_reason or 'waiting_for_fcu_frame_alignment'
        elif np.linalg.norm(fc_map-self.pose) > 0.25 or abs(math.atan2(math.sin(fc_map_yaw-self.estimated_yaw), math.cos(fc_map_yaw-self.estimated_yaw))) > 0.2:
            health_reason = health_reason or 'fcu_externalnav_alignment'
        self.controller.camera_feedback_ok = (not self.c['camera_pitch_enabled'] or
            (0 <= now-self.camera_stamp < 0.3 and abs(self.camera_pitch_measured-self.controller.desired_pitch) < 0.1))
        self.health_good_since = None if health_reason else (self.health_good_since if self.health_good_since is not None else now)
        takeoff_top = np.array([*self.c['home'][:2], self.c['takeoff_altitude']])
        if self.started and self.flight_stage == 'WAIT_EXTERNALNAV' and self.state.connected and self.grid.safe(takeoff_top) and self.health_good_since is not None and now-self.health_good_since > 2.:
            self.transition('GUIDED')
        elif self.flight_stage == 'GUIDED' and self.state.mode == 'GUIDED':
            self.transition('ARM')
        elif self.flight_stage == 'ARM' and self.state.armed:
            self.transition('TAKEOFF')
        elif self.mission_kind != 'mapping' and self.flight_stage == 'TAKEOFF' and abs(self.pose[2]-self.c['takeoff_altitude']) < 0.15 and np.linalg.norm(self.velocity) < 0.15:
            self.controller.command = self.pose.copy()
            self.controller.state = self.controller.airborne_state
            self.transition('MISSION')
        if self.flight_stage in ('GUIDED', 'ARM', 'TAKEOFF'):
            if health_reason or now-self.stage_started > 45.:
                self.controller.command = self.pose.copy()
                self.controller.fail(health_reason or 'flight_management_timeout')
                self.transition('MISSION')
            else:
                self.flight_command(now)
        if self.flight_stage == 'MISSION':
            if health_reason and not self.landing_sent:
                if self.controller.failure == 'mpc_recovering':
                    # A recoverable MPC delay cannot mask stale map/FCU inputs.
                    self.controller.failure = health_reason
                self.controller.fail(health_reason)
            # Operator mode changes stop publication; takeover cannot be undone
            # by an old mission command or periodic GUIDED retry.
            if self.state.mode != 'GUIDED' and not self.landing_sent:
                self.transition('TAKEOVER')
                return
            if not self.landing_sent:
                command, _ = self.control_step(now, dt)
                pose = PoseStamped()
                pose.header.stamp, pose.header.frame_id = self.get_clock().now().to_msg(), 'map'
                set_vector(pose.pose.position, command)
                set_quaternion(pose.pose.orientation, Rotation.from_euler('z', self.controller.yaw).as_matrix())
                self.safe_pub.publish(pose)
                if self.fcu_alignment is not None:
                    pose = copy.deepcopy(pose)
                    pose.header.frame_id = 'fcu_local'
                    set_vector(pose.pose.position, self.fcu_alignment.to_fcu(command))
                    set_quaternion(pose.pose.orientation, Rotation.from_euler('z', self.controller.yaw+self.fcu_alignment.yaw).as_matrix())
                if self.mission_kind == 'mapping':
                    self.final_pub.publish(raw_position_target(command, self.controller.command_v,
                        self.controller.command_a, self.controller.yaw, self.fcu_alignment, pose.header.stamp))
                else:
                    self.final_pub.publish(pose)
                if not hasattr(self, 'pipeline_timing'):
                    self.pipeline_timing = PipelineTiming()
                self.pipeline_timing.record(getattr(self, 'pipeline_trace', None), self.grid.version,
                    time.monotonic_ns(), now-self.last_integrated_cloud)
                if self.predictive is not None:
                    self.predictive.record(seconds(pose.header.stamp), command,
                        self.controller.command_v, self.controller.command_a)
            if self.controller.land_requested and not self.landing_sent:
                self.land_pub.publish(Bool(data=True))
                self.landing_sent = True
                self.landing_started = self.last_land_cmd = now
            if self.landing_sent and self.state.armed and self.state.mode != 'LAND' and now-self.last_land_cmd > 3.:
                self.land_pub.publish(Bool(data=True))
                self.last_land_cmd = now
            if self.landing_sent and self.state.armed and now-self.landing_started > 45.:
                self.controller.failure = 'landing_timeout'
            if self.landing_sent and not self.state.armed and self.landing_confirmed(now):
                self.controller.state = 'COMPLETE'
        self.buzzer_pub.publish(Bool(data=self.controller.buzzer_remaining > 0))
        self.pitch_pub.publish(Float64(data=float(self.controller.pitch)))
        self.publish_status(now, health_reason or '')

    def landing_confirmed(self, now):
        if self.identification_empty_arena:
            # This fixture has a known flat floor and simulator ground truth.
            # Do not reuse this gate for sensor-based or orchard navigation.
            return bool(self.pose is not None and self.state.mode == 'LAND'
                and 0 <= now-self.controller.health.pose_stamp < self.c['pose_timeout']
                and abs(self.pose[2]-self.c['initial_pose'][2]) < .1
                and np.linalg.norm(self.velocity) < .1)
        return (self.range_value is not None and 0 <= now-self.range_stamp < self.c['range_timeout']
                and self.range_value < .3)

    def alignment_settled(self, now):
        if self.predictive is None:
            return True
        if not self.predictive.ready():
            return False
        if abs(self.fc_stamp-self.controller.health.pose_stamp) > .025:
            return False
        row = np.r_[self.pose-self.fc_pose, self.estimated_yaw-self.fc_yaw]
        self.alignment_samples.append((now, row))
        if len(self.alignment_samples) < 30 or now-self.alignment_samples[0][0] < 3.:
            return False
        values = np.asarray([r for _, r in self.alignment_samples])
        return bool(np.max(np.ptp(values, axis=0)) < .003)

    @measured('trajectory_and_safety')
    def control_step(self, now, dt):
        waiting = (self.predictive is not None and self.predictive.awaiting_fresh
                   and not self.controller.failure and self.controller.state in ('SURVEY', 'RETURN'))
        self.controller.mpc_waiting_for_fresh = waiting
        result = self.controller.tick(now, dt, self.pose, self.velocity, self.range_value, self.range_stamp)
        if self.predictive is not None:
            self.predictive.sync_state(self.controller.state)
            if self.controller.failure == 'mpc_recovering':
                self.predictive.poll_recovery(now, self.controller, self.pose, self.velocity)
                return result
            if not self.controller.failure and self.controller.state in ('IDENTIFY', 'SURVEY', 'RETURN'):
                if hasattr(self.controller, 'predictive_reference'):
                    reference = self.controller.predictive_reference()
                elif self.controller.trajectory is not None:
                    rate = getattr(getattr(self.controller, 'phase', None), 'rate', 1.)
                    rows = []
                    for k in range(10):
                        p, v, a = self.controller.trajectory.sample(min(self.controller.elapsed+k*.1*rate,
                                                                      self.controller.trajectory.duration))
                        rows.append(np.r_[p, v*rate, a*rate**2])
                    reference = np.asarray(rows)
                else:
                    reference = np.tile(np.r_[self.controller.command, self.controller.command_v,
                                              self.controller.command_a], (10, 1))
                from .frames import position_at
                matched = position_at(getattr(self, 'predictive_pose_samples', []), self.fc_stamp)
                fc_map = self.fcu_alignment.to_map(self.fc_pose)
                error = float(np.linalg.norm(fc_map-matched)) if matched is not None else math.inf
                self.predictive_alignment = dict(raw_error_m=float(np.linalg.norm(fc_map-self.pose)),
                    matched_error_m=error if math.isfinite(error) else None,
                    timestamp_difference_seconds=self.fc_stamp-self.controller.health.pose_stamp,
                    time_matching_available=matched is not None)
                result = self.predictive.apply(now, dt, self.controller, self.pose, self.velocity, reference, error)
        return result

    def publish_status(self, now, health_reason=''):
        status = {'simulation_time': float(now), 'mode': self.get_parameter('mode').value, 'flight': self.flight_stage,
                   'route_blockage': self.controller.route_blockage,
                   'speed_diagnostics': getattr(self.controller, 'speed_diagnostics', {}),
                   'predictive_alignment': getattr(self, 'predictive_alignment', {}),
                   'callback_wall_max_ms': dict(self.callback_timing.max_ms),
                   'planner_worker_wall_max_ms': getattr(self.controller, 'planner_wall_max_ms', None),
                   'planner_stages_last_wall_ms': getattr(self.controller, 'planner_stages_ms', {}),
                   'planner_job_id': getattr(self.controller, 'planner_job_id', None),
                   'trajectory_timing': dict(
                       policy=getattr(self.controller.trajectory, 'timing_policy', 'stop_to_stop'),
                       durations=getattr(self.controller.trajectory, 'durations', np.array([])).tolist(),
                       knot_speeds=getattr(self.controller.trajectory, 'knot_speeds', np.array([])).tolist(),
                       geometry_cache_hits=getattr(self.controller.trajectory, 'geometry_cache_hits', 0),
                       geometry_checks=getattr(self.controller.trajectory, 'geometry_checks', 0),
                       prefetch_submitted=getattr(self.controller, 'prefetch_submitted', 0),
                       prefetch_used=getattr(self.controller, 'prefetch_used', 0),
                       prefetch_rejected=getattr(self.controller, 'prefetch_rejected', 0),
                       prefetch_reason=getattr(self.controller, 'prefetch_reason', ''),
                       handoff='certified_stop_to_stop; moving_handoff_not_enabled'),
                   'processing_pipeline': self.pipeline_timing.status() if hasattr(self, 'pipeline_timing') else None,
                   'timing_gap_previous_callback': self.timing_gap_context,
                   'setpoint_interface': self.final_interface,
                   'continuous_survey': self.c.get('continuous_survey', False),
                   'continuous_passes': getattr(self.controller, 'continuous_passes', 0),
                   'batch_end_index': getattr(self.controller, 'batch_end_index', None),
                   'speed_scale': self.controller.phase.rate if isinstance(self.controller, SurveyController) else 1.,
                   'adaptive_reason': getattr(self.controller, 'adaptive_reason', ''),
                   'commanded_speed_mps': float(np.linalg.norm(self.controller.command_v)),
                   'measured_speed_mps': float(np.linalg.norm(self.velocity)),
                   'commanded_acceleration_mps2': float(np.linalg.norm(self.controller.command_a)),
                   'tracking_error_m': float(np.linalg.norm(self.pose-self.controller.command)) if self.pose is not None else None,
                   'map_clearance_m': self.grid.clearance(self.pose) if self.pose is not None else None,
                   'mission_kind': self.mission_kind,
                   'survey_waypoint': getattr(self.controller, 'waypoint_index', None),
                   'survey_waypoint_count': len(self.controller.waypoints) if self.mission_kind == 'mapping' else None,
                   'survey_planning': getattr(self.controller, 'pending_plan', None) is not None,
                   'mission_start_received': self.started,
                   'takeoff_phase': self.takeoff_sequence.phase if self.takeoff_sequence else None,
                   'takeoff_attempts': self.takeoff_sequence.attempts if self.takeoff_sequence else 0,
                   'takeoff_result': self.takeoff_sequence.result if self.takeoff_sequence else None,
                   'armed': self.state.armed, 'fcu_mode': self.state.mode,
                   'altitude_m': float(self.pose[2]) if self.pose is not None else None,
                   'last_timing_gap_seconds': self.last_timing_gap,
                   'fcu_frame_aligned': self.fcu_alignment is not None if self.mission_kind == 'mapping' else None,
                   'mavros_parameters_ready': self.mavros_ready,
                   'reference_ready': self.reference_ready,
                   'takeoff_space_observed': self.grid.safe([*self.c['home'][:2], self.c['takeoff_altitude']]),
                   'navigation_window_origin': self.grid.lo.tolist(),
                   'navigation_window_shape': list(map(int, self.grid.shape)),
                   'global_known_voxels': len(self.history.cells) if self.history else None,
                   'observed_volume_fraction': len(self.history.cells)/int(np.prod(self.history.shape)) if self.history else None,
                   'observed_xy_fraction': len(self.history.observed_columns)/(self.history.shape[0]*self.history.shape[1]) if self.history else None,
                   'global_map_origin': self.history.lo.tolist() if self.history else self.grid.lo.tolist(),
                   'global_map_shape': list(map(int, self.history.shape)) if self.history else list(map(int, self.grid.shape)),
                   'mission': self.controller.state, 'failure': self.controller.failure, 'replans': self.controller.replans,
                   'buzzer_events': self.controller.buzzer_events, 'health_reason': health_reason,
                   'exploration_phase': self.controller.exploration_phase,
                   'exploration_views': self.controller.exploration_views,
                   'exploration_predicted_gain': self.controller.exploration_gain,
                   'map_unknown_cells': int(np.count_nonzero(self.grid.state == -1)),
                   'map_version': self.grid.version,
                   'map_measurement_age': now-self.last_integrated_cloud if math.isfinite(self.last_integrated_cloud) else None,
                   'cloud_age': now-self.controller.health.cloud_stamp if math.isfinite(self.controller.health.cloud_stamp) else None}
        status.update(self.extra_status())
        if self.predictive is not None:
            status.update(self.predictive.status())
            status['predictive_ever_active'] = self.predictive.activated
            if self.controller.state not in ('IDENTIFY', 'SURVEY', 'RETURN'):
                status['predictive_active'] = False
        self.last_status = status
        if self.separate_mapping:
            status.update(self.map_statistics)
            status['runtime_isolation'] = ('identification_no_mapping' if self.identification_empty_arena
                                           else 'mapping_io_process')
            # A small, bounded numeric route snapshot replaces whole-controller
            # copies and huge path messages in this process.
            if now-self.last_path_snapshot >= .5:
                trajectory = self.controller.trajectory
                planned = ([trajectory.sample(t)[0].tolist() for t in np.linspace(0., trajectory.duration, 80)]
                           if trajectory is not None else [])
                self.flight_paths_pub.publish(String(data=json.dumps(dict(planned=planned,
                    exploration=np.asarray(self.controller.exploration_points).tolist()))))
                self.last_path_snapshot = now
        event = (status['mission'], status['failure'], status['adaptive_reason'],
                 status['route_blockage'].get('reason'))
        if event != getattr(self, 'last_decision_event', None):
            self.last_decision_event = event
            self.get_logger().info('Navigation decision: '+json.dumps({key:status[key] for key in
                ('simulation_time', 'mission', 'failure', 'survey_waypoint', 'adaptive_reason',
                 'route_blockage', 'speed_diagnostics')}))
        self.status_pub.publish(String(data=json.dumps(status)))
        if now-getattr(self, 'last_processing_log', -math.inf) >= 5.:
            self.last_processing_log = now
            self.get_logger().info('Processing timing: '+json.dumps(dict(
                pipeline=status['processing_pipeline'], planner=status['planner_stages_last_wall_ms'])))

    def extra_status(self):
        return {}

    @measured('map_log_snapshot')
    def save_map_log(self):
        if not self.map_log:
            return
        if self.log_future is not None:
            if not self.log_future.done():
                return
            try:
                self.log_future.result()
            except Exception as exc:
                self.get_logger().error(f'3D map log failed: {exc}')
        # Fine-grid enumeration/file writing stays off the controller callback.
        grid = SimpleNamespace(res=self.grid.res, lo=self.grid.lo.copy(), shape=self.grid.shape,
                               state=self.grid.state.copy(), last_observation_stamp=self.grid.last_observation_stamp)
        points = list(self.map_log.points.values())
        truncated = self.map_log.truncated
        mode, stamp = self.get_parameter('mode').value, self.now()
        status = copy.deepcopy(self.last_status)
        flight_paths = self.flight_path_snapshot()
        if self.history is not None:
            history = copy.copy(self.history)
            # Values are immutable tuples: bounded shallow snapshot on callback,
            # expensive enumeration/serialization belongs to the export process.
            history.cells = self.history.cells.copy()
            history.prior = self.history.prior.copy()
            history.occupied_keys = self.history.occupied_keys.copy()
            history.observed_columns = self.history.observed_columns.copy()
            self.log_future = self.log_worker.submit(write_history_snapshot, self.map_log.directory,
                history, mode, stamp, points, truncated, status, flight_paths)
            return
        self.log_future = self.log_worker.submit(write_snapshot, self.map_log.directory, grid, mode, stamp,
                                                points, truncated, self.mission_kind, status, flight_paths)

    def flight_path_snapshot(self):
        nominal = (np.vstack((self.c['home'], self.controller.waypoints))
                   if self.mission_kind == 'mapping' else self.controller.nominal)
        trajectory = self.controller.trajectory
        planned = ([trajectory.sample(t)[0] for t in np.linspace(0., trajectory.duration, 150)]
                   if trajectory is not None else [])
        return self.map_log.flight_paths(nominal, planned)

    def destroy_node(self):
        if self.predictive is not None:
            self.predictive.close()
        if isinstance(self.controller, SurveyController):
            self.controller.close()
        if self.map_worker is not None:
            self.map_worker.shutdown(wait=True, cancel_futures=True)
        if self.visual_worker is not None:
            self.visual_worker.shutdown(wait=True, cancel_futures=True)
        if self.map_log:
            self.log_worker.shutdown(wait=True)
            try:
                data = (self.history.snapshot(self.get_parameter('mode').value, self.now(),
                    list(self.map_log.points.values()), self.map_log.truncated, self.last_status)
                    if self.history is not None else self.map_log.snapshot(self.grid, self.get_parameter('mode').value, self.now()))
                data.update(mission_kind=self.mission_kind, mission_status=self.last_status)
                data['flight_paths'] = self.flight_path_snapshot()
                self.map_log.write(data)
            except Exception as exc:
                self.get_logger().error(f'Final 3D map log failed: {exc}')
        super().destroy_node()


    def path_message(self, points):
        message = PathMsg()
        message.header.stamp, message.header.frame_id = self.get_clock().now().to_msg(), 'map'
        for point in points:
            pose = PoseStamped(header=message.header)
            set_vector(pose.pose.position, point)
            pose.pose.orientation.w = 1.
            message.poses.append(pose)
        return message

    def _visual_job(self, kind, data):
        future = self.visual_futures.get(kind)
        if future is not None:
            if not future.done():
                return
            try:
                messages, duration = future.result()
                self.callback_timing.record('visual_worker_'+kind, duration)
                for publisher, message in messages.items():
                    getattr(self, publisher).publish(message)
            except Exception as exc:
                self.get_logger().warning(f'Visual-only worker failed: {exc}')
            self.visual_futures.pop(kind, None)
        self.visual_futures[kind] = self.visual_worker.submit(build_visual_snapshot, data)

    @measured('global_visual_handoff')
    def global_visualise(self):
        if self.history is None:
            return
        if self.visual_worker is not None and 'global' in self.visual_futures and not self.visual_futures['global'].done():
            return
        from geometry_msgs.msg import Point
        keys = np.fromiter(self.history.occupied_keys, dtype=np.int64)
        keys = keys[::max(1, int(np.ceil(len(keys)/12000)))]
        ids = np.column_stack(np.unravel_index(keys, self.history.shape)) if len(keys) else np.empty((0, 3))
        if self.visual_worker is not None:
            stamp = self.get_clock().now().to_msg()
            points = list(self.map_log.points.values()) if self.map_log else None
            if points is not None:
                points = points[::max(1, int(np.ceil(len(points)/30000)))]
            self._visual_job('global', dict(stamp=(stamp.sec, stamp.nanosec),
                global_points=self.history.lo+(ids+.5)*self.history.res,
                resolution=self.history.res, cloud_points=points))
            return
        marker = Marker()
        marker.header.frame_id, marker.header.stamp = 'map', self.get_clock().now().to_msg()
        marker.ns, marker.id = 'global_observed_occupied', 0
        marker.type, marker.action = Marker.CUBE_LIST, Marker.ADD
        marker.pose.orientation.w = 1.
        marker.scale.x = marker.scale.y = marker.scale.z = self.history.res
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = 1., .35, .05, .65
        marker.points = [Point(x=float(p[0]), y=float(p[1]), z=float(p[2]))
                         for p in self.history.lo+(ids+.5)*self.history.res]
        self.global_map_pub.publish(MarkerArray(markers=[marker]))
        if self.global_cloud_pub:
            points = list(self.map_log.points.values())
            points = points[::max(1, int(np.ceil(len(points)/30000)))]
            header = Header(frame_id='map', stamp=self.get_clock().now().to_msg())
            self.global_cloud_pub.publish(point_cloud2.create_cloud_xyz32(header, points))

    @measured('local_visual_handoff')
    def visualise(self):
        if self.visual_worker is not None:
            if 'local' in self.visual_futures and not self.visual_futures['local'].done():
                return
            stamp = self.get_clock().now().to_msg()
            routes = dict(nominal_pub=self.controller.nominal,
                          exploration_pub=self.controller.exploration_points)
            if self.survey_pub is not None:
                routes['survey_pub'] = self.controller.waypoints.copy()
            grid = SimpleNamespace(res=self.grid.res, lo=self.grid.lo.copy(), state=self.grid.state.copy())
            self._visual_job('local', dict(stamp=(stamp.sec, stamp.nanosec), grid=grid,
                trajectory=copy.deepcopy(self.controller.trajectory), routes=copy.deepcopy(routes),
                executed=list(self.executed)))
            return
        array = MarkerArray()
        for marker_id, state, colour in [(0, OCCUPIED, [1., 0.2, 0.1, 0.6]), (1, FREE, [0.1, 0.8, 0.2, 0.05]), (2, -1, [0.5, 0.5, 0.5, 0.03])]:
            marker = Marker()
            marker.header.frame_id, marker.header.stamp = 'map', self.get_clock().now().to_msg()
            marker.ns, marker.id, marker.type, marker.action = 'voxels', marker_id, Marker.CUBE_LIST, Marker.ADD
            marker.pose.orientation.w = 1.
            marker.scale.x = marker.scale.y = marker.scale.z = self.grid.res
            marker.color.r, marker.color.g, marker.color.b, marker.color.a = colour
            from geometry_msgs.msg import Point
            ids = np.argwhere(self.grid.state == state)
            # Bound VISUAL messages only, not the navigation map or JSON.
            stride = max(1, int(np.ceil(len(ids)/6000)))
            for p in self.grid.centers(ids[::stride]):
                point = Point()
                set_vector(point, p)
                marker.points.append(point)
            array.markers.append(marker)
        self.map_pub.publish(array)
        if self.controller.trajectory is not None:
            tr = self.controller.trajectory
            points = [tr.sample(t)[0] for t in np.linspace(0., tr.duration, 150)]
            self.path_pub.publish(self.path_message(points))
        self.nominal_pub.publish(self.path_message(self.controller.nominal))
        if self.survey_pub is not None:
            self.survey_pub.publish(self.path_message(self.controller.waypoints))
        self.exploration_pub.publish(self.path_message(self.controller.exploration_points))
        executed = PathMsg()
        executed.header.frame_id, executed.header.stamp = 'odom', self.get_clock().now().to_msg()
        executed.poses = list(self.executed)
        self.executed_pub.publish(executed)


class SensorGate(ConfigNode):
    """Repeatable cloud dropout injection, timed from explicit mission start."""
    def __init__(self):
        super().__init__('polinasi_sensor_gate')
        self.declare_parameter('drop_after', -1.)
        self.started_at = None
        self.pub = self.create_publisher(PointCloud2, '/livox/lidar', qos_profile_sensor_data)
        self.rays_pub = self.create_publisher(PointCloud2, '/mapping/lidar_rays', qos_profile_sensor_data)
        self.ray_status_pub = self.create_publisher(String, '/simulation/lidar_ray_status', 1)
        self.create_subscription(PointCloud2, '/simulation/lidar', self.cloud, qos_profile_sensor_data)
        self.create_subscription(Bool, '/mission/start', self.start, 10)

    def start(self, msg):
        if msg.data and self.started_at is None:
            self.started_at = self.now()

    def cloud(self, msg):
        drop = self.get_parameter('drop_after').value
        if drop >= 0 and self.started_at is not None and self.now()-self.started_at > drop:
            return
        self.pub.publish(msg)
        if self.c.get('simulation_no_return_rays', False):
            from .simulated_rays import normalize
            try:
                if msg.width != 360 or msg.height != 32:
                    raise ValueError('unexpected Gazebo scan dimensions')
                rows = np.array([tuple(r) for r in point_cloud2.read_points(msg,
                    field_names=('x', 'y', 'z', 'ring'), skip_nans=False)], dtype=float)
                points, stats = normalize(rows[:, :3], rows[:, 3], self.c['lidar_range'])
                rays = point_cloud2.create_cloud_xyz32(msg.header, points)
                self.rays_pub.publish(rays)
                stats['measurement_stamp'] = seconds(msg.header.stamp)
                self.ray_status_pub.publish(String(data=json.dumps(stats)))
            except (ValueError, AssertionError, KeyError) as exc:
                # Failed schema/geometry verification supplies no invented rays.
                self.rays_pub.publish(msg)
                self.ray_status_pub.publish(String(data=json.dumps(dict(error=str(exc)))))


class RangeBridge(ConfigNode):
    def __init__(self):
        super().__init__('polinasi_range_bridge')
        self.pub = self.create_publisher(Range, '/simulation/rangefinder', qos_profile_sensor_data)
        self.create_subscription(LaserScan, '/simulation/range_scan', self.callback, qos_profile_sensor_data)

    def callback(self, msg):
        valid = [x for x in msg.ranges if math.isfinite(x) and msg.range_min <= x <= msg.range_max]
        if valid:
            out = Range()
            out.header = msg.header
            out.radiation_type, out.field_of_view = Range.INFRARED, 0.02
            out.min_range, out.max_range, out.range = msg.range_min, msg.range_max, min(valid)
            self.pub.publish(out)


def run(node_type, args=None):
    rclpy.init(args=args)
    node = node_type()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception:
        # ROS shutdown can invalidate a publisher during an in-flight callback.
        # Only suppress that shutdown race; live exceptions still propagate.
        if rclpy.ok():
            raise
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def navigation_main(args=None):
    run(NavigationNode, args)


class MappingNavigationNode(NavigationNode):
    def __init__(self):
        super().__init__('mapping')


def mapping_main(args=None):
    run(MappingNavigationNode, args)


def localisation_main(args=None):
    run(LocalisationNode, args)


def externalnav_main(args=None):
    run(ExternalNavNode, args)


def range_main(args=None):
    run(RangeBridge, args)


def gate_main(args=None):
    run(SensorGate, args)
