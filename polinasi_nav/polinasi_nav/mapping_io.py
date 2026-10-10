"""Simulation mapping/visual/export process: deliberately has no flight publishers."""
import copy
import json
import math
import multiprocessing
from array import array
from collections import deque
from concurrent.futures import ProcessPoolExecutor
from types import SimpleNamespace
import numpy as np
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from std_msgs.msg import String, UInt8MultiArray
from nav_msgs.msg import Odometry, Path as PathMsg
from sensor_msgs.msg import PointCloud2, Range
from mavros_msgs.msg import State
from visualization_msgs.msg import MarkerArray
from tf2_ros import Buffer, TransformListener
from .ros_nodes import ConfigNode, NavigationNode, run
from .coverage_mapping import navigation_window, SparseHistory, initialise_rolling_mapper
from .map_log import MapLog
from .map_transport import encode_window
from .runtime import CallbackTiming, measured


class MappingIONode(ConfigNode):
    # Reuse tested acquisition/export routines, not flight-management startup.
    cloud = NavigationNode.cloud
    map_pending = NavigationNode.map_pending
    odom = NavigationNode.odom
    range_cb = NavigationNode.range_cb
    save_map_log = NavigationNode.save_map_log
    global_visualise = NavigationNode.global_visualise
    _visual_job = NavigationNode._visual_job

    def __init__(self):
        super().__init__('polinasi_mapping_io')
        self.declare_parameter('mode', 'ground_truth')
        self.declare_parameter('map_log_dir', '')
        self.mission_kind, self.separate_mapping, self.rolling_map = 'mapping', False, True
        self.callback_timing = CallbackTiming()
        self.grid = navigation_window(self.c, self.c['initial_pose'])
        self.history = SparseHistory(self.c)
        self.controller = SimpleNamespace(health=SimpleNamespace(cloud_stamp=-math.inf, pose_stamp=-math.inf),
            nominal=[], exploration_points=[], waypoints=np.asarray(self.c['survey_waypoints']), trajectory=None)
        self.pose, self.velocity, self.estimated_yaw = None, np.zeros(3), 0.
        self.flight_stage, self.range_value, self.range_stamp = 'WAIT_EXTERNALNAV', None, -math.inf
        self.state = State()
        self.pending, self.executed = deque(maxlen=1), deque(maxlen=6000)
        self.last_map = self.last_integrated_cloud = -math.inf
        self.last_status, self.planned = {}, []
        self.last_snapshot_version = -1
        self.map_future = self.log_future = None
        self.visual_futures = {}
        context = multiprocessing.get_context('spawn')
        self.map_worker = ProcessPoolExecutor(max_workers=1, mp_context=context,
            initializer=initialise_rolling_mapper, initargs=(self.c,))
        self.visual_worker = ProcessPoolExecutor(max_workers=1, mp_context=context)
        directory = self.get_parameter('map_log_dir').value
        self.map_log = MapLog(directory, max_points=int(self.c.get('map_log_max_points', 100000))) if directory else None
        self.log_worker = ProcessPoolExecutor(max_workers=1, mp_context=context) if directory else None
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        visual_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.snapshot_pub = self.create_publisher(UInt8MultiArray, '/mapping/navigation_snapshot', visual_qos)
        for attribute, message, topic in (
            ('map_pub', MarkerArray, '/navigation/occupancy'),
            ('global_map_pub', MarkerArray, '/mapping/global_occupied'),
            ('global_cloud_pub', PointCloud2, '/mapping/global_cloud'),
            ('path_pub', PathMsg, '/navigation/planned_path'),
            ('nominal_pub', PathMsg, '/navigation/nominal_orbit'),
            ('executed_pub', PathMsg, '/navigation/executed_path'),
            ('exploration_pub', PathMsg, '/navigation/exploration_viewpoints'),
            ('survey_pub', PathMsg, '/navigation/survey_route')):
            setattr(self, attribute, self.create_publisher(message, topic, visual_qos))
        self.simulation_ray_stats = {}
        from .corridor_mapping import CorridorRegion
        self.corridor = CorridorRegion(self.c) if self.c.get('mapping_corridor_enabled', False) else None
        lidar_topic = '/mapping/lidar_rays' if self.c.get('simulation_no_return_rays', False) else '/livox/lidar'
        self.create_subscription(PointCloud2, lidar_topic, self.cloud, qos_profile_sensor_data)
        self.create_subscription(String, '/simulation/lidar_ray_status',
            lambda msg: setattr(self, 'simulation_ray_stats', json.loads(msg.data)), 1)
        self.create_subscription(Odometry, '/localisation/odometry', self.odom, qos_profile_sensor_data)
        self.create_subscription(Range, '/simulation/rangefinder', self.range_cb, qos_profile_sensor_data)
        self.create_subscription(String, '/navigation/status', self.navigation_status, QoSProfile(depth=1))
        self.create_subscription(String, '/mapping/flight_paths', self.flight_paths, QoSProfile(depth=1))
        self.create_subscription(State, '/mavros/state', self.fcu_state, qos_profile_sensor_data)
        self.create_timer(.1, self.update)
        self.create_timer(1., self.visualise)
        self.create_timer(3., self.global_visualise)
        if self.map_log:
            self.create_timer(10., self.save_map_log)

    def navigation_status(self, msg):
        self.last_status = json.loads(msg.data)
        self.flight_stage = self.last_status['flight']
        if self.map_log:
            self.map_log.record_debug(self.last_status)

    def fcu_state(self, msg):
        self.state = msg
        if msg.armed and self.flight_stage == 'WAIT_EXTERNALNAV':
            self.flight_stage = 'MISSION'  # Never seed an airborne/moving launch bubble.

    def flight_paths(self, msg):
        data = json.loads(msg.data)
        self.planned = data['planned']
        self.controller.exploration_points = data['exploration']

    def mapping_region(self):
        if self.corridor is None:
            return None
        state = self.last_status.get('mission', 'TAKEOFF')
        index = int(self.last_status.get('survey_waypoint', 0))
        if state == 'TAKEOFF':
            # Observe the first flight corridor while still on the ground and
            # ascending. After ascent the MID-360's -7 degree lower coverage
            # cannot recover all low cells below a nearby goal footprint.
            nominal = np.vstack(([*self.c['home'][:2], self.c['takeoff_altitude']],
                                 self.controller.waypoints[index:]))
        elif state == 'SURVEY':
            nominal = self.controller.waypoints[index:]
        elif state == 'EXPLORE':
            nominal = self.last_status.get('route_blockage', {}).get('selected_viewpoint', self.pose)
            nominal = [nominal]
        else:
            nominal = [[*self.c['home'][:2], self.c['takeoff_altitude']]]
        return self.corridor.region(self.pose, self.last_status, nominal, self.planned,
            self.last_integrated_cloud)

    @measured('mapping_io_update')
    def update(self):
        if self.pose is None:
            return
        self.map_pending()
        if self.grid.version == self.last_snapshot_version or not math.isfinite(self.last_integrated_cloud):
            return
        statistics = dict(global_known_voxels=len(self.history.cells),
            simulation_ray_stats=self.simulation_ray_stats,
            mapping_ray_counts=getattr(self, 'mapping_ray_counts', {}),
            mapping_roi=getattr(self.grid, 'mapping_roi_stats', {}),
            pipeline_trace=getattr(self, 'pipeline_trace', {}),
            observed_volume_fraction=len(self.history.cells)/int(np.prod(self.history.shape)),
            observed_xy_fraction=len(self.history.observed_columns)/(self.history.shape[0]*self.history.shape[1]),
            global_map_origin=self.history.lo.tolist(), global_map_shape=list(map(int, self.history.shape)),
            mapping_io_callback_wall_max_ms=dict(self.callback_timing.max_ms))
        payload = encode_window(self.grid, statistics)
        self.snapshot_pub.publish(UInt8MultiArray(data=array('B', payload)))
        self.last_snapshot_version = self.grid.version

    @measured('local_visual_handoff')
    def visualise(self):
        if 'local' in self.visual_futures and not self.visual_futures['local'].done():
            return
        stamp = self.get_clock().now().to_msg()
        grid = SimpleNamespace(res=self.grid.res, lo=self.grid.lo.copy(), state=self.grid.state.copy())
        self._visual_job('local', dict(stamp=(stamp.sec, stamp.nanosec), grid=grid, trajectory=None,
            routes=dict(nominal_pub=[], survey_pub=self.controller.waypoints,
                        exploration_pub=self.controller.exploration_points, path_pub=self.planned),
            executed=list(self.executed)))

    def flight_path_snapshot(self):
        return self.map_log.flight_paths(np.vstack((self.c['home'], self.controller.waypoints)), self.planned)

    def destroy_node(self):
        self.map_worker.shutdown(wait=True, cancel_futures=True)
        self.visual_worker.shutdown(wait=True, cancel_futures=True)
        if self.map_log:
            self.log_worker.shutdown(wait=True)
            try:
                data = self.history.snapshot(self.get_parameter('mode').value, self.now(),
                    list(self.map_log.points.values()), self.map_log.truncated, self.last_status)
                data['flight_paths'] = self.flight_path_snapshot()
                self.map_log.write(data)
            except Exception as exc:
                self.get_logger().error(f'Final 3D map log failed: {exc}')
        return super().destroy_node()


def main(args=None):
    run(MappingIONode, args)
