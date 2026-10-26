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
from sensor_msgs.msg import PointCloud2, Range, Image, CameraInfo
from rclpy.time import Time
from tf2_ros import TransformException
from sensor_msgs_py import point_cloud2
from mavros_msgs.msg import State
from visualization_msgs.msg import MarkerArray
from tf2_ros import Buffer, TransformListener
from .ros_nodes import ConfigNode, NavigationNode, run, seconds, rotation, vector
from .coverage_mapping import navigation_window, SparseHistory, initialise_rolling_mapper
from .map_log import MapLog
from .map_transport import encode_window
from .runtime import CallbackTiming, measured
from .depth_queue import DepthTransformQueue


class MappingIONode(ConfigNode):
    # Reuse tested acquisition/export routines, not flight-management startup.
    cloud = NavigationNode.cloud
    map_pending = NavigationNode.map_pending
    odom = NavigationNode.odom
    range_cb = NavigationNode.range_cb
    range_pending = NavigationNode.range_pending
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
        self.pending_ranges=deque(maxlen=12)
        self.range_tf_waits=0
        self.state = State()
        self.pending, self.executed = DepthTransformQueue(6), deque(maxlen=6000)
        self.last_map = self.last_integrated_cloud = -math.inf
        self.last_status, self.planned = {}, []
        self.last_snapshot_version = -1
        self.map_future = self.log_future = None
        self.visual_futures = {}
        context = multiprocessing.get_context('spawn')
        self.map_worker = ProcessPoolExecutor(max_workers=1, mp_context=context,
            initializer=initialise_rolling_mapper, initargs=(self.c,))
        self.visual_worker = ProcessPoolExecutor(max_workers=1, mp_context=context)
        self.route_worker=ProcessPoolExecutor(max_workers=1,mp_context=context) if self.c.get('history_route_enabled') else None
        self.route_future=None
        self.route_request_id=0
        self.last_route_submit=-math.inf
        self.route_pub=self.create_publisher(String,'/mapping/history_route',QoSProfile(depth=1)) if self.route_worker else None
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
        self.depth_frames=DepthTransformQueue(6)
        self.depth_info=None
        self.depth_submitted_stamp = -math.inf
        self.depth_sequence = 0
        self.depth_stats = dict(enabled=bool(self.c.get('depth_camera_enabled')),
                                source='simulated_rgbd_not_ZED_stereo',last_valid_stamp=None,
                                received_frames=0,submitted_frames=0,rejection='waiting_for_depth')
        if self.c.get('depth_camera_enabled'):
            self.depth_cloud_pub = self.create_publisher(PointCloud2,'/zed/depth/points',qos_profile_sensor_data)
            # Keep newest image only; old queued camera frames cannot help live navigation.
            latest_depth_qos = copy.copy(qos_profile_sensor_data)
            latest_depth_qos.depth = 1
            self.create_subscription(Image,'/zed/depth/image',self.depth_image,latest_depth_qos)
            self.create_subscription(CameraInfo,'/zed/camera_info',
                lambda msg:setattr(self,'depth_info',msg),qos_profile_sensor_data)
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
        # Poll futures promptly instead of waiting another complete LiDAR period.
        # Idle updates return immediately; snapshots still require a new version.
        self.create_timer(.02, self.update)
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

    @measured('depth_receive')
    def depth_image(self,msg):
        stamp = seconds(msg.header.stamp)
        self.depth_stats['received_frames'] += 1
        self.depth_stats['last_received_stamp'] = stamp
        if msg.header.frame_id!='zed_camera_optical' or not 0<=self.now()-stamp<self.c['depth_camera_timeout']:
            self.depth_stats['rejection'] = 'depth_frame_or_timestamp_invalid'
            return
        self.depth_frames.push(stamp,msg,self.now())
        self.depth_stats['transform_queue']=self.depth_frames.status()

    @measured('depth_projection_and_handoff')
    def depth_observation(self):
        info=self.depth_info
        if info is None:return ()
        last_tf_error=[]
        def lookup(msg):
            try:
                at=Time.from_msg(msg.header.stamp)
                return (self.buffer.lookup_transform('map',msg.header.frame_id,at),
                        self.buffer.lookup_transform('base_link',msg.header.frame_id,at))
            except TransformException as exc:
                last_tf_error.append(str(exc)[:160]);return None
        selected=self.depth_frames.select(self.now(),self.c['depth_camera_timeout'],
                                          self.depth_submitted_stamp,lookup)
        self.depth_stats['transform_queue']=self.depth_frames.status()
        if selected is None:
            if last_tf_error:self.depth_stats['rejection']='depth_stamp_TF_pending:'+last_tf_error[0]
            return ()
        msg,(tf,body_tf)=selected
        stamp=seconds(msg.header.stamp)
        if (info.header.frame_id!=msg.header.frame_id or info.width!=msg.width or info.height!=msg.height):
            self.depth_stats['rejection']='depth_intrinsics_mismatch'
            self.depth_frames.discard(stamp)
            self.depth_stats['transform_queue']=self.depth_frames.status()
            return ()
        try:
            from .depth_mapping import project_depth
            points,stats = project_depth(bytes(msg.data),msg.width,msg.height,msg.step,msg.encoding,
                msg.is_bigendian,info.k,self.c['depth_min_range'],self.c['depth_max_range'],
                self.c['depth_mapping_stride'],self.c['depth_mapping_budget'],self.depth_sequence)
            body = points@rotation(body_tf.transform.rotation).T+vector(body_tf.transform.translation)
            from .self_filter import self_mask
            keep = ~self_mask(body,self.c)
            stats['self_filtered_rays'] = int(np.count_nonzero(~keep))
            points = points[keep]
            if not len(points):
                self.depth_stats.update(stats,rejection='no_valid_depth_rays')
                self.depth_frames.discard(stamp)
                self.depth_stats['transform_queue']=self.depth_frames.status()
                return ()
            origin = vector(tf.transform.translation)
            world = points@rotation(tf.transform.rotation).T+origin
        except (ValueError,TransformException) as exc:
            self.depth_stats['rejection'] = 'depth_projection_invalid:'+str(exc)[:160]
            self.depth_frames.discard(stamp)
            self.depth_stats['transform_queue']=self.depth_frames.status()
            return ()
        self.depth_submitted_stamp = stamp
        self.depth_frames.acknowledge(stamp)
        self.depth_stats['transform_queue']=self.depth_frames.status()
        self.depth_sequence += 1
        self.depth_stats.update(stats,last_valid_stamp=stamp,rejection='',
            submitted_frames=self.depth_stats['submitted_frames']+1,
            camera_origin_map=origin.tolist(),retained_rays=int(len(points)),
            frame=msg.header.frame_id)
        self.depth_cloud_pub.publish(point_cloud2.create_cloud_xyz32(msg.header,points.tolist()))
        if self.map_log:
            self.map_log.add_hits(world)
        return ((origin,world,stamp,self.mapping_region(world)),)

    def flight_paths(self, msg):
        data = json.loads(msg.data)
        self.planned = data['planned']
        self.controller.exploration_points = data['exploration']

    def mapping_region(self, hit_points=None):
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
        if self.c.get('active_tree_inspection'):
            hint = self.last_status.get('inspection_goal_hint')
            if hint is not None:
                nominal = [[*self.c['home'][:2],self.c['takeoff_altitude']],hint] if state=='TAKEOFF' else [hint]
        return self.corridor.region(self.pose, self.last_status, nominal, self.planned,
            self.last_integrated_cloud, hit_points=hit_points)

    @measured('mapping_io_update')
    def update(self):
        NavigationNode.range_pending(self)
        if self.pose is None:
            return
        self.map_pending(collect_only=True)
        # Deliver the completed safety map BEFORE projection/preparation of
        # another job or copying global history for route planning.
        self.publish_snapshot()
        self.map_pending()
        if getattr(self,'route_worker',None) is not None:self.update_history_route()

    def publish_snapshot(self):
        if self.grid.version == self.last_snapshot_version or not math.isfinite(self.last_integrated_cloud):
            return
        statistics = dict(global_known_voxels=len(self.history.cells),
            mapping_poll_period_sim_seconds=.02,
            mapping_handoff_policy='completed_snapshot_before_next_job_preparation',
            mapping_lidar_transform_queue=self.pending.status(),
            depth_mapping=dict(self.depth_stats,
                last_integrated_stamp=getattr(self.grid,'depth_integrated_stamp',None),
                current_job=(getattr(self,'depth_job_trace',{}) if self.map_future is not None else None),
                last_submitted_job=getattr(self,'depth_job_trace',{}),
                last_completed_job=getattr(self,'last_completed_depth_job',{}),
                roi=getattr(self.grid,'depth_roi_stats',[])),
            mapping_worker_stages=getattr(self.grid,'mapping_worker_stages',{}),
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

    def update_history_route(self):
        if self.route_worker is None:return
        request=self.last_status.get('history_route_request')
        if self.route_future is not None:
            if not self.route_future.done():return
            try:
                route=self.route_future.result()
            except Exception as exc:
                self.get_logger().error(f'History route worker failed: {exc}')
                route=None
            self.route_future=None
            if route is not None and request is not None and np.linalg.norm(np.asarray(route['goal'])-request['goal'])<1e-4:
                self.route_pub.publish(String(data=json.dumps(route,allow_nan=False)))
        if request is None or not math.isfinite(self.last_integrated_cloud) or self.now()-self.last_route_submit<2.:return
        from .global_route import plan_history_route
        history=copy.copy(self.history)
        history.cells=self.history.cells.copy()
        history.prior=self.history.prior.copy()
        history.assumptions=self.history.assumptions.copy()
        history.occupied_keys=set()
        history.observed_columns=set()
        history.last_observation_stamp=self.last_integrated_cloud
        self.route_request_id+=1
        request=dict(request,request_id=self.route_request_id)
        self.route_future=self.route_worker.submit(plan_history_route,history,request,self.now())
        self.last_route_submit=self.now()


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
        if self.route_worker:self.route_worker.shutdown(wait=True,cancel_futures=True)
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
