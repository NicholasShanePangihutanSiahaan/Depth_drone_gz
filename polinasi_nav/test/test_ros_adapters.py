"""ROS adapter regressions; skipped for the standalone non-ROS test environment."""
import numpy as np
import pytest
import json
from unittest.mock import Mock
from types import SimpleNamespace

rclpy = pytest.importorskip('rclpy')
pytest.importorskip('mavros_msgs')
from std_msgs.msg import Header
from sensor_msgs_py import point_cloud2
from sensor_msgs.msg import Imu, PointField
from builtin_interfaces.msg import Time
from polinasi_nav.ros_nodes import (
    LocalisationNode, ExternalNavNode, NavigationNode, RangeBridge, SensorGate,
    cloud_points,
)


@pytest.fixture(scope='module', autouse=True)
def ros_context():
    rclpy.init(args=['--ros-args', '-p', 'use_sim_time:=true'])
    yield
    rclpy.shutdown()


def test_held_reference_does_not_recycle_pid_acceleration_and_braking_is_preserved():
    controller = SimpleNamespace(failure='',trajectory=None,command_v=np.zeros(3),
        command_a=np.array([.3,-.2,.1]),tick=Mock(return_value=(np.zeros(3),np.zeros(3))))
    pid = Mock()
    fake = SimpleNamespace(controller=controller,predictive=None,pid=pid,pose=np.zeros(3),
        velocity=np.zeros(3),range_value=None,range_stamp=None)
    NavigationNode.control_step(fake,1.,.05)
    np.testing.assert_allclose(controller.command_a,0.)
    controller.failure = 'test_braking'
    controller.command_a = np.array([.2,0.,0.])
    NavigationNode.control_step(fake,1.05,.05)
    np.testing.assert_allclose(controller.command_a,[.2,0.,0.])


def test_range_waits_for_measurement_time_tf_without_restamping():
    from collections import deque
    from sensor_msgs.msg import Range
    from geometry_msgs.msg import TransformStamped
    from tf2_ros import TransformException
    fake=SimpleNamespace(pending_ranges=deque(maxlen=12),range_tf_waits=0,
        range_value=None,range_stamp=-np.inf,c={'range_timeout':.3,'rangefinder_mount':[0,0,-.07,0,0,0]},
        now=lambda:1.1,buffer=Mock())
    fake.range_pending=lambda:NavigationNode.range_pending(fake)
    fake.buffer.lookup_transform.side_effect=TransformException('future TF not available yet')
    msg=Range();msg.header.stamp.sec=1;msg.min_range=.05;msg.max_range=8.;msg.range=.13
    NavigationNode.range_cb(fake,msg)
    assert fake.range_value is None and len(fake.pending_ranges)==1
    transform=TransformStamped();transform.transform.rotation.w=1.
    fake.buffer.lookup_transform.side_effect=None;fake.buffer.lookup_transform.return_value=transform
    NavigationNode.range_pending(fake)
    assert fake.range_stamp==1. and fake.range_value==pytest.approx(.2)
    assert len(fake.pending_ranges)==0
    # An old queued range never becomes a fresh measurement after TF arrives.
    fake.now=lambda:2.
    fake.pending_ranges.append(msg)
    NavigationNode.range_pending(fake)
    assert fake.range_stamp==1. and len(fake.pending_ranges)==0


def test_navigation_initialises_and_publishes_visualisation():
    node = NavigationNode()
    try:
        # Regression: these methods accidentally belonged to SensorGate,
        # causing NavigationNode startup to fail at create_timer().
        node.visualise()
        path = node.path_message([[1., 2., 3.]])
        assert path.header.frame_id == 'map'
        assert path.poses[0].pose.position.z == 3.
        assert node.final_pub.topic_name == '/mavros/setpoint_position/local'
    finally:
        node.destroy_node()


def test_mapping_fast_poll_does_not_publish_unchanged_map():
    from polinasi_nav.mapping_io import MappingIONode
    fake = SimpleNamespace(pose=np.zeros(3),map_pending=Mock(),
        grid=SimpleNamespace(version=7),last_snapshot_version=7,
        last_integrated_cloud=1.,snapshot_pub=Mock())
    MappingIONode.update(fake)
    fake.map_pending.assert_called_once()
    fake.snapshot_pub.publish.assert_not_called()


def test_mapping_io_owns_shared_range_queue_and_retry(monkeypatch):
    from polinasi_nav.mapping_io import MappingIONode
    from polinasi_nav.ros_nodes import ConfigNode
    from sensor_msgs.msg import Range
    from tf2_ros import TransformException
    original_init=ConfigNode.__init__
    def mapping_config(self,*args,**kwargs):
        original_init(self,*args,**kwargs)
        self.c.update(navigation_window_size=[10.,10.,8.],survey_waypoints=[[0.,0.,2.]])
    monkeypatch.setattr(ConfigNode,'__init__',mapping_config)
    node=MappingIONode()
    try:
        node.buffer=Mock()
        node.buffer.lookup_transform.side_effect=TransformException('waiting for localisation TF')
        node.range_cb(Range(min_range=.05,max_range=8.,range=.13))
        assert len(node.pending_ranges)==1
        assert node.range_tf_waits==1
        node.range_pending()
        assert node.range_tf_waits==2
    finally:
        node.destroy_node()


def test_sensor_auditor_only_allows_bounded_existing_map_brake_recovery():
    import importlib.util
    from pathlib import Path
    path=Path(__file__).resolve().parents[2]/'tools/audit_sensor_localisation.py'
    spec=importlib.util.spec_from_file_location('sensor_audit',path)
    import sys
    sys.path.insert(0,str(path.parent))
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    w=module.RecoveryWatch()
    nav=dict(failure='route_changed',mission='BRAKE',simulation_time=1.,health_reason='')
    assert w.check(nav,10.)==''
    assert w.check(dict(nav,simulation_time=2.),11.)==''
    assert w.check(dict(nav,failure='',mission='SURVEY',simulation_time=3.),12.)==''
    assert len(w.events)==2
    assert w.check(dict(nav,failure='localisation_loss'),13.)=='localisation_loss'
    assert w.check(dict(nav,failure='emergency_unavoidable:route_changed'),14.).startswith('emergency_unavoidable')
    assert w.check(dict(nav,mission='HOLD_ABORT'),15.)=='route_changed'
    assert w.check(dict(nav,simulation_time=6.),16.)==''
    assert w.check(dict(nav,simulation_time=12.),22.)=='map_change_recovery_timeout'


def test_simulation_ray_gate_preserves_raw_cloud_stamp_and_dropout():
    from polinasi_nav.simulated_rays import directions
    node = SensorGate()
    try:
        node.c['simulation_no_return_rays'] = True
        node.pub, node.rays_pub, node.ray_status_pub = Mock(), Mock(), Mock()
        header = Header(frame_id='mid360', stamp=Time(sec=7, nanosec=123))
        xyz = directions()*5.
        xyz[100] = np.copysign(np.inf, directions()[100])
        fields = [PointField(name=n, offset=i*4, datatype=PointField.FLOAT32, count=1)
                  for i, n in enumerate(('x', 'y', 'z'))]
        fields.append(PointField(name='ring', offset=12, datatype=PointField.UINT16, count=1))
        rows = [(*p, int(r)) for p, r in zip(xyz, np.repeat(np.arange(32), 360))]
        msg = point_cloud2.create_cloud(header, fields, rows)
        msg.width, msg.height, msg.row_step = 360, 32, 360*msg.point_step
        node.cloud(msg)
        assert node.pub.publish.call_args.args[0] is msg
        decoded = node.rays_pub.publish.call_args.args[0]
        assert decoded.header == header
        assert json.loads(node.ray_status_pub.publish.call_args.args[0].data)['explicit_no_return'] == 1
        assert len(cloud_points(decoded)) == 11520
        # Invalid dimensions fail closed: no fabricated max-range rays.
        msg.height = 1
        node.cloud(msg)
        assert node.rays_pub.publish.call_args.args[0] is msg
        assert 'error' in json.loads(node.ray_status_pub.publish.call_args.args[0].data)
        node.set_parameters([rclpy.parameter.Parameter('drop_after', value=1.)])
        node.started_at = -2.
        node.pub.publish.reset_mock()
        node.rays_pub.publish.reset_mock()
        node.cloud(msg)
        node.pub.publish.assert_not_called()
        node.rays_pub.publish.assert_not_called()
    finally:
        node.destroy_node()


def test_mapping_roi_observes_first_corridor_before_takeoff():
    from types import SimpleNamespace
    from pathlib import Path
    from polinasi_nav.config import load_config
    from polinasi_nav.corridor_mapping import CorridorRegion, contains
    from polinasi_nav.mapping_io import MappingIONode
    c = load_config(Path(__file__).resolve().parents[1]/'config/predictive_tour_check.json')
    fake = SimpleNamespace(c=c, corridor=CorridorRegion(c), pose=np.array([0., 0., .2]),
        last_status={'mission': 'TAKEOFF', 'survey_waypoint': 0}, planned=[],
        last_integrated_cloud=1., controller=SimpleNamespace(waypoints=np.asarray(c['survey_waypoints'])))
    roi = MappingIONode.mapping_region(fake)
    # Low cells below the future footprint must be retained during ascent,
    # before the LiDAR's narrow downward coverage makes them difficult to see.
    assert contains([[-.9, -2.3, 1.1], [0., 0., 2.9]], roi).all()


def test_identification_partial_log_write_uses_cached_graph_after_shutdown(tmp_path):
    from types import SimpleNamespace
    from collections import deque
    from polinasi_nav.identification_recorder import IdentificationRecorder
    recorder = SimpleNamespace(status={}, armed_seen=False, land_seen=False,
        truncated=False, c={'identification_empty_arena': True},
        commands=[], responses=[], imu_samples=[], events=[], models=deque(),
        directory=tmp_path, single_producer_seen=True, producer_violation=False,
        get_logger=Mock(return_value=Mock()),
        count_publishers=Mock(side_effect=RuntimeError('ROS context is invalid')))
    IdentificationRecorder.write(recorder)
    report = json.loads((tmp_path/'identification.json').read_text())
    assert recorder.written and not report['completed']
    assert not report['lidar_enabled'] and report['one_final_publisher']
    recorder.count_publishers.assert_not_called()


def test_empty_arena_landing_confirmation_is_not_a_rangefinder_bypass_for_mapping():
    from types import SimpleNamespace
    node = SimpleNamespace(identification_empty_arena=True, pose=np.array([0., 0., .2]),
        state=SimpleNamespace(mode='LAND'), velocity=np.zeros(3),
        c=dict(initial_pose=[0., 0., .195], pose_timeout=.3, range_timeout=.3),
        controller=SimpleNamespace(health=SimpleNamespace(pose_stamp=1.)),
        range_value=None, range_stamp=-float('inf'))
    assert NavigationNode.landing_confirmed(node, 1.1)
    assert not NavigationNode.landing_confirmed(node, 2.)
    node.pose[2] = 2.4
    assert not NavigationNode.landing_confirmed(node, 1.1)
    node.pose[2] = .2
    node.identification_empty_arena = False
    assert not NavigationNode.landing_confirmed(node, 1.1)


def test_logged_executed_path_uses_numerical_measurement_time_map_transform(tmp_path):
    from nav_msgs.msg import Odometry
    from geometry_msgs.msg import TransformStamped
    from polinasi_nav.map_log import MapLog
    node = NavigationNode()
    try:
        node.map_log = MapLog(tmp_path)
        transform = TransformStamped()
        transform.transform.translation.x = 10.
        transform.transform.rotation.z = np.sin(np.pi/4)
        transform.transform.rotation.w = np.cos(np.pi/4)
        node.buffer.lookup_transform = Mock(return_value=transform)
        msg = Odometry(header=Header(frame_id='odom', stamp=Time(sec=1)), child_frame_id='base_link')
        msg.pose.pose.position.x, msg.pose.pose.position.z = 1., 2.
        msg.pose.pose.orientation.w = 1.
        node.odom(msg)
        np.testing.assert_allclose(node.map_log.executed_path, [[10., 1., 2.]], atol=1e-9)
        np.testing.assert_allclose(node.pose, [1., 0., 2.])  # Logging never modifies control.
        assert node.map_log.executed_times == [1.]
        assert node.flight_path_snapshot()['frame'] == 'map'
        args = node.buffer.lookup_transform.call_args.args
        assert args[:2] == ('map', 'odom') and args[2].nanoseconds == 1000000000
    finally:
        node.map_log = None  # Node was created without an export worker.
        node.destroy_node()


@pytest.mark.parametrize('node_type', [LocalisationNode, ExternalNavNode, RangeBridge, SensorGate])
def test_other_adapters_initialise(node_type):
    node = node_type()
    node.destroy_node()


def test_real_pointcloud2_decode():
    points = [(1., 2., 3.), (2., 3., 4.)]
    cloud = point_cloud2.create_cloud_xyz32(Header(frame_id='mid360'), points+[(float('inf'), 0., 0.), (float('nan'), 1., 1.)])
    np.testing.assert_allclose(cloud_points(cloud), points)


def test_rejected_scan_keeps_only_fresh_previous_pose_and_never_extends_timeout():
    from types import SimpleNamespace
    from unittest.mock import Mock
    from polinasi_nav.localisation import LidarImuOdometry
    e=LidarImuOdometry([0,0,0,0,0,0],body_imu=True)
    e.ok=False;e.last_good=1.9;e.reason='registration_rejected'
    fake=SimpleNamespace(now=lambda:2.,last_stamp=1.9,last_imu=1.99,
        c={'pose_timeout':.3},mode='sensor',estimator=e,status=Mock(),
        localisation_diagnostics=Mock(),rolling_sensor=True,mount_rejection='')
    LocalisationNode.health(fake)
    assert fake.status.publish.call_args.args[0].data
    fake.now=lambda:2.31
    LocalisationNode.health(fake)
    assert not fake.status.publish.call_args.args[0].data


def test_imu_prediction_never_advances_past_scan_timestamp():
    node = LocalisationNode()
    try:
        node.now = lambda: 1.
        for nanosecond in (980000000, 990000000, 1000000000):
            msg = Imu(header=Header(frame_id='mid360_imu', stamp=Time(sec=nanosecond//1000000000, nanosec=nanosecond%1000000000)))
            msg.linear_acceleration.z = 9.81
            node.imu(msg)
        cloud = point_cloud2.create_cloud_xyz32(Header(frame_id='mid360', stamp=Time(nanosec=990000000)), [(2., 0., 0.)])
        node.scan(cloud)
        assert node.estimator.last_imu == pytest.approx(0.99)
        assert node.pending_imu[0][0] == 1.
    finally:
        node.destroy_node()


def test_startup_without_pose_publishes_wait_status_and_buzzer_off():
    node = NavigationNode()
    try:
        node.now = lambda: 1.
        node.last_tick = 0.95
        node.status_pub, node.buzzer_pub = Mock(), Mock()
        node.tick()
        status = json.loads(node.status_pub.publish.call_args.args[0].data)
        assert status['health_reason'] == 'waiting_for_localisation'
        assert status['flight'] == 'WAIT_EXTERNALNAV'
        assert not node.buzzer_pub.publish.call_args.args[0].data
    finally:
        node.destroy_node()


def test_sparse_fresh_cloud_is_not_dropout_or_free_space_evidence():
    node = NavigationNode()
    try:
        node.now = lambda: 1.
        before = node.grid.state.copy()
        cloud = point_cloud2.create_cloud_xyz32(Header(frame_id='mid360', stamp=Time(nanosec=990000000)), [(2., 0., 0.)])
        node.cloud(cloud)
        assert node.controller.health.cloud_stamp == pytest.approx(.99)
        assert len(node.pending) == 1
        np.testing.assert_array_equal(node.grid.state, before)
        old = point_cloud2.create_cloud_xyz32(Header(frame_id='mid360', stamp=Time(nanosec=100000000)), [(2., 0., 0.)])
        node.cloud(old)
        assert len(node.pending) == 1
    finally:
        node.destroy_node()


def test_voxel_visualisation_uses_fine_cell_size_and_bounded_markers():
    node = NavigationNode()
    try:
        node.map_pub = Mock()
        node.visualise()
        markers = node.map_pub.publish.call_args.args[0].markers
        assert len(markers) == 3
        for marker in markers:
            assert marker.scale.x == pytest.approx(.2)
            assert marker.scale.y == pytest.approx(.2)
            assert marker.scale.z == pytest.approx(.2)
            assert len(marker.points) <= 6000
    finally:
        node.destroy_node()


def test_duplicate_sim_timestamp_does_not_advance_or_abort():
    node = NavigationNode()
    try:
        node.now = lambda: 1.
        node.last_tick = 1.
        node.flight_stage = 'MISSION'
        node.controller.fail = Mock()
        node.final_pub = Mock()
        node.tick()
        node.controller.fail.assert_not_called()
        node.final_pub.publish.assert_not_called()
    finally:
        node.destroy_node()


def test_real_timing_gap_is_still_rejected_and_reported():
    node = NavigationNode()
    try:
        node.now = lambda: 1.
        node.last_tick = .5
        node.flight_stage = 'MISSION'
        node.status_pub = Mock()
        node.tick()
        assert node.controller.failure == 'clock_or_scheduler_gap'
        assert node.last_status['last_timing_gap_seconds'] == .5
    finally:
        node.destroy_node()



def test_sensor_auditor_depth_recovery_is_bounded_and_never_masks_sensor_loss():
    import importlib.util
    from pathlib import Path
    path=Path(__file__).resolve().parents[2]/'tools/audit_sensor_localisation.py'
    spec=importlib.util.spec_from_file_location('sensor_depth_audit',path)
    import sys
    sys.path.insert(0,str(path.parent))
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    nav=dict(failure='depth_recovering',mission='BRAKE',simulation_time=33.,health_reason='',
        depth_motion_recovery=dict(action='certified_brake_started',started=33.,valid_age=.65,hold_timeout_seconds=2.),
        depth_safety=dict(timeout_seconds=1.))
    watch=module.RecoveryWatch()
    assert watch.check(nav,100.)==''
    assert watch.check(dict(nav,mission='HOLD_ABORT',simulation_time=34.),106.)==''
    assert watch.check(dict(nav,failure='',mission='SURVEY',simulation_time=34.1),107.)==''
    assert [e['event'] for e in watch.events]==['depth_brake_started','depth_recovery_ended']
    assert watch.check(dict(nav,failure='stale_lidar'),108.)=='stale_lidar'
    assert watch.check(dict(nav,failure='emergency_unavoidable:depth_recovering'),109.).startswith('emergency_unavoidable')
    assert module.RecoveryWatch().check(dict(nav,health_reason='localisation_loss'),100.)=='localisation_loss'
    bad=dict(nav,depth_motion_recovery=dict(nav['depth_motion_recovery'],valid_age=1.05))
    assert module.RecoveryWatch().check(bad,100.)=='stale_depth_camera'
    bounded=module.RecoveryWatch();assert bounded.check(nav,100.)==''
    assert bounded.check(dict(nav,simulation_time=35.05),113.)=='stale_depth_camera'
    missing=dict(nav);missing.pop('depth_motion_recovery')
    assert module.RecoveryWatch().check(missing,100.)=='depth_recovering'


def test_depth_queue_projects_newest_transformable_original_stamp_and_rejects_dropout():
    from polinasi_nav.mapping_io import MappingIONode
    from polinasi_nav.depth_queue import DepthTransformQueue
    from sensor_msgs.msg import Image,CameraInfo
    from geometry_msgs.msg import TransformStamped
    from tf2_ros import TransformException
    clock=[20.4];tf_until=[20.1];lookups=[]
    def lookup(target,source,at):
        stamp=at.nanoseconds*1e-9;lookups.append((target,stamp))
        if stamp>tf_until[0]+1e-9:raise TransformException('future transform')
        tf=TransformStamped();tf.transform.rotation.w=1.
        if target=='map':tf.transform.translation.x=stamp
        return tf
    info=CameraInfo(width=3,height=3,k=[100.,0.,1.,0.,100.,1.,0.,0.,1.])
    info.header.frame_id='zed_camera_optical'
    fake=SimpleNamespace(c=dict(depth_camera_timeout=1.,depth_min_range=.3,depth_max_range=20.,
        depth_mapping_stride=1,depth_mapping_budget=9,drone_dimensions=[.2,.2,.2],self_filter_padding=.01),
        depth_frames=DepthTransformQueue(6),depth_info=info,depth_submitted_stamp=-np.inf,
        depth_sequence=0,depth_stats=dict(received_frames=0,submitted_frames=0),
        now=lambda:clock[0],buffer=Mock(),depth_cloud_pub=Mock(),map_log=None,
        mapping_region=lambda world:None)
    fake.buffer.lookup_transform.side_effect=lookup
    def image(stamp,value=2.):
        msg=Image(width=3,height=3,encoding='32FC1',step=12)
        msg.header=Header(stamp=Time(sec=int(stamp),nanosec=round((stamp-int(stamp))*1e9)),frame_id='zed_camera_optical')
        msg.data=np.full((3,3),value,dtype='<f4').tobytes()
        return msg
    MappingIONode.depth_image(fake,image(20.))
    MappingIONode.depth_image(fake,image(20.2))
    observed=MappingIONode.depth_observation(fake)
    assert observed[0][2]==20. and observed[0][0][0]==20.
    assert any(target=='map' and abs(stamp-20.2)<1e-9 for target,stamp in lookups)
    assert ('base_link',20.) in lookups
    assert fake.depth_submitted_stamp==20. and fake.depth_frames.status()['pending']==1
    clock[0]=20.5;tf_until[0]=20.3
    assert MappingIONode.depth_observation(fake)[0][2]==20.2
    assert fake.depth_stats['last_valid_stamp']==20.2
    # An invalid latest image provides no free ray or timestamp refresh.
    clock[0]=22.1;tf_until[0]=22.1
    MappingIONode.depth_image(fake,image(22.,float('nan')))
    assert MappingIONode.depth_observation(fake)==()
    assert fake.depth_stats['last_valid_stamp']==20.2
    assert fake.depth_stats['submitted_frames']==2
    assert fake.depth_stats['rejection']=='no_valid_depth_rays'
