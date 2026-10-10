"""Read-only ROS/Gazebo setup check. Does not arm or send flight commands."""
import argparse
import json
import math
import time
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import PointCloud2, Imu, Range, JointState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool, String
from tf2_ros import Buffer, TransformListener, TransformException
from rclpy.time import Time
from scipy.spatial.transform import Rotation


def stamp_seconds(stamp):
    return stamp.sec+stamp.nanosec*1e-9


class Probe(Node):
    def __init__(self):
        super().__init__('polinasi_setup_check', parameter_overrides=[Parameter('use_sim_time', value=True)])
        self.counts, self.last, self.frames = {}, {}, {}
        self.healthy, self.status = False, {}
        self.poses = {}
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        types = {'/clock': Clock, '/livox/lidar': PointCloud2, '/livox/imu': Imu,
                 '/localisation/odometry': Odometry, '/mavros/odometry/out': Odometry,
                 '/mavros/local_position/pose': PoseStamped,
                 '/simulation/rangefinder': Range, '/camera/joint_state': JointState}
        for topic, message_type in types.items():
            self.create_subscription(message_type, topic, lambda msg, t=topic: self.receive(t, msg), qos_profile_sensor_data)
        self.create_subscription(Bool, '/localisation/healthy', self.health, 10)
        self.create_subscription(String, '/navigation/status', self.navigation, 10)

    def receive(self, topic, msg):
        self.counts[topic] = self.counts.get(topic, 0)+1
        if hasattr(msg, 'header'):
            self.last[topic] = stamp_seconds(msg.header.stamp)
            self.frames[topic] = msg.header.frame_id
        if topic in ('/localisation/odometry', '/mavros/local_position/pose'):
            pose = msg.pose.pose if isinstance(msg, Odometry) else msg.pose
            q = pose.orientation
            yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
            self.poses[topic] = ([pose.position.x, pose.position.y, pose.position.z], yaw)

    def health(self, msg):
        self.healthy = msg.data

    def navigation(self, msg):
        self.status = json.loads(msg.data)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--seconds', type=float, default=15.)
    parser.add_argument('--output', type=Path, default=Path('reports/ros_setup.json'))
    args = parser.parse_args()
    rclpy.init()
    node = Probe()
    started = time.monotonic()
    while time.monotonic()-started < args.seconds:
        rclpy.spin_once(node, timeout_sec=0.1)
    ownership = {}
    final_interface = node.status.get('setpoint_interface', '/mavros/setpoint_position/local')
    inactive_interface = '/mavros/setpoint_position/local' if final_interface == '/mavros/setpoint_raw/local' else '/mavros/setpoint_raw/local'
    for topic in (final_interface, '/control/safe_target_pose',
                  '/localisation/odometry', '/mavros/odometry/out', '/livox/lidar'):
        ownership[topic] = {'publishers': node.count_publishers(topic), 'subscribers': node.count_subscribers(topic)}
    now = node.get_clock().now().nanoseconds*1e-9
    expected_frames = {'/livox/lidar': 'mid360', '/livox/imu': 'mid360_imu',
                       '/localisation/odometry': 'odom', '/mavros/local_position/pose': 'map'}
    empty_identification = (node.status.get('mission_kind') == 'identification'
                            and node.status.get('identification_empty_arena') is True)
    if empty_identification:
        expected_frames.pop('/livox/lidar')
        ownership.pop('/livox/lidar')
    ages = {topic: now-stamp for topic, stamp in node.last.items()}
    names = node.get_node_names()
    localisation_inputs = ([topic for topic, _ in node.get_subscriber_names_and_types_by_node('polinasi_localisation', '/')]
                           if 'polinasi_localisation' in names else [])
    mode = node.status.get('mode')
    sources_exclusive = (
        mode == 'sensor' and '/simulation/ground_truth/odom' not in localisation_inputs
        and all(topic in localisation_inputs for topic in ('/livox/lidar', '/livox/imu'))
    ) or (
        mode == 'ground_truth' and '/simulation/ground_truth/odom' in localisation_inputs
        and all(topic not in localisation_inputs for topic in ('/livox/lidar', '/livox/imu'))
    )
    alignment = {}
    if node.status.get('mission_kind') in ('mapping', 'identification'):
        expected_frames['/mavros/local_position/pose'] = 'fcu_local'
    if len(node.poses) == 2:
        estimated, fcu = (node.poses[t] for t in ('/localisation/odometry', '/mavros/local_position/pose'))
        if node.frames.get('/mavros/local_position/pose') == 'fcu_local':
            try:
                transform = node.tf_buffer.lookup_transform('map', 'fcu_local', Time())
                q, p = transform.transform.rotation, transform.transform.translation
                rotation = Rotation.from_quat([q.x, q.y, q.z, q.w])
                position = rotation.apply(fcu[0])+[p.x, p.y, p.z]
                yaw = fcu[1]+rotation.as_euler('xyz')[2]
                fcu = (position.tolist(), yaw)
            except TransformException:
                fcu = ([float('inf')]*3, fcu[1])
        alignment = {'position_difference_m': math.dist(estimated[0], fcu[0]),
                     'yaw_difference_rad': abs(math.atan2(math.sin(estimated[1]-fcu[1]), math.cos(estimated[1]-fcu[1])))}
    checks = {
        'clock_running': node.counts.get('/clock', 0) > 5 and now > 0,
        'cloud_received': node.counts.get('/livox/lidar', 0) > 5,
        'imu_received': node.counts.get('/livox/imu', 0) > 20,
        'localisation_healthy': node.healthy,
        'localisation_sources_exclusive': sources_exclusive,
        'frames_match': all(node.frames.get(t) == f for t, f in expected_frames.items()),
        'measurements_fresh': all(-0.05 <= ages.get(topic, float('inf')) <= 1.
                                  for topic in expected_frames),
        'one_producer_per_control_interface': all(info['publishers'] == 1 for info in ownership.values()),
        'inactive_control_interface_unused': node.count_publishers(inactive_interface) == 0,
        'final_control_has_receiver': ownership[final_interface]['subscribers'] >= 1,
        'mavros_receives_externalnav': ownership['/mavros/odometry/out']['subscribers'] >= 2 and node.counts.get('/mavros/odometry/out', 0) > 5,
        'fcu_pose_received': node.counts.get('/mavros/local_position/pose', 0) > 5,
        'fcu_externalnav_aligned': (alignment.get('position_difference_m', float('inf')) < 0.25
                                   and alignment.get('yaw_difference_rad', float('inf')) < 0.2),
        'range_received': node.counts.get('/simulation/rangefinder', 0) > 5,
        'camera_feedback_received': node.counts.get('/camera/joint_state', 0) > 5,
        'navigation_running': bool(node.status),
    }
    if node.status.get('runtime_isolation') == 'mapping_io_process' and not empty_identification:
        checks['mapping_io_running'] = 'polinasi_mapping_io' in names
        checks['one_navigation_snapshot_producer'] = node.count_publishers('/mapping/navigation_snapshot') == 1
        age = node.status.get('map_measurement_age')
        checks['navigation_snapshot_fresh'] = age is not None and 0 <= age < 1.
    if empty_identification:
        for key in ('cloud_received', 'range_received', 'camera_feedback_received'):
            checks.pop(key)
        checks['lidar_disabled'] = all(node.count_publishers(topic) == 0
            for topic in ('/livox/lidar', '/simulation/lidar', '/mapping/navigation_snapshot'))
        checks['mapping_disabled'] = 'polinasi_mapping_io' not in names
        checks['known_empty_arena'] = node.status.get('map_source') == 'known_empty_simulation_arena_not_lidar'
    report = {'setup_checks': checks, 'passed': all(checks.values()),
              'message_counts': node.counts, 'frames': node.frames,
              'measurement_age_seconds': ages,
              'ownership': ownership, 'navigation': node.status,
              'fcu_externalnav_alignment': alignment,
              'localisation_subscriptions': localisation_inputs,
              'flight_test': False, 'commands_sent': False}
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps(report, indent=2))
    node.destroy_node()
    rclpy.shutdown()
    raise SystemExit(0 if report['passed'] else 1)


if __name__ == '__main__':
    main()
