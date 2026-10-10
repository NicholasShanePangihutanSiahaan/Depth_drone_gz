"""Direct MAVLink/Gazebo baseline diagnostic. This does NOT validate ROS launch.

Simulation truth is the ExternalNav source. Arm/takeoff only with --fly.
All messages are restricted to the specified loopback UDP port.
"""
import os
os.environ['MAVLINK20'] = '1'
os.environ['GZ_PARTITION'] = 'polinasi_lidar'
import argparse
import json
import math
import threading
import time
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from gz.transport13 import Node
from gz.msgs10.odometry_pb2 import Odometry
from gz.msgs10.laserscan_pb2 import LaserScan
from gz.msgs10.imu_pb2 import IMU
from pymavlink import mavutil


def vec(message):
    return np.array([message.x, message.y, message.z])


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--fly', action='store_true')
    parser.add_argument('--seconds', type=float, default=30.)
    parser.add_argument('--output', type=Path, default=Path('reports/sitl_smoke.json'))
    args = parser.parse_args()
    lock = threading.Lock()
    latest = {'odom': None, 'cloud_count': 0, 'imu_count': 0, 'scan': None}
    def odom(msg):
        with lock:
            latest['odom'] = msg
    def cloud(msg):
        with lock:
            latest['cloud_count'] += 1
            latest['scan'] = msg
    def imu(msg):
        with lock:
            latest['imu_count'] += 1
    node = Node()
    node.subscribe(Odometry, '/simulation/ground_truth/odom', odom)
    node.subscribe(LaserScan, '/livox/lidar', cloud)
    node.subscribe(IMU, '/livox/imu', imu)
    link = mavutil.mavlink_connection('udpin:127.0.0.1:14550', source_system=250, source_component=191)
    link.wait_heartbeat(timeout=15)
    if link.target_system != 1:
        raise RuntimeError('expected isolated SITL system 1')
    for message_id in (32, 193, 33):
        link.mav.command_long_send(1, 1, mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0, message_id, 100000, 0, 0, 0, 0, 0)
    start = time.monotonic()
    next_origin, next_odom, last_odom_stamp = 0., 0., -1
    status_text, acknowledgements, parameters = [], [], {}
    local, ekf, armed = None, None, False
    stage, sent, max_z = 'WAIT_EKF', set(), 0.
    basis = np.array([[0., 1., 0.], [1., 0., 0.], [0., 0., -1.]])
    frd = np.diag([1., -1., -1.])
    covariance = [0.]*21
    for index in (0, 6, 11, 15, 18, 20):
        covariance[index] = .01
    checks = ['GPS1_TYPE', 'GPS2_TYPE', 'SIM_GPS_DISABLE', 'SIM_GPS2_DISABLE',
              'EK3_SRC1_POSXY', 'EK3_SRC1_POSZ', 'EK3_SRC1_VELXY', 'EK3_SRC1_VELZ', 'EK3_SRC1_YAW']
    for key in checks:
        link.mav.param_request_read_send(1, 1, key.encode(), -1)
    try:
        while time.monotonic()-start < args.seconds:
            elapsed = time.monotonic()-start
            with lock:
                truth = latest['odom']
            if elapsed >= next_origin:
                link.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER, mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
                link.mav.set_gps_global_origin_send(1, int(-35.363261*1e7), int(149.165230*1e7), 584000)
                next_origin = elapsed+1.
                if args.fly:
                    # Virtual RC: centred axes and low throttle, no forced arm.
                    link.mav.rc_channels_override_send(1, 1, 1500, 1500, 1000, 1500, 1500, 1500, 1500, 1500)
            if truth is not None and elapsed >= next_odom:
                stamp = truth.header.stamp.sec*1000000+truth.header.stamp.nsec//1000
                if stamp > last_odom_stamp:
                    p = basis@vec(truth.pose.position)
                    q = truth.pose.orientation
                    r = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
                    xyzw = Rotation.from_matrix(basis@r@frd).as_quat()
                    v, omega = frd@vec(truth.twist.linear), frd@vec(truth.twist.angular)
                    link.mav.odometry_send(stamp, mavutil.mavlink.MAV_FRAME_LOCAL_FRD,
                            mavutil.mavlink.MAV_FRAME_BODY_FRD, *p, [xyzw[3], *xyzw[:3]],
                            *v, *omega, covariance, covariance, 0, mavutil.mavlink.MAV_ESTIMATOR_TYPE_VISION, 100)
                    last_odom_stamp = stamp
                next_odom = elapsed+.05
                max_z = max(max_z, truth.pose.position.z)
            for _ in range(50):
                msg = link.recv_match(blocking=False)
                if msg is None:
                    break
                typ = msg.get_type()
                if typ == 'STATUSTEXT':
                    status_text.append(msg.text)
                    print(msg.text, flush=True)
                elif typ == 'COMMAND_ACK':
                    acknowledgements.append({'command': msg.command, 'result': msg.result})
                elif typ == 'LOCAL_POSITION_NED':
                    local = [msg.x, msg.y, msg.z]
                elif typ == 'EKF_STATUS_REPORT':
                    ekf = msg.flags
                elif typ == 'HEARTBEAT':
                    armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                elif typ == 'PARAM_VALUE':
                    parameters[msg.param_id] = msg.param_value
            ready = ekf is not None and bool(ekf & 16) and local is not None
            if args.fly and ready and elapsed > 12. and 'mode' not in sent:
                link.set_mode('GUIDED')
                sent.add('mode'); stage = 'REQUEST_GUIDED'
            if args.fly and 'mode' in sent and elapsed > 15. and 'arm' not in sent:
                link.mav.command_long_send(1, 1, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)
                sent.add('arm'); stage = 'REQUEST_ARM'
            if args.fly and armed and 'takeoff' not in sent:
                link.mav.command_long_send(1, 1, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, 2.)
                sent.add('takeoff'); stage = 'TAKEOFF'
            if args.fly and 'takeoff' in sent and 'land' not in sent and truth is not None and truth.pose.position.z > 1.8:
                stage = 'HOVER'
            if args.fly and elapsed > args.seconds-20. and armed and 'land' not in sent:
                link.mav.command_long_send(1, 1, mavutil.mavlink.MAV_CMD_NAV_LAND, 0, 0, 0, 0, 0, 0, 0, 0)
                sent.add('land'); stage = 'LAND'
            time.sleep(.01)
    finally:
        if args.fly and armed and 'land' not in sent:
            link.mav.command_long_send(1, 1, mavutil.mavlink.MAV_CMD_NAV_LAND, 0, 0, 0, 0, 0, 0, 0, 0)
        link.close()
    with lock:
        scan, truth = latest['scan'], latest['odom']
        cloud_count, imu_count = latest['cloud_count'], latest['imu_count']
    sensor = None if scan is None else {'horizontal_samples': scan.count, 'vertical_samples': scan.vertical_count,
                'horizontal_min_deg': math.degrees(scan.angle_min), 'horizontal_max_deg': math.degrees(scan.angle_max),
                'vertical_min_deg': math.degrees(scan.vertical_angle_min), 'vertical_max_deg': math.degrees(scan.vertical_angle_max),
                'frame': scan.frame, 'range_min': scan.range_min, 'range_max': scan.range_max}
    gps_disabled = all(parameters.get(k) == 0 for k in checks[:2]) and all(parameters.get(k) == 1 for k in checks[2:4])
    sources_external = all(parameters.get(k) == 6 for k in checks[4:])
    ready = ekf is not None and bool(ekf & 16) and local is not None
    result = {'localisation_mode': 'GROUND_TRUTH', 'transport': 'direct MAVLink; ROS stack NOT tested',
              'gps_disabled': gps_disabled, 'ekf_sources_externalnav': sources_external,
              'parameters': parameters, 'ekf_flags': ekf, 'local_position_ned': local,
              'simulator_max_altitude_m': max_z, 'stage': stage, 'armed_at_end': armed,
              'takeoff_requested': 'takeoff' in sent, 'land_requested': 'land' in sent,
              'touchdown_validated': 'land' in sent and not armed and truth is not None and truth.pose.position.z < .3,
              'externalnav_ready': ready, 'lidar_messages': cloud_count, 'imu_messages': imu_count,
              'sensor': sensor, 'acknowledgements': acknowledgements, 'status_text': status_text}
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2)+'\n')
    print(json.dumps(result, indent=2))
    ok = gps_disabled and sources_external and ready and sensor is not None
    if args.fly:
        ok = ok and result['touchdown_validated'] and max_z > 1.8
    raise SystemExit(0 if ok else 1)


if __name__ == '__main__':
    main()
