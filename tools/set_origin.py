"""Confirm fixed simulation EKF origin and home on SITL's loopback endpoint."""
import argparse
import math
import json
import time
from pymavlink import mavutil


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', type=int, default=5762)
    parser.add_argument('--timeout', type=float, default=30.)
    args = parser.parse_args()
    connection = mavutil.mavlink_connection(f'tcp:127.0.0.1:{args.port}', source_system=250, source_component=191)
    heartbeat = connection.wait_heartbeat(timeout=15)
    if heartbeat is None or connection.target_system != 1:
        raise RuntimeError('expected simulation system 1')
    if heartbeat.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED:
        connection.close()
        raise RuntimeError('set simulation references only while disarmed')
    # Fixed reference metadata, not a GPS measurement or navigation input.
    latitude, longitude, altitude = int(-35.363261*1e7), int(149.165230*1e7), 584000
    deadline, next_send = time.monotonic()+args.timeout, 0.
    try:
        while time.monotonic() < deadline:
            now = time.monotonic()
            if now >= next_send:
                # Early origin messages can be ignored before EKF startup.
                connection.mav.set_gps_global_origin_send(1, latitude, longitude, altitude)
                connection.mav.command_long_send(1, 1, mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE,
                                                 0, 49, 0, 0, 0, 0, 0, 0)
                next_send = now+1.
            origin = connection.recv_match(type='GPS_GLOBAL_ORIGIN', blocking=True, timeout=.5)
            if (origin is not None and origin.get_srcSystem() == 1 and origin.get_srcComponent() == 1
                    and abs(origin.latitude-latitude) <= 2 and abs(origin.longitude-longitude) <= 2
                    and abs(origin.altitude-altitude) <= 50):
                print('Simulation EKF origin confirmed; confirming fixed launch home.')
                break
        else:
            raise RuntimeError('SITL did not confirm the expected origin; do not start the mission')
        # Home and EKF origin are distinct metadata. With GPS disabled the
        # automatic home update may never occur, despite healthy local odometry.
        # Set the known simulator launch reference, not a GPS observation.
        deadline, next_send = time.monotonic()+args.timeout, 0.
        stable_since, origin_received, origin_stamp = None, -math.inf, None
        while time.monotonic() < deadline:
            now = time.monotonic()
            if now >= next_send:
                # Keep trying after EKF startup; an early origin can be lost
                # when the estimator initialises/reinitialises its core.
                connection.mav.set_gps_global_origin_send(1, latitude, longitude, altitude)
                connection.mav.command_int_send(1, 1, mavutil.mavlink.MAV_FRAME_GLOBAL,
                                                mavutil.mavlink.MAV_CMD_DO_SET_HOME,
                                                0, 0, 0, 0, 0, 0,
                                                latitude, longitude, altitude/1000.)
                connection.mav.command_long_send(1, 1, mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE,
                                                 0, 49, 0, 0, 0, 0, 0, 0)
                connection.mav.command_long_send(1, 1, mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE,
                                                 0, 242, 0, 0, 0, 0, 0, 0)
                next_send = now+1.
            message = connection.recv_match(type=['GPS_GLOBAL_ORIGIN', 'HOME_POSITION'], blocking=True, timeout=.5)
            if (message is not None and message.get_type() == 'GPS_GLOBAL_ORIGIN'
                    and message.get_srcSystem() == 1 and message.get_srcComponent() == 1
                    and abs(message.latitude-latitude) <= 2 and abs(message.longitude-longitude) <= 2
                    and abs(message.altitude-altitude) <= 50):
                origin_received, origin_stamp = time.monotonic(), message.time_usec
                continue
            home = message if message is not None and message.get_type() == 'HOME_POSITION' else None
            if (home is not None and home.get_srcSystem() == 1 and home.get_srcComponent() == 1
                    and abs(home.latitude-latitude) <= 2 and abs(home.longitude-longitude) <= 2
                    and abs(home.altitude-altitude) <= 50
                    and all(math.isfinite(value) for value in (home.x, home.y, home.z))):
                # EKF core initialisation can erase an early accepted origin.
                # Require repeated live origin+home confirmations across three
                # FCU/simulation seconds, not a single pre-initialisation ACK.
                if time.monotonic()-origin_received > 2. or origin_stamp is None:
                    stable_since = None
                    continue
                stable_since = home.time_usec if stable_since is None else stable_since
                if home.time_usec-stable_since < 3000000:
                    continue
                # Tell the running ROS mission that reference setup is finished.
                # Calibrating FCU local coordinates before origin/home setup
                # can freeze an offset that changes during EKF initialisation.
                import rclpy
                from std_msgs.msg import Bool, String
                from rclpy.qos import QoSProfile, DurabilityPolicy
                rclpy.init()
                node = rclpy.create_node('simulation_reference_confirmation')
                pub = node.create_publisher(Bool, '/simulation/reference_ready',
                    QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
                status = {}
                node.create_subscription(String, '/navigation/status',
                    lambda msg: status.update(json.loads(msg.data)), 10)
                until = time.monotonic()+30.
                acknowledged = False
                while time.monotonic() < until:
                    pub.publish(Bool(data=True))
                    rclpy.spin_once(node, timeout_sec=.1)
                    if status.get('mission_kind') not in ('mapping', 'identification') and status:
                        acknowledged = True
                        break
                    if (status.get('reference_ready') or (status.get('fcu_frame_aligned')
                            and status.get('health_reason') != 'waiting_for_confirmed_origin_and_home')):
                        acknowledged = True
                        break
                node.destroy_node()
                rclpy.shutdown()
                if not acknowledged:
                    raise RuntimeError('ROS mission did not acknowledge confirmed references; do not start')
                print('Simulation origin and home confirmed. GPS navigation remains disabled.')
                return
            elif home is not None:
                stable_since = None
        raise RuntimeError('SITL did not confirm launch home; do not start the mission')
    finally:
        connection.close()


if __name__ == '__main__':
    main()
