"""Read-only evidence for the separate mapping mission. Never sends commands."""
import argparse
import json
import math
import time
from pathlib import Path
import rclpy
from rclpy.signals import SignalHandlerOptions
from watch_exploration import Watch


class MappingWatch(Watch):
    def __init__(self):
        super().__init__()
        self.maximum_altitude = -math.inf
        self.armed_seen = False
        self.land_seen = False
        self.command_speeds, self.command_accelerations = [], []
        self.command_times, self.command_positions, self.command_velocities = [], [], []
        self.command_acceleration_vectors = []
        self.scales, self.clearances = [], []
        self.navigation_tracking_errors = []

    def command(self, msg):
        super().command(msg)
        if hasattr(msg, 'velocity'):
            self.command_speeds.append(math.sqrt(msg.velocity.x**2+msg.velocity.y**2+msg.velocity.z**2))
            a = msg.acceleration_or_force
            self.command_accelerations.append(math.sqrt(a.x*a.x+a.y*a.y+a.z*a.z))
            self.command_times.append(msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9)
            self.command_positions.append([msg.position.x, msg.position.y, msg.position.z])
            self.command_velocities.append([msg.velocity.x, msg.velocity.y, msg.velocity.z])
            self.command_acceleration_vectors.append([a.x, a.y, a.z])

    def navigation(self, msg):
        super().navigation(msg)
        if self.status.get('flight') == 'MISSION' and self.status.get('mission') in ('SURVEY', 'EXPLORE', 'RETURN'):
            error = self.status.get('tracking_error_m')
            if error is not None:
                self.navigation_tracking_errors.append(error)
        z = self.status.get('altitude_m')
        if z is not None:
            self.maximum_altitude = max(self.maximum_altitude, z)
        self.armed_seen |= bool(self.status.get('armed'))
        self.land_seen |= self.status.get('fcu_mode') == 'LAND' and self.armed_seen
        if self.status.get('speed_scale') is not None:
            self.scales.append(self.status['speed_scale'])
        if (self.status.get('map_clearance_m') is not None and self.status.get('flight') == 'MISSION'
                and self.status.get('mission') in ('SURVEY', 'EXPLORE', 'RETURN')):
            self.clearances.append(self.status['map_clearance_m'])


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--seconds', type=float, default=600.)
    parser.add_argument('--output', type=Path, default=Path('reports/mapping_live.json'))
    args = parser.parse_args()
    # Keep the ROS context alive through report collection on Ctrl+C.
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = MappingWatch()
    start = time.monotonic()
    observer_error = None
    try:
        while time.monotonic()-start < args.seconds:
            rclpy.spin_once(node, timeout_sec=.1)
            if node.status.get('mission') == 'COMPLETE':
                break
    except KeyboardInterrupt:
        pass
    except RuntimeError as exc:
        observer_error = str(exc)
    completed = (observer_error is None and node.status.get('mission_kind') == 'mapping'
                 and node.status.get('mission') == 'COMPLETE'
                 and not node.status.get('armed') and node.armed_seen and node.land_seen
                 and node.status.get('survey_waypoint') == node.status.get('survey_waypoint_count'))
    report = {'scope': 'live ROS/Gazebo mapping flight', 'mode': node.status.get('mode'),
              'events': node.events, 'final_status': node.status,
              'maximum_altitude_m': node.maximum_altitude if math.isfinite(node.maximum_altitude) else None,
              'armed_seen': node.armed_seen, 'land_seen': node.land_seen,
              'mapping_mission_completed': completed,
              'tracking_error_max_m': max(node.tracking_errors, default=None),
              'navigation_tracking_error_max_m': max(node.navigation_tracking_errors, default=None),
              'tracking_error_scope': 'legacy tracking_error_max_m may include landing; navigation metric excludes takeoff/LAND',
              'setpoints_received': node.setpoints,
              'final_setpoint_publishers': node.count_publishers(node.status.get('setpoint_interface', '/mavros/setpoint_position/local')),
              'position_interface_publishers': node.count_publishers('/mavros/setpoint_position/local'),
              'raw_interface_publishers': node.count_publishers('/mavros/setpoint_raw/local'),
              'command_speed_max_mps': max(node.command_speeds, default=None),
              'command_acceleration_max_mps2': max(node.command_accelerations, default=None),
              'adaptive_scale_min': min(node.scales, default=None),
              'known_map_clearance_min_m': min(node.clearances, default=None),
              'command_samples': dict(frame='fcu_local', times=node.command_times,
                                      positions=node.command_positions, velocities=node.command_velocities,
                                      accelerations=node.command_acceleration_vectors),
              'commands_sent_by_recorder': False, 'lidar_localisation_validated': False}
    report['observer_error'] = observer_error
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps({k: report[k] for k in ('mapping_mission_completed', 'maximum_altitude_m',
        'tracking_error_max_m', 'armed_seen', 'land_seen', 'observer_error')}, indent=2))
    print(f'Report: {args.output}')
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
