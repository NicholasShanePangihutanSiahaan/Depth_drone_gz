"""Read-only live mission recorder. Never arms or changes flight mode."""
import argparse
import json
import math
import time
from pathlib import Path
import rclpy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State, PositionTarget
from rclpy.qos import qos_profile_sensor_data
from check_ros_simulation import Probe


class Watch(Probe):
    def __init__(self):
        super().__init__()
        self.events, self.previous = [], None
        self.flight = State()
        self.setpoints, self.tracking_errors = 0, []
        self.create_subscription(State, '/mavros/state', self.flight_cb, qos_profile_sensor_data)
        self.create_subscription(PoseStamped, '/mavros/setpoint_position/local', self.command, 10)
        self.create_subscription(PositionTarget, '/mavros/setpoint_raw/local', self.command, 10)
        self.create_subscription(PoseStamped, '/control/safe_target_pose', self.tracking_command, 10)

    def flight_cb(self, msg):
        self.flight = msg

    def navigation(self, msg):
        super().navigation(msg)
        key = (self.status.get('flight'), self.status.get('mission'), self.status.get('exploration_phase'),
               self.status.get('failure'), self.status.get('health_reason'), self.status.get('survey_waypoint'),
               self.flight.armed, self.flight.mode)
        if key != self.previous:
            event = {'simulation_time': self.get_clock().now().nanoseconds*1e-9,
                     **self.status, 'armed': self.flight.armed, 'fcu_mode': self.flight.mode}
            self.events.append(event)
            self.previous = key
            print(json.dumps({k: event.get(k) for k in ('simulation_time', 'flight', 'mission',
                'survey_waypoint', 'failure', 'health_reason', 'armed', 'fcu_mode')}), flush=True)

    def command(self, msg):
        self.setpoints += 1

    def tracking_command(self, msg):
        # The safe target is in map ENU; final MAVROS commands may be transformed
        # into fcu_local. Never compare positions from different coordinate frames.
        if '/localisation/odometry' in self.poses:
            p = msg.pose.position
            self.tracking_errors.append(math.dist([p.x, p.y, p.z], self.poses['/localisation/odometry'][0]))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--seconds', type=float, default=60.)
    parser.add_argument('--output', type=Path, default=Path('reports/exploration/live_ros.json'))
    args = parser.parse_args()
    rclpy.init()
    node = Watch()
    started = time.monotonic()
    try:
        while time.monotonic()-started < args.seconds:
            rclpy.spin_once(node, timeout_sec=.1)
    except KeyboardInterrupt:
        pass
    report = {'mode': node.status.get('mode'), 'events': node.events,
              'final_status': node.status, 'setpoints_received': node.setpoints,
              'live_exploration_observed': any(e.get('mission') == 'EXPLORE' and e['armed'] for e in node.events),
              'tracking_error_max_m': max(node.tracking_errors, default=None),
              'setpoint_publishers': node.count_publishers('/mavros/setpoint_position/local'),
              'commands_sent_by_recorder': False, 'full_mission_validated': False,
              'localisation_accuracy_validated': False}
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps(report, indent=2))
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
