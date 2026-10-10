"""Separate read-only recorder and bounded adaptive-model shadow observer."""
import json
import os
from collections import deque
from pathlib import Path
import numpy as np
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from mavros_msgs.msg import PositionTarget
from tf2_ros import Buffer, TransformListener, TransformException
from .ros_nodes import ConfigNode, run, rotation, vector, seconds
from .adaptive_model import GuardedRLS, load_validated_seed


class IdentificationRecorder(ConfigNode):
    def __init__(self):
        super().__init__('polinasi_identification_recorder')
        if not self.c.get('identification_simulation_only'):
            raise RuntimeError('Identification recorder requires simulation preset')
        self.declare_parameter('output_dir', '')
        directory = self.get_parameter('output_dir').value
        if not directory:
            raise ValueError('provide a separate output_dir')
        self.directory = Path(directory)
        self.directory.mkdir(parents=True, exist_ok=True)
        if (self.directory/'identification.json').exists():
            raise ValueError('refusing to overwrite an existing identification dataset')
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.status, self.last_status_time = {}, -np.inf
        self.commands, self.responses, self.imu_samples, self.events = [], [], [], []
        self.models = deque(maxlen=1500)
        self.command_history = deque(maxlen=300)
        seeds, self.delays, self.seed_source = load_validated_seed(self.c.get('adaptive_model_file'), self.c)
        self.estimators = [GuardedRLS(initial=seed, forgetting=self.c['adaptive_forgetting'],
            window=self.c['adaptive_window']) for seed in seeds]
        self.get_logger().info(f'Shadow model seed: {self.seed_source}; delays={self.delays}')
        self.previous, self.last_adaptation = None, -np.inf
        self.armed_seen = self.land_seen = self.written = False
        self.truncated = False
        self.single_producer_seen, self.producer_violation = False, False
        self.pub = self.create_publisher(String, '/identification/model_candidate', 10)
        self.create_subscription(PositionTarget, '/mavros/setpoint_raw/local', self.command, 10)
        self.create_subscription(Odometry, '/localisation/odometry', self.response, qos_profile_sensor_data)
        self.create_subscription(Imu, '/livox/imu', self.imu, qos_profile_sensor_data)
        self.create_subscription(String, '/navigation/status', self.navigation, 10)
        self.create_timer(.5, self.publish_model)

    def navigation(self, msg):
        status = json.loads(msg.data)
        if status.get('mission_kind') != 'identification' or status.get('mode') != 'ground_truth':
            return
        key = (status['mission'], status['identification_phase'], status['armed'], status['fcu_mode'], status['failure'])
        old = (self.status.get('mission'), self.status.get('identification_phase'), self.status.get('armed'),
               self.status.get('fcu_mode'), self.status.get('failure'))
        self.status, self.last_status_time = status, self.now()
        self.armed_seen |= status['armed']
        self.land_seen |= status['fcu_mode'] == 'LAND' and self.armed_seen
        if key != old:
            self.events.append(dict(t=self.now(), status=status))
        if status['mission'] == 'COMPLETE' and not status['armed'] and not self.written:
            self.write()
        elif status['mission'] == 'HOLD_ABORT' and not self.written:
            self.write()

    def command(self, msg):
        if self.written or msg.header.frame_id != 'fcu_local' or msg.type_mask != PositionTarget.IGNORE_YAW_RATE:
            return
        if len(self.commands) >= 12000:
            self.truncated = True
            return
        t = seconds(msg.header.stamp)
        if self.command_history and t <= self.command_history[-1]['t']:
            return
        try:
            transform = self.buffer.lookup_transform('map', 'fcu_local', Time.from_msg(msg.header.stamp))
            r = rotation(transform.transform.rotation)
            p = r@vector(msg.position)+vector(transform.transform.translation)
            v, a = r@vector(msg.velocity), r@vector(msg.acceleration_or_force)
        except (TransformException, ValueError):
            return
        if not np.all(np.isfinite(np.r_[p, v, a])):
            return
        yaw = msg.yaw+np.arctan2(r[1, 0], r[0, 0])
        row = dict(t=t, p=p.tolist(), v=v.tolist(), a=a.tolist(), yaw=float(yaw),
                   frame='map', source_frame='fcu_local', mask=msg.type_mask)
        self.commands.append(row)
        self.command_history.append(row)

    def response(self, msg):
        if self.written or msg.header.frame_id != 'odom' or msg.child_frame_id != 'base_link':
            return
        if len(self.responses) >= 30000:
            self.truncated = True
            return
        t = seconds(msg.header.stamp)
        if self.responses and t <= self.responses[-1]['t']:
            return
        try:
            body = rotation(msg.pose.pose.orientation)
            transform = self.buffer.lookup_transform('map', 'odom', Time.from_msg(msg.header.stamp))
            basis = rotation(transform.transform.rotation)
        except (TransformException, ValueError):
            return
        p = basis@vector(msg.pose.pose.position)+vector(transform.transform.translation)
        v = basis@body@vector(msg.twist.twist.linear)
        if not np.all(np.isfinite(np.r_[p, v])):
            return
        q = msg.pose.pose.orientation
        row = dict(t=t, p=p.tolist(), v=v.tolist(), quaternion_odom=[q.x, q.y, q.z, q.w],
                   omega_body=vector(msg.twist.twist.angular).tolist(), frame='map',
                   phase=self.status.get('identification_phase', 'WAIT'),
                   healthy=bool(self.status.get('flight') == 'MISSION' and self.status.get('mission') == 'IDENTIFY'
                     and not self.status.get('failure')
                     and not self.status.get('health_reason') and 0 <= self.now()-self.last_status_time < .3))
        self.responses.append(row)
        # Keep one sample per ~50 ms. Finite-difference output is used only in
        # a shadow model; it is not interpreted as a thrust/mass measurement.
        if t-self.last_adaptation < .045:
            return
        old = self.previous
        self.previous, self.last_adaptation = row, t
        if old is None:
            return
        dt = t-old['t']
        for axis, estimator in enumerate(self.estimators):
            candidates = [command for command in self.command_history if command['t'] <= old['t']-self.delays[axis]]
            if not candidates:
                continue
            command = candidates[-1]
            healthy = row['healthy'] and old['healthy'] and old['t']-self.delays[axis]-command['t'] <= .12
            phi = [command['p'][axis]-old['p'][axis], command['v'][axis]-old['v'][axis], command['a'][axis], 1.]
            estimator.update(phi, (row['v'][axis]-old['v'][axis])/max(dt, 1e-9), dt, healthy)

    def imu(self, msg):
        if not self.written and len(self.imu_samples) < 100000:
            self.imu_samples.append(dict(t=seconds(msg.header.stamp), frame=msg.header.frame_id,
                specific_force=vector(msg.linear_acceleration).tolist(), omega=vector(msg.angular_velocity).tolist()))

    def publish_model(self):
        count = self.count_publishers('/mavros/setpoint_raw/local')
        self.single_producer_seen |= count == 1
        if self.status.get('mission_start_received') and count != 1:
            self.producer_violation = True
        model = dict(t=self.now(), mode='shadow_only', delay_seconds_by_axis=dict(zip('xyz', self.delays)),
            seed_source=self.seed_source,
            equation='dv/dt = kp*(p_cmd-p) + kv*(v_cmd-v) + ka*a_cmd + bias',
            axes={axis: estimator.snapshot() for axis, estimator in zip('xyz', self.estimators)},
            estimated_mass_kg=None, controls_flight=False)
        self.pub.publish(String(data=json.dumps(model)))
        if not self.written:
            self.models.append(model)

    def write(self):
        completed = bool(self.status.get('mission') == 'COMPLETE' and self.status.get('protocol_completed')
                         and not self.status.get('armed') and self.armed_seen and self.land_seen
                         and not self.status.get('failure') and not self.truncated)
        report = dict(format_version=1, scope='Gazebo/ArduPilot PVA closed-loop identification',
            localisation_mode='ground_truth', lidar_localisation_validated=False, hardware_validated=False,
            lidar_enabled=not self.c.get('identification_empty_arena', False),
            arena_source='known_simulation_geometry_not_sensor_mapping',
            model_controls_flight=bool(self.status.get('model_controls_flight')),
            config=self.c, commands=self.commands, responses=self.responses,
            imu=self.imu_samples, events=self.events, adaptive_models=list(self.models), final_status=self.status,
            completed=completed, truncated=self.truncated,
            # Context can already be invalid after SIGINT. Cached live graph
            # observations preserve partial logs without a shutdown ROS call.
            one_final_publisher=self.single_producer_seen and not self.producer_violation,
            note='Effective small-signal closed-loop model, not mass/inertia identification. '
                 'Command stamps are ROS send times; estimated delay includes the MAVROS/FCU chain. '
                 'The recorder RLS stays shadow-only. Final PVA may use active MPC; '
                 'check model_controls_flight and predictive status. No ArduPilot PID updates.')
        temporary = self.directory/'identification.json.tmp'
        temporary.write_text(json.dumps(report, indent=2)+'\n')
        os.replace(temporary, self.directory/'identification.json')
        self.written = True
        self.get_logger().info(f'Identification dataset saved: {self.directory}/identification.json; completed={completed}')

    def destroy_node(self):
        if not self.written:
            self.write()
        return super().destroy_node()


def main(args=None):
    run(IdentificationRecorder, args)
