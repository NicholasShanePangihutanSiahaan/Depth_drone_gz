"""Apply/acknowledge child-plugin settings ignored by this MAVROS build's CLI.

Plugin::Plugin disables use_global_arguments; launch parameter files therefore
do not reach those nodes. Configure through their ordinary ROS parameter API.
"""
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, DurabilityPolicy
from rclpy.executors import ExternalShutdownException
from rcl_interfaces.srv import SetParameters
from std_msgs.msg import Bool

PLUGIN_SETTINGS = {
    '/mavros/time': {'use_sim_time': True, 'timesync_mode': 'NONE', 'timesync_rate': 0.},
    '/mavros/local_position': {'use_sim_time': True, 'frame_id': 'fcu_local', 'tf.send': False},
    '/mavros/odometry': {'use_sim_time': True, 'fcu.odom_parent_id_des': 'odom',
                         'fcu.odom_child_id_des': 'base_link', 'fcu.map_id_des': 'odom'},
    '/mavros/setpoint_raw': {'use_sim_time': True},
    '/mavros/cmd': {'use_sim_time': True},
    '/mavros/imu': {'use_sim_time': True},
    '/mavros/sys': {'use_sim_time': True},
}


class MavrosConfigurator(Node):
    def __init__(self):
        super().__init__('polinasi_mavros_configurator')
        self.pub = self.create_publisher(Bool, '/simulation/mavros_ready',
                                        QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.parameter_clients = {name: self.create_client(SetParameters, name+'/set_parameters') for name in PLUGIN_SETTINGS}
        self.pending = {}
        self.done = set()
        self.pub.publish(Bool(data=False))
        self.timer = self.create_timer(.5, self.tick)

    def tick(self):
        for name, settings in PLUGIN_SETTINGS.items():
            if name in self.done:
                continue
            if name in self.pending:
                if not self.pending[name].done():
                    continue
                response = self.pending.pop(name).result()
                if response is None or len(response.results) != len(settings) or not all(r.successful for r in response.results):
                    reasons = [r.reason for r in response.results] if response else ['no response']
                    self.get_logger().error(f'MAVROS plugin configuration failed for {name}: {reasons}; arming remains blocked')
                    continue
                self.done.add(name)
                self.get_logger().info(f'Applied and acknowledged plugin settings: {name}')
            elif self.parameter_clients[name].service_is_ready():
                request = SetParameters.Request(parameters=[Parameter(key, value=value).to_parameter_msg()
                                                            for key, value in settings.items()])
                self.pending[name] = self.parameter_clients[name].call_async(request)
        self.pub.publish(Bool(data=len(self.done) == len(PLUGIN_SETTINGS)))


def main(args=None):
    rclpy.init(args=args)
    node = MavrosConfigurator()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception:
        # Ctrl+C can invalidate the ROS context while the executor builds its
        # next wait set. Suppress only this shutdown race, not live failures.
        if rclpy.ok():
            raise
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
