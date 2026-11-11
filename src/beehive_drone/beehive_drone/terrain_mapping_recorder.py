"""Record independent altitude channels and detected trees for SITL experiments."""
import csv
import math
import time
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import FluidPressure, Range
from std_msgs.msg import Float32, Float64
from uav_interfaces.msg import TreeArray


class Recorder(Node):
    def __init__(self):
        super().__init__('terrain_mapping_recorder')
        self.declare_parameter('output_directory', '~/beehive_mission_reports/terrain_mapping')
        self.directory = Path(self.get_parameter('output_directory').value).expanduser()/time.strftime('%Y%m%d_%H%M%S')
        self.directory.mkdir(parents=True)
        self.values = {}
        self.p0 = None
        self.trees = {}
        self.columns = ['sim_time', 'x', 'y', 'truth_z', 'local_z', 'target_z', 'desired_agl', 'filtered_agl', 'rangefinder_raw', 'pressure_pa', 'baro_relative_z', 'fcu_relative_alt', 'terrain_estimate_z']
        self.file = (self.directory/'altitudes.csv').open('w', newline='')
        self.writer = csv.DictWriter(self.file, fieldnames=self.columns)
        self.writer.writeheader()
        def sub(cls, topic, cb):
            self.create_subscription(cls, topic, cb, qos_profile_sensor_data)
        sub(PoseStamped, '/simulation/local_position/pose', self.truth)
        sub(PoseStamped, '/mavros/local_position/pose', lambda m: self.put('local_z', m.pose.position.z))
        sub(PoseStamped, '/mavros/setpoint_position/local', lambda m: self.put('target_z', m.pose.position.z))
        sub(Float32, '/control/target_agl', lambda m: self.put('desired_agl', m.data))
        sub(Float32, '/control/measured_agl', lambda m: self.put('filtered_agl', m.data))
        sub(Range, '/simulation/rangefinder', lambda m: self.put('rangefinder_raw', m.range))
        sub(FluidPressure, '/mavros/imu/static_pressure', self.pressure)
        sub(Float64, '/mavros/global_position/rel_alt', lambda m: self.put('fcu_relative_alt', m.data))
        self.create_subscription(TreeArray, '/map/trees', self.map_cb, 10)
        self.create_timer(0.2, self.record)
        self.get_logger().info(f'Recording to {self.directory}; missing/stale channels are NaN.')

    def put(self, key, value):
        self.values[key] = (float(value), time.monotonic())

    def truth(self, msg):
        p = msg.pose.position
        for key, value in [('x',p.x), ('y',p.y), ('truth_z',p.z)]:
            self.put(key,value)

    def pressure(self, msg):
        p = msg.fluid_pressure
        if not math.isfinite(p) or p <= 0:
            return
        if self.p0 is None:
            self.p0 = p
        self.put('pressure_pa', p)
        # ISA pressure-derived height relative to first pressure sample, not EKF Z.
        self.put('baro_relative_z', 44330.0*(1-(p/self.p0)**0.190294957))

    def map_cb(self, msg):
        for tree in msg.trees:
            self.trees[tree.id] = [tree.id,tree.x,tree.y,tree.z,tree.confidence]
        with (self.directory/'trees.csv').open('w',newline='') as f:
            w=csv.writer(f);w.writerow(['id','x','y','z','confidence']);w.writerows(self.trees.values())

    def record(self):
        now = time.monotonic()
        row={key:(self.values[key][0] if key in self.values and now-self.values[key][1]<1.0 else math.nan) for key in self.columns}
        row['sim_time']=self.get_clock().now().nanoseconds/1e9
        row['terrain_estimate_z']=row['truth_z']-row['filtered_agl']
        self.writer.writerow(row)
        self.file.flush()


def main():
    rclpy.init();node=Recorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.file.close();node.destroy_node();rclpy.shutdown()
