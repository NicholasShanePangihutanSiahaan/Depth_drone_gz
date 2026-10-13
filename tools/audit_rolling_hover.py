"""Read-only ROS audit, separate body pose, FCU pose and measured sensor roll."""
import argparse
import json
import math
import time
import subprocess
import sys
from pathlib import Path
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from scipy.spatial.transform import Rotation
from rclpy.executors import ExternalShutdownException


def pose(p):
    q = p.orientation
    return dict(xyz=[p.position.x,p.position.y,p.position.z],
                rpy=Rotation.from_quat([q.x,q.y,q.z,q.w]).as_euler('xyz').tolist())


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--seconds',type=float,default=180.)
    parser.add_argument('--stop-run',type=Path,help='Stop only this registered simulation at the first failure')
    args = parser.parse_args()
    rclpy.init()
    node = Node('rolling_hover_readonly_audit')
    node.set_parameters([rclpy.parameter.Parameter('use_sim_time',value=True)])
    latest = {}
    node.create_subscription(Odometry,'/simulation/ground_truth/odom',lambda m:latest.update(body=pose(m.pose.pose)),qos_profile_sensor_data)
    node.create_subscription(PoseStamped,'/mavros/local_position/pose',lambda m:latest.update(fcu=pose(m.pose)),qos_profile_sensor_data)
    node.create_subscription(String,'/navigation/status',lambda m:latest.update(status=json.loads(m.data)),10)
    args.output.parent.mkdir(parents=True,exist_ok=True)
    deadline,last = time.monotonic()+args.seconds,-math.inf
    samples = []
    try:
        with args.output.open('w') as stream:
            while time.monotonic()<deadline:
                rclpy.spin_once(node,timeout_sec=.1)
                now = node.get_clock().now().nanoseconds*1e-9
                if now < last:
                    break  # A new simulator must not contaminate this run's audit.
                if now-last<.5 or 'status' not in latest:
                    continue
                last = now
                d = latest['status']
                row = {k:d.get(k) for k in ('simulation_time','flight','mission','failure','health_reason','armed','altitude_m','tracking_error_m','sensor_mounts','tree_phase','inspection_tree_id','inspected_tree_ids','scan_anchor','scan_sector_records','scan_start_aligned','commanded_speed_mps','exploration_phase','planner_stages_last_wall_ms')}
                row.update(body=latest.get('body'),fcu=latest.get('fcu'))
                samples.append(row)
                stream.write(json.dumps(row)+'\n'); stream.flush()
                print(json.dumps({k:row[k] for k in ('simulation_time','flight','mission','failure','altitude_m','tree_phase','scan_anchor','commanded_speed_mps')}),flush=True)
                if d.get('failure'):
                    if args.stop_run:
                        subprocess.run([sys.executable,str(Path(__file__).resolve().parent/'stop_simulation.py'),
                            '--run',str(args.stop_run)],check=True)
                    break
    except (KeyboardInterrupt,ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    hover = [s for s in samples if s['flight']=='MISSION' and s['armed'] and not s['failure'] and abs(s['altitude_m']-2.)<.15]
    summary = dict(samples=len(samples),hover_samples=len(hover),
                   failures=sorted(set(s['failure'] for s in samples if s['failure'])),
                   scope='ground_truth_simulation_not_LIO',output=str(args.output))
    print(json.dumps(summary),flush=True)
    args.output.with_suffix('.summary.json').write_text(json.dumps(summary,indent=2)+'\n')


if __name__ == '__main__':
    main()
