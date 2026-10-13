"""READ-ONLY estimator accuracy audit; ground truth never leaves this node."""
import argparse,json,time,subprocess,sys
from collections import deque
from pathlib import Path
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from scipy.spatial.transform import Rotation


def main():
    p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True)
    p.add_argument('--seconds',type=float,default=1800.)
    p.add_argument('--stop-run',type=Path)
    args=p.parse_args()
    rclpy.init();node=Node('sensor_localisation_readonly_audit')
    truths=deque(maxlen=1000);latest={};samples=[]
    def stamp(msg):return msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
    def values(msg):
        pos=msg.pose.pose.position;q=msg.pose.pose.orientation
        return np.array([pos.x,pos.y,pos.z]),Rotation.from_quat([q.x,q.y,q.z,q.w])
    def estimate(msg):
        t=stamp(msg)
        if not truths:return
        # Measurement-time match, not latest truth; unmatched pairs excluded.
        source=min(truths,key=lambda m:abs(stamp(m)-t))
        if abs(stamp(source)-t)>.011:return
        ep,er=values(msg);tp,tr=values(source)
        ev=msg.twist.twist.linear;tv=source.twist.twist.linear
        row=dict(stamp=t,position_error_m=float(np.linalg.norm(ep-tp)),
            orientation_error_rad=float((tr.inv()*er).magnitude()),
            velocity_error_mps=float(np.linalg.norm(er.apply([ev.x,ev.y,ev.z])-tr.apply([tv.x,tv.y,tv.z]))),
            estimated_position=ep.tolist(),truth_position=tp.tolist(),
            match_error_seconds=abs(stamp(source)-t),debug=latest.get('debug',{}))
        samples.append(row);stream.write(json.dumps(row)+'\n');stream.flush()
        if len(samples)>=5 and all(r['position_error_m']>.2 or r['orientation_error_rad']>.2
                or r['velocity_error_mps']>.15 for r in samples[-5:]):
            latest['stop_reason']='sensor_accuracy_exceeds_test_margin'
    def truth(msg):
        if truths and stamp(msg)<stamp(truths[-1]):latest['stop_reason']='simulator_clock_reset'
        truths.append(msg)
    node.create_subscription(Odometry,'/simulation/ground_truth/odom',truth,qos_profile_sensor_data)
    node.create_subscription(Odometry,'/localisation/odometry',estimate,qos_profile_sensor_data)
    node.create_subscription(String,'/localisation/debug',lambda m:latest.update(debug=json.loads(m.data)),10)
    node.create_subscription(String,'/navigation/status',lambda m:latest.update(nav=json.loads(m.data)),10)
    args.output.parent.mkdir(parents=True,exist_ok=True)
    deadline=time.monotonic()+args.seconds;last_print=0.
    try:
        with args.output.open('w') as stream:
            while time.monotonic()<deadline:
                rclpy.spin_once(node,timeout_sec=.1)
                if time.monotonic()-last_print>10:
                    print(json.dumps(dict(pairs=len(samples),latest=samples[-1] if samples else latest.get('debug',{}))),flush=True)
                    last_print=time.monotonic()
                nav=latest.get('nav',{})
                if nav.get('failure'):latest['stop_reason']=nav['failure']
                if latest.get('stop_reason'):
                    if args.stop_run and latest['stop_reason']!='simulator_clock_reset':
                        subprocess.run([sys.executable,str(Path(__file__).resolve().parent/'stop_simulation.py'),
                            '--run',str(args.stop_run)],check=True)
                    break
                if nav.get('mission')=='COMPLETE' and not nav.get('armed'):break
    except KeyboardInterrupt:pass
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
    errors=[r['position_error_m'] for r in samples]
    summary=dict(scope='simulation_sensor_pose_accuracy_not_hardware_validation',samples=len(samples),
        position_rmse_m=float(np.sqrt(np.mean(np.square(errors)))) if errors else None,
        position_max_error_m=max(errors) if errors else None,last_navigation=latest.get('nav',{}),
        stop_reason=latest.get('stop_reason',''),
        ground_truth_used_by_estimator=False)
    args.output.with_suffix('.summary.json').write_text(json.dumps(summary,indent=2)+'\n')
    print(json.dumps({k:v for k,v in summary.items() if k!='last_navigation'}),flush=True)


if __name__=='__main__':main()
