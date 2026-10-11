"""Simulation-only watchdog: read status, log faults, stop the exact owned run."""
import argparse
import json
from pathlib import Path
import subprocess
import time
import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import String

ROOT = Path(__file__).resolve().parents[1]

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--run',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--seconds',type=float,default=1800.)
    args = parser.parse_args()
    run = args.run.resolve()
    if run.parent != ROOT/'.dependencies' or not (run/'session.json').exists():
        raise ValueError('watchdog requires a registered private simulation run')
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = Node('tree_inspection_audit_watchdog')
    status = {}
    receipt = [time.monotonic()]
    def receive(msg):
        status.clear()
        status.update(json.loads(msg.data))
        receipt[0] = time.monotonic()
    node.create_subscription(String,'/navigation/status',receive,10)
    args.output.parent.mkdir(parents=True,exist_ok=True)
    started = time.monotonic()
    previous = None
    fault = None
    try:
        with args.output.open('w') as stream:
            while time.monotonic()-started < args.seconds:
                rclpy.spin_once(node,timeout_sec=.2)
                event = tuple(status.get(k) for k in ('flight','mission','tree_phase','inspection_tree_id','inspection_view','lidar_tilt_target','failure'))
                if event != previous and status:
                    previous = event
                    stream.write(json.dumps(status,allow_nan=False)+'\n')
                    stream.flush()
                    print(json.dumps({k:status.get(k) for k in ('simulation_time','flight','mission','tree_phase','inspection_tree_id','inspection_view','lidar_tilt_target','altitude_m','ascent_stop_reason','failure','health_reason')}),flush=True)
                failure = status.get('failure','')
                if failure and failure not in ('route_changed','terrain_replan_required'):
                    fault = failure
                if time.monotonic()-receipt[0] > 15.:
                    fault = 'navigation_status_missing_wall_15_seconds'
                if status.get('mission') == 'COMPLETE':
                    break
                if fault:
                    stream.write(json.dumps(dict(watchdog_fault=fault,wall_time=time.time(),last_status=status))+'\n')
                    stream.flush()
                    subprocess.run(['/usr/bin/python3',str(ROOT/'tools/stop_simulation.py'),'--run',str(run)],check=True)
                    print('WATCHDOG STOP: '+fault,flush=True)
                    break
    except KeyboardInterrupt:
        print('Observer stopped; simulation not signalled.',flush=True)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__=='__main__':
    main()
