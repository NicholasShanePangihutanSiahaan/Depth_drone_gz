"""READ-ONLY estimator accuracy audit; ground truth never leaves this node."""
import argparse,json,time,subprocess,sys
from approach_viewer import ApproachViewer
from collections import deque
from pathlib import Path
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from scipy.spatial.transform import Rotation


class RecoveryWatch:
    """Observe the existing brake/replan state; never send flight commands."""
    def __init__(self):
        self.started=None
        self.wall_started=None
        self.recent=[]
        self.events=[]
        self.depth_started=None
        self.depth_wall_started=None
        self.depth_recent=[]

    def check(self,nav,wall):
        failure=nav.get('failure','')
        stamp=nav.get('simulation_time',0.)
        if self.depth_started is not None and failure!='depth_recovering':
            self.events.append(dict(event='depth_recovery_ended',stamp=stamp,failure=failure))
            self.depth_started=self.depth_wall_started=None
        if failure=='depth_recovering':
            if self.started is not None:
                self.events.append(dict(event='recovery_ended',stamp=stamp,failure=failure))
                self.started=self.wall_started=None
            return self._depth_check(nav,wall)
        if failure!='route_changed':
            if self.started is not None:
                self.events.append(dict(event='recovery_ended',stamp=stamp,failure=failure))
                self.started=self.wall_started=None
            return failure
        if nav.get('mission')!='BRAKE' or nav.get('health_reason'):
            return failure
        if self.started is None:
            self.started,self.wall_started=stamp,wall
            self.recent=[t for t in self.recent if stamp-t<20.]+[stamp]
            self.events.append(dict(event='map_change_brake',stamp=stamp,
                diagnostics=nav.get('route_failure_diagnostics',{})))
        if len(self.recent)>3:return 'repeated_map_change_recovery'
        if stamp-self.started>5. or wall-self.wall_started>60.:return 'map_change_recovery_timeout'
        return ''

    def _depth_check(self,nav,wall):
        """Observe the controller's existing 2s brake/settle recovery only.

        Receiving frames is insufficient: controller still requires a newer
        integrated map and actual settling before clearing its recovery flag.
        """
        import math
        stamp=nav.get('simulation_time',0.)
        status=nav.get('depth_motion_recovery',{})
        if nav.get('health_reason'):
            return nav['health_reason']
        if (nav.get('mission') not in ('BRAKE','HOLD_ABORT')
                or status.get('action') not in ('certified_brake_started','braking_or_waiting')):
            return 'depth_recovering'
        valid_age=status.get('valid_age')
        timeout=nav.get('depth_safety',{}).get('timeout_seconds')
        limit=status.get('hold_timeout_seconds')
        started=status.get('started')
        if (not all(isinstance(v,(float,int)) and math.isfinite(v) for v in (valid_age,timeout,limit,started))
                or timeout<=0 or limit<=0 or valid_age<0 or valid_age>timeout+1e-9):
            return 'stale_depth_camera'
        if not -1e-9<=stamp-started<=min(2.,limit)+1e-9:
            return 'stale_depth_camera'
        if self.depth_started is None:
            self.depth_started,self.depth_wall_started=started,wall
            self.depth_recent=[t for t in self.depth_recent if stamp-t<20.]+[stamp]
            self.events.append(dict(event='depth_brake_started',stamp=stamp,
                diagnostics=status))
        if len(self.depth_recent)>3:return 'repeated_depth_recovery'
        if stamp-self.depth_started>min(2.,limit)+1e-9:return 'stale_depth_camera'
        if wall-self.depth_wall_started>60.:return 'depth_recovery_wall_timeout'
        return ''


def main():
    p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True)
    p.add_argument('--seconds',type=float,default=1800.)
    p.add_argument('--stop-run',type=Path)
    p.add_argument('--rviz-after-approach',action='store_true')
    args=p.parse_args()
    rclpy.init();node=Node('sensor_localisation_readonly_audit')
    truths=deque(maxlen=1000);latest={};samples=[];motion={}
    recovery=RecoveryWatch();events_written=0
    viewer=ApproachViewer(args.stop_run,args.rviz_after_approach)
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
        estimated_velocity=er.apply([ev.x,ev.y,ev.z]);truth_velocity=tr.apply([tv.x,tv.y,tv.z])
        previous=motion.get("previous")
        acceleration=((estimated_velocity-previous[1])/(t-previous[0])).tolist() if previous is not None and t>previous[0] else None
        motion["previous"]=(t,estimated_velocity.copy())
        row=dict(estimated_velocity_mps=estimated_velocity.tolist(),truth_velocity_mps=truth_velocity.tolist(),
            estimated_acceleration_mps2=acceleration,acceleration_method="finite_difference_accepted_sensor_velocity",
            stamp=t,position_error_m=float(np.linalg.norm(ep-tp)),
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
    args.output.parent.mkdir(parents=True,exist_ok=True)
    def diagnostics(msg):
        value=json.loads(msg.data)
        latest['debug']=value
        with args.output.with_suffix('.localisation.jsonl').open('a') as output:
            output.write(json.dumps(value)+'\n')
    def navigation(msg):
        value=json.loads(msg.data)
        latest['nav']=value
        with args.output.with_suffix('.navigation.jsonl').open('a') as output:
            output.write(json.dumps(value)+'\n')
    node.create_subscription(String,'/localisation/debug',diagnostics,10)
    node.create_subscription(String,'/navigation/status',navigation,10)
    deadline=time.monotonic()+args.seconds;last_print=0.
    try:
        with args.output.open('w') as stream:
            while time.monotonic()<deadline:
                rclpy.spin_once(node,timeout_sec=.1)
                if time.monotonic()-last_print>10:
                    print(json.dumps(dict(pairs=len(samples),latest=samples[-1] if samples else latest.get('debug',{}))),flush=True)
                    last_print=time.monotonic()
                nav=latest.get('nav',{})
                viewer.update(nav)
                if args.stop_run:
                    session=args.stop_run/'session.json'
                    if session.exists() and json.loads(session.read_text()).get('stopped'):
                        latest['stop_reason']='registered_simulation_stopped'
                        break
                fault=recovery.check(nav,time.monotonic())
                if fault:latest['stop_reason']=fault
                if len(recovery.events)>events_written:
                    with args.output.with_suffix('.events.jsonl').open('a') as event_stream:
                        for event in recovery.events[events_written:]:event_stream.write(json.dumps(event)+'\n')
                    events_written=len(recovery.events)
                if latest.get('stop_reason'):
                    if args.stop_run and latest['stop_reason']!='simulator_clock_reset':
                        subprocess.run([sys.executable,str(Path(__file__).resolve().parent/'stop_simulation.py'),
                            '--run',str(args.stop_run)],check=True)
                    break
                if nav.get('mission')=='COMPLETE' and not nav.get('armed'):break
    except KeyboardInterrupt:pass
    finally:
        viewer.close()
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
    errors=[r['position_error_m'] for r in samples]
    summary=dict(scope='simulation_sensor_pose_accuracy_not_hardware_validation',samples=len(samples),
        position_rmse_m=float(np.sqrt(np.mean(np.square(errors)))) if errors else None,
        position_max_error_m=max(errors) if errors else None,last_navigation=latest.get('nav',{}),
        last_localisation_debug=latest.get('debug',{}),
        stop_reason=latest.get('stop_reason',''),
        recovery_events=recovery.events,viewer_events=viewer.events,
        ground_truth_used_by_estimator=False)
    args.output.with_suffix('.summary.json').write_text(json.dumps(summary,indent=2)+'\n')
    print(json.dumps({k:v for k,v in summary.items() if k!='last_navigation'}),flush=True)


if __name__=='__main__':main()
