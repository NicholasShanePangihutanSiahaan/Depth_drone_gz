"""Read-only physical-link pose audit; no command publishers."""
import argparse
import json
import time
from pathlib import Path
from gz.transport13 import Node
from gz.msgs10.pose_v_pb2 import Pose_V

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--seconds',type=float,default=600.)
    args = parser.parse_args()
    node = Node()
    args.output.parent.mkdir(parents=True,exist_ok=True)
    with args.output.open('w') as output:
        def receive(msg):
            poses = []
            for p in msg.pose:
                if 'mid360' in p.name or 'base_link' in p.name or 'camera' in p.name:
                    poses.append(dict(name=p.name,xyz=[p.position.x,p.position.y,p.position.z],q=[p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w]))
            if poses:
                output.write(json.dumps(dict(stamp=msg.header.stamp.sec+msg.header.stamp.nsec*1e-9,poses=poses))+'\n')
                output.flush()
        if not node.subscribe(Pose_V,'/model/iris_lidar/pose',receive):
            raise RuntimeError('pose subscription failed')
        deadline = time.monotonic()+args.seconds
        while time.monotonic()<deadline:
            time.sleep(.1)
        node.unsubscribe('/model/iris_lidar/pose')

if __name__=='__main__': main()
