"""Attach two read-only viewers to an active registered simulation; own cleanup."""
import argparse
import fcntl
import json
import os
from pathlib import Path
import signal
import subprocess
import time
from stop_simulation import ROOT,directory,live


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run',type=directory,required=True)
    args = parser.parse_args()
    if not os.environ.get('DISPLAY') and not os.environ.get('WAYLAND_DISPLAY'):
        parser.exit(2,'No desktop display; no viewers opened.\n')
    data = json.loads((args.run/'session.json').read_text())
    if not live(data):
        parser.exit(2,'Simulation is not active.\n')
    children = []
    handles = []
    with (args.run/'attached_viewers.lock').open('w') as lock:
        try:
            fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
        except BlockingIOError:
            parser.exit(2,'Attached viewers already supervised.\n')
        try:
            for name,command in (
                ('attached_rviz',['rviz2','-d',str(ROOT/'polinasi_nav/config/navigation.rviz'),
                    '--ros-args','-p','use_sim_time:=true']),
                ('attached_camera',['ros2','run','rqt_image_view','rqt_image_view','/zed/image'])):
                handle = (args.run/(name+'.log')).open('w')
                handles.append(handle)
                children.append(subprocess.Popen(command,stdout=handle,stderr=subprocess.STDOUT,
                    start_new_session=True))
            print('Attached RViz and camera; viewers close when this registered run ends.',flush=True)
            while live(json.loads((args.run/'session.json').read_text())):
                if any(child.poll() is not None for child in children):
                    print('A viewer exited; see attached viewer logs. Simulation unchanged.',flush=True)
                    break
                time.sleep(1.)
        except KeyboardInterrupt:
            pass
        finally:
            for child in children:
                if child.poll() is None:
                    os.killpg(child.pid,signal.SIGINT)
            for child in children:
                try:
                    child.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    os.killpg(child.pid,signal.SIGTERM)
                    try:
                        child.wait(timeout=5)
                    except subprocess.TimeoutExpired:
                        os.killpg(child.pid,signal.SIGKILL)
                        child.wait()
            for handle in handles:
                handle.close()


if __name__=='__main__':
    main()
