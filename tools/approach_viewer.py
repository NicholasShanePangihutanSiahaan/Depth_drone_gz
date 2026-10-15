"""Open one owned RViz process after the first observed approach completes."""
import os
import signal
import subprocess
from pathlib import Path


class ApproachViewer:
    def __init__(self, run, enabled=False):
        self.run = Path(run) if run else None
        self.enabled = enabled
        self.approach_seen = self.attempted = False
        self.process = self.stream = None
        self.events = []

    def update(self, navigation):
        if not self.enabled or self.attempted:
            return
        phase = navigation.get('tree_phase')
        if phase == 'APPROACH':
            self.approach_seen = True
        if (not self.approach_seen or phase not in ('ASCEND', 'NORMALIZE', 'SCAN')
                or navigation.get('failure')):
            return
        self.attempted = True
        event = dict(event='rviz_after_approach', stamp=navigation.get('simulation_time'), phase=phase)
        try:
            if self.run is None:
                raise ValueError('owned run directory required')
            if not (os.environ.get('DISPLAY') or os.environ.get('WAYLAND_DISPLAY')):
                raise RuntimeError('desktop display unavailable')
            self.stream = (self.run/'rviz_after_approach.log').open('w')
            config = Path(__file__).resolve().parents[1]/'polinasi_nav/config/navigation.rviz'
            environment=dict(os.environ,ROS_LOG_DIR=str(self.run/'rviz_ros_logs'))
            self.process = subprocess.Popen(['rviz2','-d',str(config),'--ros-args','-p','use_sim_time:=true'],
                stdout=self.stream,stderr=subprocess.STDOUT,start_new_session=True,env=environment)
            event.update(result='spawned',pid=self.process.pid)
        except (OSError,ValueError,RuntimeError) as exc:
            event.update(result='failed',reason=str(exc))
        self.events.append(event)
        print(event,flush=True)

    def close(self):
        if self.process is not None and self.process.poll() is None:
            for sig in (signal.SIGINT,signal.SIGTERM,signal.SIGKILL):
                try:
                    os.killpg(self.process.pid,sig)
                    self.process.wait(timeout=2.)
                    break
                except ProcessLookupError:
                    break
                except subprocess.TimeoutExpired:
                    continue
        if self.stream is not None:
            self.stream.close()
