"""Stop ONLY a registered private simulation launcher and its owned GUIs."""
import argparse
import json
import os
from pathlib import Path
import signal
import time

ROOT = Path(__file__).resolve().parents[1]


def process_start(pid):
    return Path(f'/proc/{pid}/stat').read_text().rsplit(')', 1)[1].split()[19]


def directory(value):
    path = Path(value).resolve()
    if path.parent != ROOT/'.dependencies' or not path.name.startswith('ros_'):
        raise ValueError('not a private simulation run directory')
    return path


def live(data):
    try:
        pid = int(data['launcher_pid'])
        command = Path(f'/proc/{pid}/cmdline').read_bytes().replace(b'\0', b' ').decode()
        return (process_start(pid) == data['launcher_start_ticks']
                and str(ROOT/'tools/start_simulation.sh') in command
                and not data.get('stopped', False))
    except (FileNotFoundError, ProcessLookupError, KeyError):
        return False


def request_stop(path, data):
    if data.get('stop_requested'):
        print('Cleanup already requested; not interrupting launcher cleanup: '+data['run'])
        return
    data['stop_requested'] = True
    path.write_text(json.dumps(data, indent=2)+'\n')
    os.kill(int(data['launcher_pid']), signal.SIGTERM)
    print('Stop requested; launcher closes its Gazebo, RViz, ROS and SITL: '+data['run'])


def reap_orphan_sitl(run):
    """Exact registered run cwd + SITL executable; never signal real FCUs."""
    executable=(ROOT/'.dependencies/ardupilot/build/sitl/bin/arducopter').resolve()
    found=False
    for proc in Path('/proc').iterdir():
        if not proc.name.isdigit():continue
        try:
            if (proc/'cwd').resolve()!=run or (proc/'exe').resolve()!=executable:
                continue
            command=(proc/'cmdline').read_bytes().split(b'\0')
            if b'JSON' not in command or b'127.0.0.1' not in command:continue
            pid=int(proc.name);ticks=process_start(pid)
            os.kill(pid,signal.SIGTERM)
            for _ in range(25):
                time.sleep(.2)
                if not proc.exists() or process_start(pid)!=ticks:break
            else:
                os.kill(pid,signal.SIGKILL)
            print('Stopped orphan SITL for exact registered run: '+str(run))
            found=True
        except (FileNotFoundError,ProcessLookupError,PermissionError):
            continue
    return found


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--register', type=directory)
    parser.add_argument('--launcher', type=int)
    parser.add_argument('--mark-stopped', type=directory)
    parser.add_argument('--run', type=directory)
    args = parser.parse_args()
    if args.register:
        if args.launcher is None:
            parser.error('--register requires --launcher')
        data = dict(launcher_pid=args.launcher, launcher_start_ticks=process_start(args.launcher),
                    stopped=False, run=str(args.register), scope='simulation_only',
                    ros_domain_id=os.environ.get('ROS_DOMAIN_ID'),
                    gazebo_partition=os.environ.get('GZ_PARTITION'),
                    ros_log_dir=os.environ.get('ROS_LOG_DIR'),
                    ports={'gazebo_sitl_udp':9002,'mavros_udp':int(os.environ.get('POLINASI_MAVROS_UDP_PORT','14550')),'sitl_tcp':5762})
        if not live(data):
            raise RuntimeError('refusing to register a non-project launcher')
        (args.register/'session.json').write_text(json.dumps(data, indent=2)+'\n')
        return
    if args.mark_stopped:
        path = args.mark_stopped/'session.json'
        if path.exists():
            data = json.loads(path.read_text())
            data['stopped'] = True
            path.write_text(json.dumps(data, indent=2)+'\n')
        return
    paths = [args.run/'session.json'] if args.run else sorted(
        (ROOT/'.dependencies').glob('ros_*/session.json'), key=lambda p:p.stat().st_mtime, reverse=True)
    for path in paths:
        data = json.loads(path.read_text())
        if live(data):
            # Bash started with '&' inherits ignored SIGINT. SIGTERM still
            # invokes the launcher's TERM cleanup trap and owned-group cleanup.
            request_stop(path,data)
            return
        if args.run and data.get('run')==str(args.run) and reap_orphan_sitl(args.run):
            return
    print('No registered active project simulation found; no processes were signalled.')


if __name__ == '__main__':
    main()
