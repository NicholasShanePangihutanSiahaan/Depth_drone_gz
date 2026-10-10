"""Stop ONLY a registered private simulation launcher and its owned GUIs."""
import argparse
import json
import os
from pathlib import Path
import signal

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
                    stopped=False, run=str(args.register), scope='simulation_only')
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
            os.kill(int(data['launcher_pid']), signal.SIGINT)
            print('Stop requested; launcher closes its Gazebo, RViz, ROS and SITL: '+data['run'])
            return
    print('No registered active project simulation found; no processes were signalled.')


if __name__ == '__main__':
    main()
