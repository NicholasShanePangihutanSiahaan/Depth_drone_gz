"""Bounded simulation-only runner. Never edits code or kills other runs."""
import argparse
import fcntl
import json
import os
from pathlib import Path
import signal
import subprocess
import time

from stop_simulation import live

ROOT = Path(__file__).resolve().parents[1]


def audit(path):
    """Summarise watchdog evidence; absence of failure is NOT mission completion."""
    count = 0
    last = {}
    failures = []
    max_speed = 0.0
    max_tracking = 0.0
    if path.exists():
        for line in path.read_text().splitlines():
            try:
                status = json.loads(line)
            except json.JSONDecodeError:
                failures.append('malformed_watchdog_log')
                continue
            if 'watchdog_fault' in status:
                failures.append(status['watchdog_fault'])
                last = status.get('last_status', last)
                continue
            last = status
            count += 1
            if status.get('failure') and status['failure'] not in ('route_changed', 'terrain_replan_required'):
                failures.append(status['failure'])
            max_speed = max(max_speed, status.get('measured_speed_mps', 0.0))
            max_tracking = max(max_tracking, status.get('tracking_error_m', 0.0))
    trees = last.get('inspected_tree_ids', [])
    records = last.get('tree_scan_records', [])
    views = {str(tree): len({r.get('view') for r in records if r.get('tree_id')==tree}) for tree in trees}
    # ROS flight_stage stays MISSION; mission COMPLETE is set only after
    # the adapter confirms touchdown and disarming.
    landing = last.get('armed') is False and last.get('mission') == 'COMPLETE'
    return dict(completed=last.get('mission') == 'COMPLETE' and not failures
                    and len(set(trees))>=2 and all(v>=6 for v in views.values()) and landing,
                inspected_tree_ids=trees, unique_views_per_tree=views, landing_confirmed=landing,
                failures=sorted(set(failures)), status_events=count,
                last_phase=last.get('tree_phase'), last_mission=last.get('mission'),
                event_sample_max_speed_mps=max_speed,
                event_sample_max_tracking_error_m=max_tracking,
                measurement_scope='phase-change samples, not full-flight extrema',
                localisation='ground_truth; not LiDAR localisation validation')


def positive_seconds(value):
    result = int(value)
    if not 60 <= result <= 14400:
        raise argparse.ArgumentTypeError('timeout must be 60..14400 wall seconds')
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--runs', type=int, choices=range(1, 11), default=1)
    parser.add_argument('--wall-timeout', type=positive_seconds, default=7200)
    parser.add_argument('--adapter-tests-only',action='store_true',
        help='Run fixed ROS adapter tests in isolated domain 199, without flight commands')
    args = parser.parse_args()
    if args.adapter_tests_only:
        env = dict(os.environ,ROS_DOMAIN_ID='199',ROS_LOG_DIR=str(ROOT/'reports/ros_adapter_test_logs'))
        Path(env['ROS_LOG_DIR']).mkdir(exist_ok=True)
        return subprocess.run(['/usr/bin/python3','-m','pytest','-q',
            'polinasi_nav/test/test_ros_adapters.py','polinasi_nav/test/test_sensor_mount_ros.py'],
            cwd=ROOT,env=env,check=False).returncode
    dependencies = ROOT / '.dependencies'
    dependencies.mkdir(exist_ok=True)
    with (dependencies / 'inspection_loop.lock').open('w') as lock:
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            parser.exit(2, 'Another inspection loop is active.\n')
        for session in dependencies.glob('ros_*/session.json'):
            if live(json.loads(session.read_text())):
                parser.exit(2, 'Existing simulation active; stop it first. No process was signalled.\n')
        folder = ROOT / 'reports' / ('inspection_loop_' + time.strftime('%Y%m%d_%H%M%S'))
        folder.mkdir(exist_ok=False)
        print('Loop logs: ' + str(folder), flush=True)

        def step(command, name, timeout=180):
            with (folder / (name + '.log')).open('w') as output:
                subprocess.run(command, cwd=ROOT, stdout=output, stderr=subprocess.STDOUT,
                               timeout=timeout, check=True)

        step(['/usr/bin/python3', 'tools/generate_sensor_gimbal_config.py'], 'config')
        step(['colcon', '--log-base', 'log_ros', 'build', '--symlink-install',
              '--build-base', 'build_ros', '--install-base', 'install_ros',
              '--base-paths', 'polinasi_nav', 'beehive_drone', 'uav_interfaces',
              '--packages-select', 'polinasi_nav'], 'build', 600)
        for number in range(1, args.runs + 1):
            name = 'run_' + str(number)
            run = None
            observer = None
            error = None
            watchdog = folder / (name + '_watchdog.jsonl')
            with (folder / (name + '_launcher.log')).open('w') as output:
                launcher = subprocess.Popen(['bash', str(ROOT / 'tools/start_simulation.sh'),
                    'ground_truth', 'palm_farm', 'mapping',
                    str(ROOT / 'polinasi_nav/config/tilt_inspection.json')],
                    cwd=ROOT, stdout=output, stderr=subprocess.STDOUT)
                try:
                    deadline = time.monotonic() + 120
                    while run is None:
                        if launcher.poll() is not None:
                            raise RuntimeError('launcher_exited_before_registration')
                        for session in dependencies.glob('ros_*/session.json'):
                            data = json.loads(session.read_text())
                            if data.get('launcher_pid') == launcher.pid and live(data):
                                run = session.parent
                                break
                        if time.monotonic() > deadline:
                            raise RuntimeError('launcher_registration_timeout')
                        time.sleep(.5)
                    print('Registered simulation: ' + run.name, flush=True)
                    step([str(ROOT / '.venv/bin/python'), 'tools/set_origin.py', '--timeout', '120'], name + '_origin', 150)
                    setup = folder / (name + '_setup.json')
                    step(['/usr/bin/python3', 'tools/check_ros_simulation.py', '--output', str(setup)], name + '_setup', 120)
                    if not json.loads(setup.read_text()).get('passed'):
                        raise RuntimeError('setup_checks_failed')
                    with (folder / (name + '_observer.log')).open('w') as observation:
                        observer = subprocess.Popen(['/usr/bin/python3', 'tools/watch_tree_inspection.py',
                            '--run', str(run), '--output', str(watchdog),
                            '--seconds', str(args.wall_timeout)], cwd=ROOT,
                            stdout=observation, stderr=subprocess.STDOUT)
                        step(['/usr/bin/python3', 'tools/start_mission.py', '--timeout', '60'], name + '_start', 90)
                        observer.wait(timeout=args.wall_timeout + 10)
                    if observer.returncode:
                        raise RuntimeError('watchdog_process_failed')
                except (Exception, KeyboardInterrupt) as exc:
                    error = type(exc).__name__ + ': ' + str(exc)
                finally:
                    # Signal only the launcher we created; its trap owns child cleanup.
                    if launcher.poll() is None:
                        launcher.send_signal(signal.SIGINT)
                        try:
                            launcher.wait(timeout=30)
                        except subprocess.TimeoutExpired:
                            error = 'owned_launcher_cleanup_timeout; manual inspection required'
                    if observer is not None and observer.poll() is None:
                        observer.terminate()
                        try:
                            observer.wait(timeout=10)
                        except subprocess.TimeoutExpired:
                            observer.kill()
                            observer.wait()
            result = audit(watchdog)
            result.update(run=str(run) if run else None, runner_error=error,
                map_html=str(ROOT / 'reports' / run.name / 'map_3d/map.html') if run else None,
                headless=os.environ.get('POLINASI_SIM_GUI','1')=='0',
                scope='configured demo tree count, not entire farm')
            (folder / (name + '_audit.json')).write_text(json.dumps(result, indent=2) + '\n')
            print(json.dumps(result), flush=True)
            if error or not result['completed']:
                print('Stopped. Audit and fix code before restarting; no blind retry.', flush=True)
                return 1
        return 0


if __name__ == '__main__':
    raise SystemExit(main())
