"""Read-only measured active-MPC flight summary; no fitting or FCU writes."""
import argparse
import json
from pathlib import Path
import numpy as np


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('dataset', type=Path)
    parser.add_argument('--output', required=True, type=Path)
    args = parser.parse_args()
    if args.output.exists():
        raise ValueError('refusing to overwrite summary')
    data = json.loads(args.dataset.read_text())
    status, config = data['final_status'], data['config']
    commands = data['commands']
    times = np.array([r['t'] for r in commands])
    a, v = (np.array([r[key] for r in commands]) for key in ('a', 'v'))
    dt = np.diff(times)
    if len(commands) < 2 or np.any(dt <= 0):
        raise ValueError('insufficient/nonmonotonic command data')
    error, positions = [], []
    for response in data['responses']:
        index = np.searchsorted(times, response['t'], side='right')-1
        if (not response['healthy'] or response['phase'] == 'LAND' or index < 0
                or response['t']-times[index] > .12):
            continue
        positions.append(response['p'])
        error.append(np.asarray(response['p'])-commands[index]['p'])
    error = np.linalg.norm(error, axis=1)
    metrics = dict(tracking_max_m=float(error.max()), tracking_rms_m=float(np.sqrt(np.mean(error**2))),
        commanded_speed_max_mps=float(np.linalg.norm(v, axis=1).max()),
        commanded_acceleration_max_mps2=float(np.linalg.norm(a, axis=1).max()),
        commanded_jerk_max_mps3=float((np.linalg.norm(np.diff(a, axis=0), axis=1)/dt).max()),
        tracking_samples=len(error), published_commands=len(commands),
        predictive_applied=status.get('predictive_applied', 0),
        worker_wall_max_ms=status.get('predictive_worker_wall_max_ms'),
        control_wall_max_ms=status['callback_wall_max_ms']['control_tick'])
    checks = dict(completed=data['completed'] and status['mission'] == 'COMPLETE' and not status['armed'],
        single_final_publisher=data['one_final_publisher'],
        active_model_used=data['model_controls_flight'] and metrics['predictive_applied'] > 0,
        no_failure=not status['failure'], no_timing_gap=status['last_timing_gap_seconds'] is None,
        tracking_bounded=metrics['tracking_max_m'] <= config['tracking_limit'],
        command_speed_bounded=metrics['commanded_speed_max_mps'] <= config['command_speed']+1e-5,
        command_acceleration_bounded=metrics['commanded_acceleration_max_mps2'] <= config['acceleration']+1e-5,
        command_jerk_bounded=metrics['commanded_jerk_max_mps3'] <= config['jerk']+1e-5)
    report = dict(dataset=str(args.dataset), scope='small-motion ground-truth empty-arena MPC test',
        checks=checks, passed=all(checks.values()), metrics=metrics,
        full_nmpc_implemented=False, adaptive_flight_validated=False,
        orchard_flight_validated=False, hardware_validated=False,
        note='Tracking uses the latest command available at each response stamp (zero-order hold). '
             'No paired A/B test: changed preflight alignment/pacing prevents attributing improvements to MPC.')
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
