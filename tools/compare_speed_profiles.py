"""Repeatable fixed-path comparison; not an ArduPilot flight test."""
import argparse
import json
import time
from pathlib import Path
import numpy as np
from polinasi_nav.config import load_config
from polinasi_nav.continuous import ContinuousTrajectory


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', type=Path, default=Path('reports/local_speed_profile_benchmark.json'))
    args = parser.parse_args()
    config = load_config(Path(__file__).resolve().parents[1]/'polinasi_nav/config/predictive_tour_check.json')
    paths = dict(straight=[[0,0,2],[2,0,2],[4,0,2],[6,0,2]],
        short_corner=[[0,0,2],[2,0,2],[4,0,2],[4.2,0,2],[4.2,.2,2],[6,.2,2],[8,.2,2]],
        reversal=[[0,0,2],[2,0,2],[4,0,2],[2,0,2]])
    results = {}
    for name, points in paths.items():
        results[name] = {}
        for enabled in (False, True):
            times = []
            for _ in range(5):
                started = time.perf_counter()
                trajectory = ContinuousTrajectory(points, dict(config, local_speed_profile=enabled))
                times.append((time.perf_counter()-started)*1000.)
            speeds = [float(np.linalg.norm(trajectory.sample(t)[1]))
                      for t in np.linspace(0., trajectory.duration, 1000)]
            results[name]['local' if enabled else 'legacy'] = dict(
                duration_sim_seconds=trajectory.duration, durations=trajectory.durations.tolist(),
                mean_reference_speed_mps=float(np.mean(speeds)), max_reference_speed_mps=max(speeds),
                construction_median_wall_ms=float(np.median(times)),
                derivative_bounds=trajectory.bounds().tolist())
    report = dict(scope='fixed-path numerical reference comparison; no flight physics',
                  command_speed_limit_mps=config['command_speed'], paths=results)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
