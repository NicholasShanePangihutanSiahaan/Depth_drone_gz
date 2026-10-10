"""Synthetic mapping-only comparison: no commands, physics or localisation test."""
import argparse
import json
from pathlib import Path
import statistics
import time
import numpy as np
from polinasi_nav.config import load_config
from polinasi_nav.corridor_mapping import CorridorRegion
from polinasi_nav.coverage_mapping import SparseHistory
from polinasi_nav.simulated_rays import directions


def main():
    p = argparse.ArgumentParser()
    root = Path(__file__).resolve().parents[1]
    p.add_argument('--config', type=Path, default=root/'polinasi_nav/config/predictive_tour_check.json')
    p.add_argument('--output', type=Path, default=root/'reports/corridor_benchmark.json')
    args = p.parse_args()
    c = load_config(args.config)
    origin = np.array([0., 0., 2.])
    points = origin+directions()[::8]*c['lidar_range']
    roi = CorridorRegion(c).region(origin, {}, c['survey_waypoints'], [], 1.)
    result = dict(scope='synthetic valid max-range rays; integration only, not flight validation',
                  rays=len(points), resolution_m=c['resolution'], results={})
    for name, region in (('whole_range', None), ('clipped_corridor', roi)):
        timings = []
        for _ in range(5):
            h = SparseHistory(c)
            start = time.perf_counter()
            h.integrate(origin, points, np.zeros(len(points), bool), 1., region)
            timings.append(time.perf_counter()-start)
        result['results'][name] = dict(median_seconds=statistics.median(timings),
            timings_seconds=timings, known_voxels=len(h.cells), stats=h.last_integration_stats)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2)+'\n')
    print(json.dumps({k:dict(median_ms=v['median_seconds']*1000, known_voxels=v['known_voxels'])
                      for k, v in result['results'].items()}, indent=2))


if __name__ == '__main__':
    main()
