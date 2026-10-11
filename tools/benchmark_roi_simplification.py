"""Read-only geometric ROI benchmark; synthetic rays, not a flight replay."""
import argparse
import json
from pathlib import Path
import time
import numpy as np
from polinasi_nav.corridor_mapping import simplify_roi_route,sample_intersections,contains


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('watchdog',type=Path)
    parser.add_argument('--output',type=Path,required=True)
    args = parser.parse_args()
    last = json.loads(args.watchdog.read_text().splitlines()[-1])
    status = last.get('last_status',last)
    original = status['mapping_roi']['region']
    reduced = [simplify_roi_route(route,.05) for route in original['routes']]
    extra = max((e for _,e in reduced),default=0.)
    simplified = dict(original,routes=[r for r,_ in reduced],radius=original['radius']+extra)
    rng = np.random.default_rng(360)
    origin = np.asarray(original['centre'])
    directions = rng.normal(size=(1000,3))
    directions /= np.linalg.norm(directions,axis=1)[:,None]
    endpoints = origin+directions*rng.uniform(2.,20.,(1000,1))
    timings = {}
    samples = {}
    for name,roi in (('original',original),('simplified',simplified)):
        durations = []
        for _ in range(3):
            started = time.perf_counter()
            points,_ = sample_intersections(origin,endpoints,roi,.2/3)
            durations.append((time.perf_counter()-started)*1000)
        timings[name] = float(np.median(durations))
        samples[name] = points
    enclosed = bool(np.all(contains(samples['original'],simplified)))
    result = dict(scope='synthetic_ray_geometric_benchmark_using_logged_ROI_not_flight_validation',
        source=str(args.watchdog),seed=360,rays=1000,
        original_segments=sum(max(0,len(r)-1) for r in original['routes']),
        simplified_segments=sum(max(0,len(r)-1) for r in simplified['routes']),
        extra_radius_m=extra,original_samples_enclosed=enclosed,
        median_wall_ms=timings,ratio=timings['original']/timings['simplified'])
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result,indent=2))
    if not enclosed:
        raise RuntimeError('original_ROI_not_enclosed')


if __name__=='__main__':
    main()
