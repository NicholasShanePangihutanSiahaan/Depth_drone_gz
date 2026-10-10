"""Static fine-grid rebuild timing; NOT a ROS deadline/flight validation."""
import json
import sys
import time
from pathlib import Path
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'polinasi_nav'))
from polinasi_nav.config import load_config
from polinasi_nav.mapping import VoxelMap, FREE, OCCUPIED


def main():
    measurements = []
    for resolution in (.4, .2):
        c = load_config(ROOT/'polinasi_nav/config/palm_farm_navigation.json')
        c['resolution'] = resolution
        grid = VoxelMap(c)
        grid.state[:] = FREE
        grid.seen[:] = 0.
        grid.state[:, :, :3] = OCCUPIED
        grid.state[grid.shape[0]//2, :, :grid.shape[2]//2] = OCCUPIED
        durations = []
        for _ in range(3):
            start = time.perf_counter()
            grid.rebuild(0.)
            durations.append(time.perf_counter()-start)
        measurements.append({'resolution_m': resolution, 'cells': int(grid.state.size),
                             'rebuild_seconds': durations,
                             'median_rebuild_seconds': float(np.median(durations)),
                             'core_grid_arrays_bytes': sum(a.nbytes for a in
                                 (grid.state, grid.seen, grid.blocked, grid.blocked_actual, grid.distance))})
    report = {'scope': 'static map rebuild on this computer, not controller deadline or flight validation',
              'measurements': measurements}
    (ROOT/'reports/voxel_performance.json').write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
