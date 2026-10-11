"""Read-only replay of intermediate-goal selection on a saved voxel map."""
import argparse
import json
from pathlib import Path
import sys
import time
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'polinasi_nav'))
from polinasi_nav.config import load_config
from polinasi_nav.coverage_mapping import navigation_window
from polinasi_nav.local_subgoal import inspection_hint
from polinasi_nav.mapping import FREE,OCCUPIED


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('map',type=Path)
    args = parser.parse_args()
    data = json.loads(args.map.read_text())
    status = data['mission_status']
    if 'actual_position' not in status.get('pid',{}):
        rows = (args.map.parent/'navigation_debug.jsonl').read_text().splitlines()
        status = next(s for s in (json.loads(line) for line in reversed(rows))
            if 'actual_position' in s.get('pid',{}))
    position = np.asarray(status['pid']['actual_position'])
    target = status['inspection_goal_hint']
    c = load_config(ROOT/'polinasi_nav/config/tilt_inspection.json')
    grid = navigation_window(c,position)
    stamp = data['map_observation_time']
    for key,value in (('free',FREE),('occupied',OCCUPIED)):
        points = np.asarray(data['origin'])+(np.asarray(data[key])+.5)*data['resolution']
        indices = grid.indices(points)
        indices = indices[grid.inside(indices)]
        grid.state[tuple(indices.T)] = value
        grid.seen[tuple(indices.T)] = stamp
    grid.last_observation_stamp = stamp
    grid.rebuild(stamp)
    started = time.perf_counter()
    selected,detail = inspection_hint(grid,position,target,status['tree_phase']=='APPROACH')
    detail.update(wall_ms=(time.perf_counter()-started)*1000,position=position.tolist(),
        selected_safe=grid.safe(selected),selected_segment_safe=grid.line_safe(position,selected),
        replay_scope='saved occupancy only, not flight validation')
    print(json.dumps(detail,indent=2))


if __name__=='__main__':
    main()
