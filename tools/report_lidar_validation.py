"""Summarise matched before/after sensor replays, not flight validation."""
import argparse
import json
from pathlib import Path
import numpy as np

def metrics(d):
    steps = d['steps']
    return dict(completed=d['completed'],clouds=sum(s['clouds'] for s in steps),
        outside_filter_points=sum(s['outside_body_filter_points'] for s in steps),
        false_near_voxel_peak=max(s['map']['near_occupied_voxels'] for s in steps),
        above_floor_voxel_peak=max(s['map']['above_floor_occupied_voxels'] for s in steps) if all('above_floor_occupied_voxels' in s['map'] for s in steps) else None,
        floor_voxel_peak=max(s['map']['floor_occupied_voxels'] for s in steps) if all('floor_occupied_voxels' in s['map'] for s in steps) else None,
        fixture_surface_points=sum(s.get('fixture_surface_points',0) for s in steps),
        fixture_voxel_peak=max(s['map'].get('fixture_occupied_voxels',0) for s in steps))

def main():
    p = argparse.ArgumentParser()
    p.add_argument('--output',type=Path,default=Path('reports/lidar_sampler_validation.json'))
    p.add_argument('--fixture-map',type=Path)
    args = p.parse_args()
    result = dict(scope='ground_truth_unarmed_Gazebo_sensor_regression_not_hardware_or_flight',comparisons={},complete=False,
                  upstream='https://github.com/gazebosim/gz-rendering/pull/1319')
    for variant in ('visible','hidden'):
        baseline = json.loads(Path(f'reports/empty_mount_{variant}_result.json').read_text())
        fixed_path = Path(f'reports/empty_mount_{variant}_sampler_fixed.json')
        if not fixed_path.exists(): continue
        fixed = json.loads(fixed_path.read_text())
        if baseline['hide_self']!=fixed['hide_self'] or [s['targets'] for s in baseline['steps']] != [s['targets'] for s in fixed['steps']]:
            raise ValueError('before/after test conditions differ')
        a,b = metrics(baseline),metrics(fixed)
        passed = a['completed'] and b['completed'] and a['false_near_voxel_peak']>0 and b['outside_filter_points']==0 and b['above_floor_voxel_peak']==0 and b['floor_voxel_peak']>0
        result['comparisons'][variant] = dict(before=a,after=b,passed=bool(passed))
    fixture_path = Path('reports/lidar_sampler_fixture.json')
    if fixture_path.exists():
        fixture = metrics(json.loads(fixture_path.read_text()))
        result['obstacle_preservation'] = dict(measurements=fixture,passed=bool(fixture['completed'] and fixture['fixture_surface_points']>0 and fixture['fixture_voxel_peak']>0 and fixture['false_near_voxel_peak']==0))
        if args.fixture_map:
            data = json.loads(args.fixture_map.read_text())
            config = json.loads(Path('polinasi_nav/config/empty_mount_fixture.json').read_text())['simulation_lidar_validation_box']
            points = np.asarray(data['origin'])+(np.asarray(data['occupied'])+.5)*data['resolution']
            inside = np.all(np.abs(points-np.asarray(config['centre']))<=np.asarray(config['size'])/2+data['resolution'],axis=1)
            unexpected = int(((points[:,2]>.15)&~inside).sum())
            result['obstacle_preservation']['global_map'] = dict(path=str(args.fixture_map),unexpected_non_floor_voxels=unexpected,
                fixture_voxels=int(inside.sum()),floor_voxels=int((points[:,2]<=.15).sum()))
            result['obstacle_preservation']['passed'] &= unexpected==0 and bool(inside.any())
        else:
            result['obstacle_preservation']['passed'] = False  # Whole-map check is required.
    result['complete'] = bool(len(result['comparisons'])==2 and all(c['passed'] for c in result['comparisons'].values()) and result.get('obstacle_preservation',{}).get('passed'))
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result,indent=2))

if __name__=='__main__': main()
