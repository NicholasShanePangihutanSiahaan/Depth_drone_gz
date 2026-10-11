"""Compare completed unarmed A/B experiments; no ROS/flight commands."""
import argparse
import json
from pathlib import Path

def metrics(data):
    steps = data['steps']
    return dict(clouds=sum(s['clouds'] for s in steps),
        near_above_floor_points=sum(s['near_above_floor_points'] for s in steps),
        outside_body_filter_points=sum(s['outside_body_filter_points'] for s in steps),
        outside_filter_peak_per_scan=max(s['outside_filter_peak_per_scan'] for s in steps),
        near_occupied_voxel_peak=max(s['map']['near_occupied_voxels'] for s in steps),
        maximum_body_position_change_m=max(s['body_position_change_m'] for s in steps),
        final_mount_error_peak_rad={j:max(abs(s['measured'][j]-s['targets'][j]) for s in steps)
                                   for j in ('lidar_pitch','camera_yaw','camera_pitch')})

def main():
    p = argparse.ArgumentParser()
    p.add_argument('--visible',type=Path,default=Path('reports/empty_mount_visible_result.json'))
    p.add_argument('--hidden',type=Path,default=Path('reports/empty_mount_hidden_result.json'))
    p.add_argument('--output',type=Path,default=Path('reports/empty_mount_comparison.json'))
    args = p.parse_args()
    visible,hidden = [json.loads(path.read_text()) for path in (args.visible,args.hidden)]
    if not all(d.get('completed') and d['scope']=='unarmed_empty_arena_not_flight' for d in (visible,hidden)):
        raise ValueError('both unarmed experiments must complete')
    if visible['hide_self'] or not hidden['hide_self']:
        raise ValueError('A/B visibility flags do not match')
    if [s['targets'] for s in visible['steps']] != [s['targets'] for s in hidden['steps']]:
        raise ValueError('A/B commanded angle sequence differs')
    a,b = metrics(visible),metrics(hidden)
    isolated = a['outside_body_filter_points']>0 and a['near_occupied_voxel_peak']>0 and b['outside_body_filter_points']==0 and b['near_occupied_voxel_peak']==0
    result = dict(scope='unarmed_empty_arena_AB_not_airborne_or_hardware_validation',visible=a,hidden=b,
        self_visual_exclusion_eliminates_false_voxels=isolated,
        limitations=['GPU rendering artefact versus physical self-return not separated',
                     'rotating propellers and airborne body motion not tested',
                     'hidden variant omits self occlusion; not a real sensor filter',
                     'point counts include repeat observations, not unique surfaces',
                     'mounts may cross target tolerance transiently; final errors reported separately'],
        steps=[dict(targets=x['targets'],visible_peak_points=x['outside_filter_peak_per_scan'],
                    hidden_peak_points=y['outside_filter_peak_per_scan'],visible_voxels=x['map']['near_occupied_voxels'],
                    hidden_voxels=y['map']['near_occupied_voxels']) for x,y in zip(visible['steps'],hidden['steps'])])
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result,indent=2))

if __name__=='__main__': main()
