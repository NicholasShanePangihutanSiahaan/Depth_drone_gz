"""Offline reference/corridor/controller evaluation. Never launches ROS or flight."""
import argparse
import json
import time
from pathlib import Path
import numpy as np
from polinasi_nav.config import load_config
from polinasi_nav.mapping import VoxelMap,FREE,OCCUPIED,UNKNOWN
from polinasi_nav.continuous import ContinuousTrajectory
from polinasi_nav.survey import SurveyController
from polinasi_nav.super_policy import reference_speed,voxel_corridor,explicit_backup
from polinasi_nav.adaptive_model import load_validated_seed


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--config',type=Path,default=Path('reports/fixed_lidar/inspection.json'))
    parser.add_argument('--output',type=Path,default=Path('reports/super_adaptation_evaluation.json'))
    args=parser.parse_args();config=load_config(args.config)
    c=dict(config,bounds_min=[-4.,-4.,0.],bounds_max=[8.,8.,7.],navigation_window_size=[12.,12.,7.],
           free_ttl=10000.,survey_waypoints=[[4.,0.,2.]],history_route_enabled=False)
    cases=dict(straight=[[0.,0.,2.],[4.,0.,2.]],
               turn=[[0.,0.,2.],[2.,0.,2.],[2.,2.,2.],[4.,2.,2.]],
               vertical_detour=[[0.,0.,2.],[1.,0.,2.],[1.,0.,3.],[3.,0.,3.],[3.,0.,2.],[4.,0.,2.]])
    results={}
    for name,points in cases.items():
        grid=VoxelMap(c);grid.state[:]=FREE;grid.seen[:]=0.;grid.rebuild(0.)
        ctl=SurveyController(c,grid,asynchronous=False);ctl.state='SURVEY'
        # Same selected route, same explicit uncertainty and physical limits.
        legacy=ContinuousTrajectory(points,dict(c,duration_optimisation=False,local_speed_profile=False))
        started=time.perf_counter();trajectory=ctl._trajectory_from_points(points)
        build_ms=(time.perf_counter()-started)*1000
        corridor_started=time.perf_counter()
        boxes=[voxel_corridor(grid,p) for p in trajectory.controls]
        corridor_ms=(time.perf_counter()-corridor_started)*1000
        speeds=[np.linalg.norm(trajectory.sample(t)[1]) for t in np.linspace(0.,trajectory.duration,500)]
        p,v,a=trajectory.sample(trajectory.duration*.5)
        backup,backup_detail=explicit_backup(grid,p,v,a,c,optimise=True)
        results[name]=dict(legacy_duration_seconds=legacy.duration,
            new_duration_seconds=trajectory.duration,reference_time_reduction_fraction=1-trajectory.duration/legacy.duration,
            reference_max_speed_mps=float(max(speeds)),derivative_bounds=trajectory.bounds().tolist(),
            construction_wall_ms=build_ms,curve_safe=bool(trajectory.safe(grid)),
            voxel_convex_boxes=len([b for b in boxes if b is not None]),segments=len(boxes),
            corridor_wall_ms=corridor_ms,backup=backup_detail,
            duration_search=trajectory.duration_optimisation)
    model=Path(__file__).resolve().parents[1]/'polinasi_nav/config/identified_pva_model.json'
    try:
        load_validated_seed(str(model),config)
        model_compatibility='accepted_by_existing_envelope_check_not_a_fixed_sensor_flight_validation'
    except ValueError as exc:model_compatibility=str(exc)
    report=dict(scope='offline_synthetic_reference_and_voxel_corridor_evaluation_not_flight_or_Jetson_benchmark',
        configuration=str(args.config),physical_speed_limit_mps=config['speed'],
        reference_speed_cap_mps=reference_speed(config),velocity_uncertainty_mps=config['velocity_uncertainty_mps'],
        cases=results,
        CIRI=dict(implemented=False,evaluated=True,
            source='https://github.com/hku-mars/SUPER/blob/master/super_planner/src/super_core/ciri.cpp',
            retained_backend='inflated_three_state_voxel_map_with_convex_box_acceleration_and_recursive_curve_certificate',
            reasons=['CIRI point inputs alone cannot preserve this project unknown/FOV-prior semantics',
                     'CIRI spherical body model needs an explicit conservative envelope for this rectangular airframe',
                     'Full FOV/extrinsic/visibility certification must precede any replacement of voxel collision gates'],
            benchmark_limit='Only voxel baseline measured; no claim of measured runtime comparison against CIRI'),
        controller=dict(active='PID_PVA_feedback',original_SUPER_MPC_imported=False,
            existing_identified_model_check=model_compatibility,
            decision='retain PID pending fixed-configuration tracking/latency flight evidence; do not bypass MPC model checks'),
        not_implemented=['MINCO joint spatial-time optimization','original SUPER on-manifold MPC','CIRI replacement of map safety gates'])
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report,indent=2))


if __name__=='__main__':main()
