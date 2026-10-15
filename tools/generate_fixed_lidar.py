"""Fixed top-mounted MID-360 and front-facing pitch-only camera fixtures."""
import json
from pathlib import Path
from generate_sensor_localisation import main as generate_sensor

ROOT = Path(__file__).resolve().parents[1]


def main():
    generate_sensor()
    output = ROOT / 'reports/fixed_lidar'
    output.mkdir(parents=True, exist_ok=True)
    for stage in ('hover', 'inspection'):
        c = json.loads((ROOT / f'reports/sensor_localisation/{stage}.json').read_text())
        c.update(sensor_rolling_localisation=False, sensor_fixed_localisation=True,
                 simulation_continuous_roll=False, simulation_roll_rate=0.,
                 lidar_fixed=True, fixed_fov_assumptions=True, automatic_tilt_ascent=False,
                 lidar_mount=[0., 0., .28, 0., 0., 0.],
                 camera_mount=[.20, 0., .06], camera_mount_yaw=0.,
                 camera_yaw_enabled=False, camera_pitch_enabled=True,
                 drone_dimensions=[.60, .60, .70],
                 simulation_balance_front_payload=False,
                 mission_scope='experimental_fixed_lidar_sensor_localisation_' + stage)
        # Replace the old moving-sensor envelopes; preserve chassis geometry.
        shapes = c['self_filter_shapes'][:-2]
        for mount, radius in ((c['lidar_mount'], .055), (c['camera_mount'], .095)):
            shapes.append(dict(kind='sphere', size=[radius], pose=[*mount[:3], 0., 0., 0.]))
        c['self_filter_shapes'] = shapes
        if stage == 'inspection':
            c.update(scan_adaptive_height=True,scan_target_repair=True,
                     scan_height_max_slope=.45,scan_height_hysteresis=.08,
                     scan_stem_top_offset=0.,scan_target_max_descent=.6,
                     scan_target_max_outward=1.,survey_batch_points=6,
                     tree_exploration_same_altitude=True,
                     exploration_directional_preference=True,exploration_progress_tolerance=.1,
                     exploration_backward_penalty=4.,exploration_retrace_penalty=2.,
                     depth_camera_enabled=True,depth_camera_timeout=1.,
                     depth_min_range=.3,depth_max_range=20.,
                     depth_mapping_stride=8,depth_mapping_budget=800,
                     depth_approach_depression_degrees=25.,
                     tree_camera_observe_before_plan=True,tree_camera_observe_dwell=.4,
                     fixed_scan_depth_observation=True)
        (output / f'{stage}.json').write_text(json.dumps(c, indent=2) + '\n')


if __name__ == '__main__':
    main()
