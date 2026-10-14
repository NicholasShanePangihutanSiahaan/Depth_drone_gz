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
                 lidar_fixed=True, automatic_tilt_ascent=False,
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
        (output / f'{stage}.json').write_text(json.dumps(c, indent=2) + '\n')


if __name__ == '__main__':
    main()
