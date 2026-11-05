"""Small reports and strict JSON; CSV retains the per-sector support counts."""
import csv
import json
import os
from pathlib import Path


def atomic_json(path, data):
    path = Path(path)
    temporary = path.with_suffix(path.suffix+'.tmp')
    temporary.write_text(json.dumps(data, indent=2, allow_nan=False)+'\n', encoding='utf8')
    os.replace(temporary, path)


def format_line(elapsed, distances, estimate, missing, status):
    parts = []
    for direction in ('front', 'right', 'left', 'back'):
        value = distances.get(direction, {}).get('distance_cm')
        parts.append(f'{direction} = {value:.1f} cm' if value is not None else f'{direction} = N/A')
    yaw = f"{estimate['rpy_deg'][2]:.2f} deg" if estimate.get('valid') else 'N/A'
    speed = f"{estimate['speed_mps']:.3f} m/s" if estimate.get('valid') else 'N/A'
    return (f'Time = {elapsed:.3f} s | Smallest distance: '+', '.join(parts)+
            f' | Yaw_est = {yaw} | Speed_est = {speed} | Missing = {",".join(missing) or "none"}'
            f' | Status = {status}')


class ReadingFiles:
    def __init__(self, directory):
        self.directory = Path(directory)
        self.directory.mkdir(parents=True, exist_ok=True)
        self.text = (self.directory/'readings.log').open('x', buffering=1)
        self.file = (self.directory/'readings.csv').open('x', newline='', buffering=1)
        self.writer = csv.writer(self.file)
        self.writer.writerow(['elapsed_s', 'cloud_stamp_s', 'imu_stamp_s', 'imu_sync_offset_s',
            'front_cm', 'right_cm', 'left_cm', 'back_cm',
            'front_points', 'right_points', 'left_points', 'back_points',
            'roll_est_deg', 'pitch_est_deg', 'yaw_est_deg', 'vx_est_mps', 'vy_est_mps',
            'vz_est_mps', 'speed_est_mps', 'integration_s', 'imu_state', 'missing', 'status'])

    def write(self, elapsed, cloud, estimate, status):
        sectors = cloud.get('sectors', {})
        missing = cloud.get('missing', ['front', 'right', 'left', 'back'])
        line = format_line(elapsed, sectors, estimate, missing, status)
        self.text.write(line+'\n')
        valid = estimate.get('valid', False)
        self.writer.writerow([elapsed, cloud.get('stamp'), estimate.get('stamp'),
            estimate.get('sync_offset_s'),
            *[sectors.get(k, {}).get('distance_cm') for k in ('front', 'right', 'left', 'back')],
            *[sectors.get(k, {}).get('points', 0) for k in ('front', 'right', 'left', 'back')],
            *(estimate['rpy_deg'] if valid else [None]*3),
            *(estimate['velocity_mps'] if valid else [None]*3),
            estimate.get('speed_mps') if valid else None, estimate.get('integration_seconds'),
            estimate.get('state', 'NO_IMU'), ','.join(missing), status])
        return line

    def close(self):
        self.text.close()
        self.file.close()
