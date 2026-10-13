"""Separate continuous-roll hover configuration; no inspection mission edits."""
import json
import argparse
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def generate(rate=.4):
    c = json.loads((ROOT/'polinasi_nav/config/empty_mount_visible.json').read_text())
    c.update(sensor_gimbals_enabled=True,simulation_continuous_roll=True,
             simulation_roll_rate=rate,lidar_mount=[.65,0.,.45,0.,0.,0.],
             simulation_balance_front_payload=True,
             camera_yaw_enabled=False,depth_camera_enabled=False,
             active_tree_inspection=False,automatic_tilt_ascent=False,
             mapping_corridor_enabled=True,mapping_near_radius=6.,
             mapping_corridor_radius=3.,mapping_corridor_max_radius=4.,
             mapping_stride=6,mapping_no_return_stride=18,
             continuous_survey=False,
             survey_waypoints=[[0.,0.,2.]],survey_dwell=600.,
             bounds_min=[-40.,-40.,-.6],bounds_max=[52.,42.,12.],
             lidar_range=20.,mission_scope='simulation_only_rolling_lidar_takeoff_hover_return_land',
             simulation_lidar_hide_self=False)
    output = ROOT/'reports/rolling_hover'
    output.mkdir(parents=True,exist_ok=True)
    (output/'config.json').write_text(json.dumps(c,indent=2)+'\n')


if __name__ == '__main__':
    p = argparse.ArgumentParser()
    p.add_argument('--rate',type=float,default=.4)
    args = p.parse_args()
    generate(args.rate)
