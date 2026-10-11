"""Opt-in test config. Original model and ordinary mission configs stay intact."""
import json
import math
from pathlib import Path
ROOT = Path(__file__).resolve().parents[1]
c = json.loads((ROOT/'polinasi_nav/config/tree_inspection.json').read_text())
c.update(sensor_gimbals_enabled=True,camera_pitch_enabled=True,camera_yaw_enabled=False,
         scan_body_faces_path=True,inspection_transit_camera=True,tree_view_count=6,
         mission_scope='experimental_gimbals_ground_truth_only',tree_demo_limit=2,
         tree_view_distances=[2.])
(ROOT/'polinasi_nav/config/tree_inspection_gimbal.json').write_text(json.dumps(c,indent=2)+'\n')
c.update(automatic_tilt_ascent=True,tree_approach_altitude=2.,tree_ascent_step=.4,
         tree_continuous_ascent=True,tree_ascent_speed=.12,tree_ascent_reaction_time=.7,
         tree_ascent_observation_guard=.5,
         tree_camera_observe_before_plan=True,tree_camera_observe_dwell=.4,
         tree_exploration_same_altitude=True,
         tree_tilt_angles=[math.radians(68)],tree_tilt_dwell=.8,tree_mount_timeout=20.,
         simulation_lidar_hide_self=False)
c.update(depth_camera_enabled=True,depth_camera_timeout=1.,depth_min_range=.3,
         depth_max_range=20.,depth_mapping_stride=8,depth_mapping_budget=800,
         depth_approach_depression_degrees=25.)
(ROOT/'polinasi_nav/config/tilt_inspection.json').write_text(json.dumps(c,indent=2)+'\n')
