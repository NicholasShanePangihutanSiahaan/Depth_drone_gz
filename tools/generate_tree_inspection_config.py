"""Simulation-only measured-tree mission; no tree positions in flight config."""
import json
from pathlib import Path
ROOT = Path(__file__).resolve().parents[1]
c = json.loads((ROOT/'polinasi_nav/config/pid_tour.json').read_text())
for name in ('coverage_tree_centres','trees','nominal_route_policy'):
    c.pop(name,None)
c.update(active_tree_inspection=True, tree_demo_limit=0, tree_search_wait=10.,
         tree_view_count=8, tree_view_distances=[3.5,4.5,5.5],
         survey_waypoints=[[0.,0.,2.]], survey_prefetch=False, survey_dwell=.5,
         mock_flower_events=False, camera_pitch_enabled=False,
         survey_max_duration=1800., mission_scope='all_confirmed_visible_candidates_not_proven_entire_farm',
         command_speed=.3, map_log_max_points=1000000)
(ROOT/'polinasi_nav/config/tree_inspection.json').write_text(json.dumps(c,indent=2)+'\n')
