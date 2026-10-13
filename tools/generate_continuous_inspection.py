"""Opt-in continuous scanning configuration; do not overwrite discrete mission."""
import json
from pathlib import Path
from generate_visual50_hover import main as generate_visual
ROOT=Path(__file__).resolve().parents[1]


def main():
    generate_visual()
    c=json.loads((ROOT/'reports/visual50_hover/config.json').read_text())
    c.update(active_tree_inspection=True,continuous_tree_scanning=True,
             continuous_survey=True,tree_view_count=6,tree_scan_anchors=24,
             tree_demo_limit=2,tree_view_distances=[2.],tree_approach_altitude=2.,
             survey_batch_points=24,survey_pass_radius=.35,survey_dwell=.05,
             survey_prefetch=False,inspection_transit_camera=False,
             tree_camera_observe_before_plan=False,
             rolling_canopy_ascent=True,tree_continuous_ascent=True,
             tree_ascent_speed=.12,tree_ascent_reaction_time=.7,
             tree_ascent_max_map_age=.4,tree_canopy_gap=.6,
             tree_ascent_observation_guard=max(0.,.6-c['clearance']-c['tracking_margin']),
             mission_scope='experimental_continuous_two_tree_GT_only',survey_max_duration=2400.)
    output=ROOT/'reports/continuous_inspection';output.mkdir(parents=True,exist_ok=True)
    (output/'config.json').write_text(json.dumps(c,indent=2)+'\n')


if __name__=='__main__':main()
