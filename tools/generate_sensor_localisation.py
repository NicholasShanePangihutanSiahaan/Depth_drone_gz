"""Independent sensor-navigation fixtures; GT remains audit-only."""
import json
from pathlib import Path
from generate_continuous_inspection import main as generate_continuous
ROOT=Path(__file__).resolve().parents[1]


def main():
    generate_continuous()
    c=json.loads((ROOT/'reports/continuous_inspection/config.json').read_text())
    c.update(sensor_rolling_localisation=True,mission_scope='experimental_rolling_sensor_localisation_not_FAST_LIO')
    c.update(smooth_approach_ascent=True,approach_blend_distance=2.,
             approach_blend_rise=.6,approach_blend_terminal_speed=.08,
             approach_blend_max_descent=.4)
    c.update(scan_target_repair=True,scan_target_max_outward=1.,scan_target_max_descent=.6)
    c.update(scan_adaptive_height=True,scan_height_max_slope=.45,
             scan_height_hysteresis=.08,scan_stem_top_offset=0.,survey_batch_points=6)
    folder=ROOT/'reports/sensor_localisation';folder.mkdir(parents=True,exist_ok=True)
    (folder/'inspection.json').write_text(json.dumps(c,indent=2)+'\n')
    c.update(active_tree_inspection=False,continuous_tree_scanning=False,rolling_canopy_ascent=False,
             tree_continuous_ascent=False,smooth_approach_ascent=False,scan_target_repair=False,
             scan_adaptive_height=False,
             continuous_survey=False,survey_waypoints=[[0.,0.,2.]],survey_dwell=20.)
    (folder/'hover.json').write_text(json.dumps(c,indent=2)+'\n')


if __name__=='__main__':main()
