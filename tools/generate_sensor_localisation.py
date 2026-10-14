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
    folder=ROOT/'reports/sensor_localisation';folder.mkdir(parents=True,exist_ok=True)
    (folder/'inspection.json').write_text(json.dumps(c,indent=2)+'\n')
    c.update(active_tree_inspection=False,continuous_tree_scanning=False,rolling_canopy_ascent=False,
             tree_continuous_ascent=False,smooth_approach_ascent=False,
             continuous_survey=False,survey_waypoints=[[0.,0.,2.]],survey_dwell=20.)
    (folder/'hover.json').write_text(json.dumps(c,indent=2)+'\n')


if __name__=='__main__':main()
