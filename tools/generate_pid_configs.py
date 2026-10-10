"""PID presets derived from the same mapping route and safety limits."""
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def main():
    for name in ('smoke', 'check', 'tour'):
        c = json.loads((ROOT/f'polinasi_nav/config/mapping_{name}.json').read_text())
        c.update(pid_enabled=True, predictive_enabled=False,
                 pid_kp=[.6,.6,.6], pid_ki=[.03,.03,.03], pid_kd=[.9,.9,.9],
                 pid_integral_limit=.5, pid_correction_acceleration=.15,
                 controller_description='model_free_position_PID_plus_PVA_feedforward')
        (ROOT/f'polinasi_nav/config/pid_{name}.json').write_text(json.dumps(c,indent=2)+'\n')
        if name == 'tour':
            check = dict(c, survey_waypoints=c['survey_waypoints'][:4], survey_max_duration=600.,
                         validation_scope='first_four_farm_survey_targets_return_land')
            (ROOT/'polinasi_nav/config/pid_tour_check.json').write_text(json.dumps(check,indent=2)+'\n')


if __name__ == '__main__':
    main()
