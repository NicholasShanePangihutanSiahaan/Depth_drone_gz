"""Opt-in simulation MPC presets; original missions stay unchanged."""
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
for name, source in (('arena', 'identification_validation'),
                     ('smoke', 'mapping_smoke'), ('tour', 'mapping_tour')):
    c = json.loads((ROOT/f'polinasi_nav/config/{source}.json').read_text())
    c.update(predictive_enabled=True, predictive_simulation_only=True,
        predictive_adaptation=False, command_speed=.3, speed=.4,
        identification_setpoint_interface='PVA_ENU', identification_payload_kg=0.,
        adaptive_model_file=str(ROOT/'polinasi_nav/config/identified_pva_model.json'))
    (ROOT/f'polinasi_nav/config/predictive_{name}.json').write_text(json.dumps(c, indent=2)+'\n')
    if name == 'tour':
        # Focused regression of the actual farm's first survey legs plus return
        # and land. Same sensors/physics/map, explicitly not full coverage.
        check = dict(c, survey_waypoints=c['survey_waypoints'][:4], survey_max_duration=600.,
                     validation_scope='first_four_farm_survey_targets_return_land')
        (ROOT/'polinasi_nav/config/predictive_tour_check.json').write_text(json.dumps(check, indent=2)+'\n')
