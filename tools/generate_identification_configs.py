"""Generate simulation-only identification presets and an empty arena."""
import json
from pathlib import Path
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]


def main():
    base = json.loads((ROOT/'polinasi_nav/config/mapping_smoke.json').read_text())
    base.update(home=[0., 0., 2.4], takeoff_altitude=2.4, survey_waypoints=[[0., 0., 2.4]],
        bounds_min=[-5., -5., -.6], bounds_max=[5., 5., 5.4], navigation_window_size=[10., 10., 6.],
        command_speed=.3, camera_pitch_enabled=False, terrain_following=False,
        mock_flower_events=False, map_log_max_points=100000, continuous_survey=False,
        identification_simulation_only=True, identification_empty_arena=True,
        identification_setpoint_interface='PVA_ENU', free_ttl=1e9,
        identification_payload_kg=0., adaptive_forgetting=.995, adaptive_window=120,
        adaptive_model_file=str(ROOT/'polinasi_nav/config/identified_pva_model.json'))
    for profile in ('train', 'validation'):
        config = dict(base, identification_profile=profile)
        (ROOT/f'polinasi_nav/config/identification_{profile}.json').write_text(json.dumps(config, indent=2)+'\n')
    tree = ET.parse(ROOT/'polinasi_nav/worlds/clear_orbit.sdf')
    world = tree.getroot().find('world')
    world.set('name', 'polinasi_identification')
    for model in list(world.findall('model')):
        if model.get('name') != 'obstacle_0':
            world.remove(model)
    ground = world.find("model[@name='obstacle_0']")
    ground.find('pose').text = '0 0 -0.3 0 0 0'
    for size in ground.findall('.//box/size'):
        size.text = '30 30 0.6'
    tree.write(ROOT/'polinasi_nav/worlds/identification.sdf', encoding='utf-8', xml_declaration=True)


if __name__ == '__main__':
    main()
