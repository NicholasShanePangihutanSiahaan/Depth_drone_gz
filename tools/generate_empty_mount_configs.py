"""Empty arena with rendering sensors retained, unarmed mount tests only."""
import json
from pathlib import Path
ROOT = Path(__file__).resolve().parents[1]
c = json.loads((ROOT/'polinasi_nav/config/tree_inspection_gimbal.json').read_text())
c.update(active_tree_inspection=False,automatic_tilt_ascent=False,
         identification_empty_arena=False,survey_waypoints=[[0.,0.,2.]],
         bounds_min=[-8.,-8.,-.6],bounds_max=[8.,8.,8.],lidar_range=12.,
         mission_scope='unarmed_empty_arena_mount_AB_test_not_flight')
for variant in ('visible','hidden'):
    c['simulation_lidar_hide_self'] = variant=='hidden'
    (ROOT/f'polinasi_nav/config/empty_mount_{variant}.json').write_text(json.dumps(c,indent=2)+'\n')
c.update(simulation_lidar_hide_self=False,mapping_corridor_enabled=False,
         simulation_lidar_validation_box=dict(centre=[3.,4.,3.],size=[1.,1.,1.]))
(ROOT/'polinasi_nav/config/empty_mount_fixture.json').write_text(json.dumps(c,indent=2)+'\n')
