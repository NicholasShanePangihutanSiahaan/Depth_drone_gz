"""Branch-specific simulator fixture. Old configs and assets remain intact."""
import json
import math
from pathlib import Path
from generate_drone_visual import geometry,export
from generate_rolling_hover_test import generate

ROOT=Path(__file__).resolve().parents[1]


def main():
    scene=geometry();export(scene);generate(math.pi)
    c=json.loads((ROOT/'reports/rolling_hover/config.json').read_text())
    c.update(simulation_visual_model='polinasi_visual_50cm',
             lidar_mount=scene['groups']['lidar']['pose'],
             camera_mount=scene['groups']['camera']['pose'][:3],camera_mount_yaw=math.pi/2,
             rangefinder_mount=[.115,0.,-.07,0.,math.pi/2,0.],
             drone_dimensions=[.60,.60,.50],self_filter_shape_padding=.05,
             mission_scope='visual50_pitch_left_rolling_hover_GT_only',survey_dwell=300.)
    # Static chassis geometry plus conservative envelopes for moving sensors.
    shapes=[p for p in scene['parts'] if p['group']=='body']
    for group,radius in [('lidar',.055),('camera',.095)]:
        shapes.append(dict(kind='sphere',size=[radius],pose=[*scene['groups'][group]['pose'][:3],0,0,0]))
    c['self_filter_shapes']=shapes
    output=ROOT/'reports/visual50_hover';output.mkdir(parents=True,exist_ok=True)
    (output/'config.json').write_text(json.dumps(c,indent=2)+'\n')


if __name__=='__main__':main()
