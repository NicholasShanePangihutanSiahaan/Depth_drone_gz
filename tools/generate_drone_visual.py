"""Shared primitive geometry for Gazebo and dependency-free browser preview.

Static design asset only: never substitutes a flight dynamics model.
"""
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
MODEL = ROOT/'polinasi_nav/models/polinasi_visual_50cm'


def geometry():
    parts = []
    def part(name, kind, size, xyz, colour, rpy=(0,0,0), group='body'):
        parts.append(dict(name=name,kind=kind,size=list(size),pose=[*xyz,*rpy],colour=colour,group=group))
    dark=[.12,.16,.18]; grey=[.32,.37,.39]; silver=[.68,.72,.74]
    part('body', 'box', [.26,.18,.095], [0,0,0],grey)
    part('top_plate','box',[.23,.165,.014],[0,0,.055],dark)
    part('battery_tray','box',[.23,.145,.016],[-.025,0,-.075],silver)
    part('battery','box',[.18,.115,.055],[-.035,0,-.112],[.19,.22,.24])
    for x in [-.085,.035]:
        part('battery_strap_'+str(x),'box',[.014,.119,.059],[x,0,-.112],[.07,.08,.09])
    # Opposite motor centres across a diagonal are exactly 0.50 m.
    motor_centres=[]
    for i,angle in enumerate([math.pi/4,3*math.pi/4,5*math.pi/4,7*math.pi/4]):
        x,y=.25*math.cos(angle),.25*math.sin(angle)
        motor_centres.append([x,y,.035])
        part(f'arm_{i}','box',[.25,.025,.022],[x/2,y/2,-.007],dark,(0,0,angle))
        part(f'motor_{i}','cylinder',[.022,.041],[x,y,.025],dark)
        part(f'motor_cap_{i}','cylinder',[.0225,.006],[x,y,.048],silver)
        part(f'propeller_{i}','box',[.20,.014,.003],[x,y,.057],[.25,.29,.31],(0,0,angle+.35))
    # Narrow skid rails and inclined legs, matching the user's sketch.
    for side in [-1,1]:
        y=side*.12
        part('skid_'+str(side),'cylinder',[.006,.38],[0,y,-.235],grey,(0,math.pi/2,0))
        for x in [-.075,.075]:
            part(f'leg_{side}_{x}','cylinder',[.006,.19],[x,y*.84,-.145],grey,(side*-.20,0,0))
    part('lidar_bracket','box',[.08,.027,.012],[.158,0,.065],silver)
    part('lidar_roll_axis','cylinder',[.006,.06],[.16,0,.085],dark,(0,math.pi/2,0))
    # Group local coordinates; link origin is the actual rolling pivot.
    part('mid360_base','box',[.065,.065,.032],[0,0,-.012],silver,group='lidar')
    part('mid360_window','sphere',[.027],[0,0,.015],[.06,.55,.77],group='lidar')
    part('mid360_label','box',[.001,.04,.011],[.033,0,-.012],[.23,.27,.29],group='lidar')
    part('zed_pitch_mount','box',[.06,.035,.018],[0,0,.072],silver)
    # Optical origin is z=-.07; housing stays behind its downward beam.
    part('down_rangefinder','sphere',[.009],[.115,0,-.056],[.07,.11,.13])
    part('zed_housing','box',[.028,.175,.028],[0,0,0],dark,group='camera')
    for y in [-.060,.060]:
        part('zed_lens_'+str(y),'cylinder',[.011,.005],[.017,y,0],[.04,.23,.32],(0,math.pi/2,0),group='camera')
    return dict(name='Polinasi — konsep visual 50 cm',span_m=.5,
                span_definition='opposite_motor_centres_diagonal_X',motor_centres=motor_centres,
                axes='FLU: X depan, Y kiri, Z atas',
                groups=dict(body=dict(pose=[0,0,0,0,0,0]),
                            lidar=dict(pose=[.205,0,.085,0,0,0],axis=[1,0,0]),
                            camera=dict(pose=[0,0,.098,0,0,math.pi/2],axis=[1,0,0])),
                parts=parts,scope='visual_concept_not_validated_flight_dynamics',
                estimates='Only span and sensor directions specified; other dimensions estimated.')


def export(scene, directory=MODEL):
    directory.mkdir(parents=True,exist_ok=True)
    (directory/'scene.json').write_text(json.dumps(scene,indent=2)+'\n')
    sdf=ET.Element('sdf',version='1.9'); model=ET.SubElement(sdf,'model',name='polinasi_visual_50cm')
    ET.SubElement(model,'static').text='true'
    for group,spec in scene['groups'].items():
        link=ET.SubElement(model,'link',name=group)
        ET.SubElement(link,'pose').text=' '.join(map(str,spec['pose']))
        for p in (p for p in scene['parts'] if p['group']==group):
            visual=ET.SubElement(link,'visual',name=p['name'])
            ET.SubElement(visual,'pose').text=' '.join(map(str,p['pose']))
            shape=ET.SubElement(ET.SubElement(visual,'geometry'),p['kind'])
            if p['kind']=='box': ET.SubElement(shape,'size').text=' '.join(map(str,p['size']))
            else:
                ET.SubElement(shape,'radius').text=str(p['size'][0])
                if p['kind']=='cylinder': ET.SubElement(shape,'length').text=str(p['size'][1])
            material=ET.SubElement(visual,'material')
            for field in ['ambient','diffuse']:
                ET.SubElement(material,field).text=' '.join(map(str,[*p['colour'],1]))
    for group in ['lidar','camera']:
        joint=ET.SubElement(model,'joint',name=group+'_visual_axis',type='revolute')
        ET.SubElement(joint,'parent').text='body'; ET.SubElement(joint,'child').text=group
        axis=ET.SubElement(joint,'axis')
        ET.SubElement(axis,'xyz',expressed_in='__model__').text='1 0 0'
    ET.ElementTree(sdf).write(directory/'model.sdf',encoding='utf-8',xml_declaration=True)
    config=ET.Element('model'); ET.SubElement(config,'name').text='Polinasi visual 50cm'
    ET.SubElement(config,'version').text='1.0'
    ET.SubElement(config,'sdf',version='1.9').text='model.sdf'
    ET.SubElement(config,'description').text='Static visual concept. No flight controller, motors, sensor drivers or dynamic validation.'
    ET.ElementTree(config).write(directory/'model.config',encoding='utf-8',xml_declaration=True)


if __name__=='__main__':
    export(geometry())
    print(MODEL)
