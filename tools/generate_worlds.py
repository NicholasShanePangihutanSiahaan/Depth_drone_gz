"""Generate Gazebo scene assets from the same geometry as planner scenarios."""
import argparse
import sys
from pathlib import Path
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'polinasi_nav'))
from polinasi_nav.config import load_config
from polinasi_nav.simulation import SCENARIOS, Scene


def add(parent, tag, text=None, **attrs):
    node = ET.SubElement(parent, tag, attrs)
    if text is not None:
        node.text = str(text)
    return node


def world(name, config):
    sdf = ET.Element('sdf', version='1.9')
    w = add(sdf, 'world', name='polinasi')
    physics = add(w, 'physics', name='1ms', type='ignore')
    add(physics, 'max_step_size', '0.001')
    add(physics, 'real_time_factor', '1.0')
    for library, system in [('physics', 'Physics'), ('sensors', 'Sensors'), ('imu', 'Imu'),
                            ('user-commands', 'UserCommands'), ('scene-broadcaster', 'SceneBroadcaster')]:
        plugin = add(w, 'plugin', filename=f'gz-sim-{library}-system', name=f'gz::sim::systems::{system}')
        if system == 'Sensors':
            add(plugin, 'render_engine', 'ogre2')
    scene = add(w, 'scene')
    add(scene, 'ambient', '0.8 0.8 0.8')
    light = add(w, 'light', name='sun', type='directional')
    add(light, 'pose', '0 0 10 0 0 0')
    add(light, 'diffuse', '0.8 0.8 0.8 1')
    add(light, 'direction', '-0.4 0.2 -0.9')
    vehicle = add(w, 'include')
    add(vehicle, 'uri', 'model://iris_lidar')
    add(vehicle, 'pose', ' '.join(map(str, config['initial_pose']))+' 0 0 0')
    for index, (lo, hi) in enumerate(Scene(config, name).boxes):
        model = add(w, 'model', name=f'obstacle_{index}')
        add(model, 'static', 'true')
        add(model, 'pose', ' '.join(map(str, (lo+hi)/2))+' 0 0 0')
        link = add(model, 'link', name='obstacle')
        for tag in ('collision', 'visual'):
            element = add(link, tag, name=tag)
            geometry = add(element, 'geometry')
            box = add(geometry, 'box')
            add(box, 'size', ' '.join(map(str, hi-lo)))
            if tag == 'visual':
                material = add(element, 'material')
                add(material, 'ambient', '0.3 0.6 0.2 1' if index else '0.5 0.5 0.4 1')
                add(material, 'diffuse', '0.3 0.6 0.2 1' if index else '0.5 0.5 0.4 1')
    ET.indent(sdf, space='  ')
    return ET.ElementTree(sdf)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--config', type=Path)
    parser.add_argument('--output', type=Path, default=ROOT/'polinasi_nav/worlds')
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    c = load_config(args.config)
    for name in SCENARIOS:
        world(name, c).write(args.output/f'{name}.sdf', encoding='utf-8', xml_declaration=True)


if __name__ == '__main__':
    main()
