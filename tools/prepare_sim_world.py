"""Create a run-local world with slower wall-clock pacing, identical physics."""
import argparse
import math
import json
from pathlib import Path
import xml.etree.ElementTree as ET


def prepare(source, output, factor, sensor_config=None):
    if not math.isfinite(factor) or not 0 < factor <= 1:
        raise ValueError('simulation real-time factor must be finite, in (0, 1]')
    tree = ET.parse(source)
    physics = tree.getroot().find('world/physics')
    if physics is None:
        raise ValueError('world has no physics configuration')
    element = physics.find('real_time_factor')
    if element is None:
        element = ET.SubElement(physics, 'real_time_factor')
    element.text = str(factor)
    if sensor_config is not None:
        config = json.loads(Path(sensor_config).read_text())
        config['simulation_real_time_factor'] = factor
        (Path(output).parent/'mission_config.json').write_text(json.dumps(config, indent=2)+'\n')
        maximum = float(config['lidar_range'])
        if not math.isfinite(maximum) or maximum <= config['lidar_min_range']:
            raise ValueError('invalid simulation LiDAR range')
        project = Path(__file__).resolve().parents[1]
        directory = Path(output).parent/'models/iris_lidar_mapping'
        directory.mkdir(parents=True, exist_ok=True)
        sensor_tree = ET.parse(project/'polinasi_nav/models/iris_lidar/model.sdf')
        sensor_tree.getroot().find(".//sensor[@name='mid360']/lidar/range/max").text = str(maximum)
        if config.get('identification_empty_arena'):
            if not config.get('identification_simulation_only') or source.stem != 'identification':
                raise ValueError('sensor removal is restricted to the identification arena')
            # Keep IMU and all inertial/actuation properties. Remove rendering
            # sensors, including the downward GPU laser and camera, only here.
            for link in sensor_tree.getroot().findall('.//link'):
                for sensor in list(link.findall('sensor')):
                    if sensor.get('type') in ('gpu_lidar', 'lidar', 'camera'):
                        link.remove(sensor)
            world = tree.getroot().find('world')
            for plugin in list(world.findall('plugin')):
                if plugin.get('filename') == 'gz-sim-sensors-system':
                    world.remove(plugin)  # No rendering sensors in this arena.
        sensor_tree.write(directory/'model.sdf', encoding='utf-8', xml_declaration=True)
        metadata = ET.parse(project/'polinasi_nav/models/iris_lidar/model.config')
        metadata.write(directory/'model.config', encoding='utf-8', xml_declaration=True)
        for include in tree.getroot().findall('world/include'):
            if include.findtext('uri') == 'model://iris_lidar':
                include.find('uri').text = 'model://iris_lidar_mapping'
                ET.SubElement(include, 'name').text = 'iris_lidar'
    tree.write(output, encoding='utf-8', xml_declaration=True)


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('source', type=Path)
    parser.add_argument('output', type=Path)
    parser.add_argument('--factor', type=float, default=.2)
    parser.add_argument('--sensor-config', type=Path)
    args = parser.parse_args()
    prepare(args.source, args.output, args.factor, args.sensor_config)
