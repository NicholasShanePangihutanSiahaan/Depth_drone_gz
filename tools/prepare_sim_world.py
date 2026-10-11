"""Create a run-local world with slower wall-clock pacing, identical physics."""
import argparse
import math
import json
from pathlib import Path
import xml.etree.ElementTree as ET


def add_sensor_gimbals(sensor_tree, camera_yaw_enabled=True):
    """Opt-in run-local joints; never modify the original fixed-sensor model."""
    model = sensor_tree.getroot().find('model')
    # A moving first direct link must not become the model's pose reference.
    # The odometry plugin reads model pose, not the robot_base_frame label.
    model.set('canonical_link','iris_with_standoffs::base_link')
    pose_plugin = ET.SubElement(model,'plugin',filename='gz-sim-pose-publisher-system',name='gz::sim::systems::PosePublisher')
    ET.SubElement(pose_plugin,'publish_link_pose').text = 'true'
    ET.SubElement(pose_plugin,'use_pose_vector_msg').text = 'true'
    ET.SubElement(pose_plugin,'update_frequency').text = '10'
    joint = model.find("joint[@name='lidar_mount']")
    joint.set('name', 'lidar_pitch')
    joint.set('type', 'revolute')
    axis = ET.SubElement(joint, 'axis')
    ET.SubElement(axis, 'xyz').text = '0 -1 0'  # Positive command looks up.
    limits = ET.SubElement(axis, 'limit')
    for key, value in [('lower','-0.2'),('upper',str(math.radians(70))),('velocity','0.4'),('effort','0.3')]:
        ET.SubElement(limits,key).text = value
    yaw_link = ET.SubElement(model,'link',name='camera_yaw_link')
    ET.SubElement(yaw_link,'pose').text = '0.20 0 0.02 0 0 0'
    inertial = ET.SubElement(yaw_link,'inertial')
    ET.SubElement(inertial,'mass').text = '0.01'
    inertia = ET.SubElement(inertial,'inertia')
    for key in ('ixx','iyy','izz'):
        ET.SubElement(inertia,key).text = '0.00002'
    visual = ET.SubElement(yaw_link,'visual',name='gimbal_base')
    ET.SubElement(ET.SubElement(ET.SubElement(visual,'geometry'),'box'),'size').text = '0.035 0.035 0.025'
    yaw_joint = ET.SubElement(model,'joint',name='camera_yaw',type='revolute' if camera_yaw_enabled else 'fixed')
    ET.SubElement(yaw_joint,'parent').text = 'iris_with_standoffs::base_link'
    ET.SubElement(yaw_joint,'child').text = 'camera_yaw_link'
    axis = ET.SubElement(yaw_joint,'axis')
    ET.SubElement(axis,'xyz').text = '0 0 1'
    limits = ET.SubElement(axis,'limit')
    for key,value in [('lower','-2.96705972839'),('upper','2.96705972839'),('velocity','0.6'),('effort','0.2')]:
        ET.SubElement(limits,key).text = value
    if not camera_yaw_enabled:
        yaw_joint.remove(axis)
    model.find("joint[@name='camera_pitch']/parent").text = 'camera_yaw_link'
    # Extend pitch for viewing a crown from below. No ZED stereo/depth emulation.
    model.find("joint[@name='camera_pitch']/axis/limit/upper").text = '1.2'
    for name,topic in [('lidar_pitch','/lidar/pitch_cmd'),('camera_yaw','/camera/yaw_cmd')]:
        if name=='camera_yaw' and not camera_yaw_enabled:
            continue
        plugin = ET.SubElement(model,'plugin',filename='gz-sim-joint-position-controller-system',name='gz::sim::systems::JointPositionController')
        for key,value in [('joint_name',name),('topic',topic),('p_gain','0.15'),('d_gain','0.02'),('cmd_max','0.3'),('cmd_min','-0.3')]:
            ET.SubElement(plugin,key).text = value
    camera_state = model.find("plugin[@name='gz::sim::systems::JointStatePublisher']")
    if camera_yaw_enabled:
        ET.SubElement(camera_state,'joint_name').text = 'camera_yaw'
    plugin = ET.SubElement(model,'plugin',filename='gz-sim-joint-state-publisher-system',name='gz::sim::systems::JointStatePublisher')
    ET.SubElement(plugin,'joint_name').text = 'lidar_pitch'
    ET.SubElement(plugin,'topic').text = '/lidar/joint_state'


def exclude_simulation_self_visuals(sensor_tree, model_root, project):
    """GPU-LiDAR-only self exclusion; preserve physics and external visuals.

    Gazebo renders visual geometry, not collision geometry. Run-local copies
    use bit 2 for vehicle visuals and LiDAR mask bit 1. Camera still sees both.
    This deliberately simplifies physical self occlusion; not a hardware filter.
    """
    for name in ('iris_with_standoffs','iris_with_ardupilot'):
        original = project/'.dependencies/ardupilot_gazebo/models'/name
        copied = model_root/(name+'_lidar_filtered')
        copied.mkdir(parents=True,exist_ok=True)
        model = ET.parse(original/'model.sdf')
        for visual in model.getroot().findall('.//visual'):
            flags = visual.find('visibility_flags')
            if flags is None: flags = ET.SubElement(visual,'visibility_flags')
            flags.text = '2'
        for uri in model.getroot().findall('.//include/uri'):
            if uri.text == 'model://iris_with_standoffs':
                uri.text = 'model://iris_with_standoffs_lidar_filtered'
        model.write(copied/'model.sdf',encoding='utf-8',xml_declaration=True)
        ET.parse(original/'model.config').write(copied/'model.config',encoding='utf-8',xml_declaration=True)
    sensor_tree.getroot().find('model/include/uri').text = 'model://iris_with_ardupilot_lidar_filtered'
    for visual in sensor_tree.getroot().findall('.//visual'):
        ET.SubElement(visual,'visibility_flags').text = '2'
    ET.SubElement(sensor_tree.getroot().find(".//sensor[@name='mid360']/lidar"),'visibility_mask').text = '1'


def prepare(source, output, factor, sensor_config=None):
    if not math.isfinite(factor) or not 0 < factor <= 1:
        raise ValueError('simulation real-time factor must be finite, in (0, 1]')
    tree = ET.parse(source)
    if sensor_config is not None:
        validation = json.loads(Path(sensor_config).read_text()).get('simulation_lidar_validation_box')
        if validation is not None:
            if source.stem != 'identification':
                raise ValueError('known calibration box is restricted to the empty sensor test arena')
            model = ET.SubElement(tree.getroot().find('world'),'model',name='lidar_validation_box')
            ET.SubElement(model,'static').text = 'true'
            ET.SubElement(model,'pose').text = ' '.join(map(str,[*validation['centre'],0,0,0]))
            link = ET.SubElement(model,'link',name='box')
            for kind in ('collision','visual'):
                element = ET.SubElement(link,kind,name=kind)
                ET.SubElement(ET.SubElement(ET.SubElement(element,'geometry'),'box'),'size').text = ' '.join(map(str,validation['size']))
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
        if config.get('sensor_gimbals_enabled'):
            if config.get('identification_empty_arena'):
                raise ValueError('gimbals are not enabled in the identification arena')
            if config['lidar_mount'] != [0.,0.,.2,0.,0.,0.] or config['camera_mount'] != [.2,0.,.02]:
                raise ValueError('gimbal SDF currently requires the calibrated default mounting poses')
            add_sensor_gimbals(sensor_tree, config.get('camera_yaw_enabled',True))
            if config.get('simulation_lidar_hide_self'):
                exclude_simulation_self_visuals(sensor_tree, directory.parent, project)
        if config.get('depth_camera_enabled'):
            if not config.get('sensor_gimbals_enabled'):
                raise ValueError('depth fixture requires timestamped gimbal transforms')
            camera_sensor = sensor_tree.getroot().find(".//sensor[@name='zed_placeholder']")
            camera_sensor.set('type','rgbd_camera')
            camera_sensor.find('topic').text = '/zed'
            camera_sensor.find('gz_frame_id').text = 'zed_camera_link'
            camera_sensor.find('update_rate').text = '5'
            camera = camera_sensor.find('camera')
            ET.SubElement(camera,'optical_frame_id').text = 'zed_camera_optical'
            camera.find('image/width').text = '320'
            camera.find('image/height').text = '240'
            camera.find('clip/far').text = str(config.get('depth_max_range',8.))
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
