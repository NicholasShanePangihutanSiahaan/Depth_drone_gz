"""MID-360-style configuration checks; no ROS master or hardware required."""
from pathlib import Path
import math
import xml.etree.ElementTree as ET
import yaml
import pytest

ROOT=Path(__file__).resolve().parents[1]


@pytest.mark.parametrize('name',['plantation_quadrotor','plantation_quadrotor_ardupilot'])
def test_top_livox_geometry_topics_and_coverage(name):
    model=ET.parse(ROOT/'models'/name/'model.sdf').getroot().find('model')
    link=model.find("link[@name='livox_top_link']")
    assert list(map(float,link.findtext('pose').split()))==[0.,0.,.18,0.,0.,0.]
    assert link.find('pose').get('relative_to')=='base_link'
    assert float(link.findtext('inertial/mass'))==.265
    joint=model.find("joint[@name='livox_top_mount']")
    assert joint.get('type')=='fixed' and joint.findtext('parent')=='base_link'
    assert joint.findtext('child')=='livox_top_link'
    lidar=link.find("sensor[@name='livox_top_lidar']")
    assert lidar.get('type')=='gpu_lidar'
    assert lidar.findtext('topic')=='/livox/top/scan'
    assert lidar.findtext('gz_frame_id')=='livox_top_link'
    assert float(lidar.findtext('update_rate'))==10.
    horizontal=lidar.find('lidar/scan/horizontal')
    step=2*math.pi/360
    assert float(horizontal.findtext('max_angle'))-float(horizontal.findtext('min_angle'))==pytest.approx(2*math.pi-step)
    vertical=lidar.find('lidar/scan/vertical')
    assert float(vertical.findtext('min_angle'))==pytest.approx(math.radians(-7))
    assert float(vertical.findtext('max_angle'))==pytest.approx(math.radians(52))
    assert int(vertical.findtext('samples'))==32
    imu=link.find("sensor[@name='livox_top_imu']")
    assert imu.findtext('topic')=='/livox/top/imu'
    assert imu.findtext('gz_frame_id')=='livox_top_link'
    assert float(imu.findtext('update_rate'))==200.
    topics=[s.findtext('topic') for s in model.findall('.//sensor')]
    assert len(topics)==len(set(topics))


def test_independent_one_way_bridges_preserve_existing_sensors():
    bridges=yaml.safe_load((ROOT/'config/bridge_config.yaml').read_text())
    by_topic={b['ros_topic_name']:b for b in bridges}
    assert len(by_topic)==len(bridges)
    assert by_topic['/livox/top/points']['gz_topic_name']=='/livox/top/scan/points'
    assert by_topic['/livox/top/points']['ros_type_name']=='sensor_msgs/msg/PointCloud2'
    assert by_topic['/livox/top/imu']['ros_type_name']=='sensor_msgs/msg/Imu'
    for name in ('/livox/top/points','/livox/top/imu'):
        assert by_topic[name]['direction']=='GZ_TO_ROS'
        assert by_topic[name]['qos_profile']=='SENSOR_DATA'
    assert by_topic['/imu']['gz_topic_name']=='/plantation_uav/imu'
    assert '/range' in by_topic and '/zed2i/depth/depth_registered' in by_topic


def test_worlds_support_imu_and_single_static_top_transform():
    for name in ('plantation.sdf','plantation_simple.sdf','plantation_ardupilot.sdf'):
        plugins=ET.parse(ROOT/'worlds'/name).findall('.//world/plugin')
        assert sum(p.get('name')=='gz::sim::systems::Imu' for p in plugins)==1
    launch=(ROOT/'launch/plantation_sim.launch.py').read_text()
    assert launch.count("name='livox_top_mount_tf'")==1
    assert "'--z', '0.18'" in launch and "'--child-frame-id', 'livox_top_link'" in launch
