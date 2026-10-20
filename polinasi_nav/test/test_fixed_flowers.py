"""World selection must preserve fixed mounting and sensor mission policy."""
import json
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import pytest
from polinasi_nav.config import load_config

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT/'tools'))
from ensure_palm_flowers import audit_assets
from generate_fixed_lidar import main as generate
from prepare_sim_world import prepare


def test_flower_world_preserves_fixed_sensor_mission_and_obstacles(tmp_path):
    generate('palm_farm_flowers')
    path = ROOT/'reports/fixed_lidar/inspection.json'
    c = load_config(path)
    assert c['simulation_scenario'] == 'palm_farm_flowers'
    assert c['sensor_fixed_localisation'] and not c['sensor_rolling_localisation']
    assert c['lidar_fixed'] and c['fixed_fov_assumptions']
    assert c['tree_demo_limit'] == 2 and c['active_tree_inspection']
    assert not c['mock_flower_events']
    assert c['speed'] == .4 and c['tracking_limit'] == .2
    source = ROOT/'polinasi_nav/worlds/palm_farm_flowers.sdf'
    output = tmp_path/'world.sdf'
    prepare(source, output, .15, path)
    original = ET.parse(source).getroot().find('world')
    prepared = ET.parse(output).getroot().find('world')
    trees = [m for m in original.findall('model') if m.get('name').startswith('oil_palm_')]
    assert len(trees) == 29
    assert any(m.findall("link/collision") for m in trees)
    for tree in trees:
        assert ET.tostring(tree) == ET.tostring(prepared.find(f"model[@name='{tree.get('name')}']"))
    model = ET.parse(tmp_path/'models/iris_lidar_mapping/model.sdf').getroot().find('model')
    assert model.find("joint[@name='lidar_pitch']").get('type') == 'fixed'
    assert model.find("joint[@name='lidar_roll']") is None
    assert model.find("joint[@name='camera_yaw']").get('type') == 'fixed'
    assert model.find("link[@name='mid360_link']/pose").text == '0.0 0.0 0.28 0.0 0.0 0.0'
    saved = json.loads((tmp_path/'mission_config.json').read_text())
    assert saved['simulation_scenario'] == 'palm_farm_flowers'


def test_resource_audit_rejects_missing_mesh_and_changes_hash(tmp_path):
    world = tmp_path/'polinasi_nav/worlds/palm_farm_flowers.sdf'
    world.parent.mkdir(parents=True)
    world.write_text('<sdf><world name="test"><model name="oil_palm_00"><link><visual><geometry><mesh><uri>model://palm_farm_flowers/meshes/palm_00.dae</uri></mesh></geometry></visual></link></model></world></sdf>')
    with pytest.raises(FileNotFoundError, match='palm_00.dae'):
        audit_assets(tmp_path)
    mesh = tmp_path/'polinasi_nav/models/palm_farm_flowers/meshes/palm_00.dae'
    mesh.parent.mkdir(parents=True)
    mesh.write_text('first resource')
    first = audit_assets(tmp_path)
    mesh.write_text('changed resource')
    second = audit_assets(tmp_path)
    assert first['tree_count'] == 1
    assert first['resource_sha256'] != second['resource_sha256']


def test_invalid_fixed_scenario_fails_before_generating():
    with pytest.raises(ValueError, match='unsupported'):
        generate('palm_farm_realistic')
