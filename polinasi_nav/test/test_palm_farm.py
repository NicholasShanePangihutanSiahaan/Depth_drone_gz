"""Asset integrity checks; do not claim flight success from scene validation."""
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
from polinasi_nav.config import load_config

ROOT = Path(__file__).resolve().parents[2]
NS = {'c': 'http://www.collada.org/2005/11/COLLADASchema'}


def test_farm_scene_targets_collision_and_launch_clearance():
    world = ET.parse(ROOT/'polinasi_nav/worlds/palm_farm.sdf').getroot().find('world')
    config = load_config(ROOT/'polinasi_nav/config/palm_farm_navigation.json')
    palms = [m for m in world.findall('model') if m.get('name').startswith('oil_palm_')]
    assert len(palms) >= 20
    locations = [list(map(float, m.findtext('pose').split()[:3])) for m in palms]
    for target in config['trees']:
        assert any(np.allclose(p[:2], target[:2]) for p in locations)
    assert all(np.linalg.norm(p[:2]) >= 6 for p in locations)
    for model in palms:
        assert model.findtext('static') == 'true'
        assert model.findtext('link/visual/geometry/mesh/uri') == model.findtext('link/collision/geometry/mesh/uri')
    assert world.find('light/diffuse') is not None
    assert world.findtext('scene/background') != '0 0 0 1'


def test_palm_mesh_has_valid_indices_and_colored_leaf_trunk_fruit_materials():
    root = ET.parse(ROOT/'polinasi_nav/models/palm_farm/meshes/palm.dae').getroot()
    positions = root.find('.//c:source/c:float_array', NS)
    coords = np.array(list(map(float, positions.text.split()))).reshape(-1, 3)
    assert np.all(np.isfinite(coords))
    assert coords[:, 2].min() >= -.04  # Lower leaf bases slightly embedded in soil.
    assert coords[:, 2].max() > 5.5
    materials = {m.get('id') for m in root.findall('.//c:material', NS)}
    assert {'bark0-mat', 'green0-mat', 'fruit0-mat'} <= materials
    for triangles in root.findall('.//c:triangles', NS):
        indices = list(map(int, triangles.findtext('c:p', namespaces=NS).split()))
        assert len(indices) == int(triangles.get('count'))*6
        assert min(indices) >= 0 and max(indices) < len(coords)
        assert triangles.find("c:input[@semantic='NORMAL']", NS) is not None
    normals = np.array(list(map(float, root.find(".//c:source[@id='normals']/c:float_array", NS).text.split()))).reshape(-1, 3)
    assert normals.shape == coords.shape
    assert np.all(np.linalg.norm(normals, axis=1) > .99)
    for color in root.findall('.//c:diffuse/c:color', NS):
        rgba = list(map(float, color.text.split()))
        assert len(rgba) == 4 and sum(rgba[:3]) > 0


def test_ground_assets_cover_farm_and_old_scenes_have_diffuse_colors():
    root = ET.parse(ROOT/'polinasi_nav/models/palm_farm/meshes/ground.dae').getroot()
    coords = np.array(list(map(float, root.findtext('.//c:float_array', namespaces=NS).split()))).reshape(-1, 3)
    assert coords[:, 0].min() <= -40 and coords[:, 0].max() >= 50
    for name in ('clear_orbit', 'leaf_blocked', 'vertical_detour', 'fully_blocked', 'sensor_dropout'):
        world = ET.parse(ROOT/f'polinasi_nav/worlds/{name}.sdf').getroot()
        for visual in world.findall('.//model/link/visual'):
            assert visual.find('material/diffuse') is not None
