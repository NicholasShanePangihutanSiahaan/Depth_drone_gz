"""Realistic palms at the flower farm's original heights, with mild site jitter."""
import copy
import hashlib
import json
import math
import random
import xml.etree.ElementTree as ET

import numpy as np

from generate_palm_flowers import ASSET, ROOT, SOURCE_WORLD
from generate_palm_realistic import palm, tree_parameters, trunk_centre

NS = {'c': 'http://www.collada.org/2005/11/COLLADASchema'}
MAX_DISPLACEMENT = 1.6
MIN_SPACING = 6.5


def protected_hashes():
    paths = []
    for name in ('palm_farm', 'palm_farm_realistic'):
        world = ROOT / f'polinasi_nav/worlds/{name}.sdf'
        if world.is_file():
            paths.append(world)
        paths.extend(p for p in (ROOT / f'polinasi_nav/models/{name}').rglob('*') if p.is_file() and p.suffix != '.html')
    return {str(p.relative_to(ROOT)): hashlib.sha256(p.read_bytes()).hexdigest() for p in sorted(paths)}


def jitter_positions(poses, seed):
    rng = random.Random(seed + 7741)
    placed = []
    for index, pose in enumerate(poses):
        for attempt in range(1000):
            angle, radius = rng.uniform(-math.pi, math.pi), rng.uniform(.45, MAX_DISPLACEMENT)
            x, y = pose[0] + radius*math.cos(angle), pose[1] + radius*math.sin(angle)
            # Reserve room around accepted trees, unprocessed sites, and launch.
            neighbours = placed + [p[:2] for p in poses[index+1:]]
            if math.hypot(x, y) >= 7.0 and all(math.hypot(x-a, y-b) >= MIN_SPACING for a, b in neighbours):
                placed.append([x, y])
                break
        else:
            raise ValueError('could not perturb planting sites with required clearance')
    return placed


def varied_parameters(index, seed):
    parameters = tree_parameters(index, seed)
    parameters['height'] = 5.2
    rng = random.Random(seed + 63101*(index+1))
    parameters.update(
        frond_lift_factor=rng.uniform(.8, 1.15),
        frond_droop_factor=rng.uniform(.8, 1.25),
        frond_sideways_factor=rng.uniform(.7, 1.6),
        leaflet_length_factor=rng.uniform(.8, 1.2),
        leaflet_width_factor=rng.uniform(.8, 1.2),
        leaflet_bend_factor=rng.uniform(.65, 1.6),
        leaflet_twist_factor=rng.uniform(.6, 1.6),
        leaflet_missing_fraction=rng.uniform(.015, .07))
    return parameters


def match_crown_height(mesh, height, target_top, target_bottom):
    """Match exact old bounds; retain trunk height and recalculate face normals."""
    old_top = max(v[2] for v in mesh.vertices)
    factor = (target_top-height)/(old_top-height)
    old_bottom = min(v[2] for v in mesh.vertices)
    mesh.vertices = [(x, y, height+(z-height)*factor if z>height else
                      (z*target_bottom/old_bottom if z<0 else z)) for x, y, z in mesh.vertices]
    points = np.asarray(mesh.vertices)
    for faces in mesh.faces.values():
        ids = np.asarray(faces)
        triangles = points[ids]
        normals = np.cross(triangles[:, 1]-triangles[:, 0], triangles[:, 2]-triangles[:, 0])
        normals /= np.linalg.norm(normals, axis=1)[:, None]
        for face, normal in zip(ids, normals):
            for vertex in face:
                mesh.normals[vertex] = tuple(normal)
    return factor


def upgrade(world_path, flower_manifest, seed):
    before = protected_hashes()
    sdf = ET.parse(world_path)
    world = sdf.getroot().find('world')
    trees = sorted((m for m in world.findall('model') if m.get('name', '').startswith('oil_palm_')), key=lambda m: m.get('name'))
    poses = [list(map(float, m.findtext('pose').split())) for m in trees]
    sites = jitter_positions(poses, seed)
    original = ET.parse(ROOT / 'polinasi_nav/models/palm_farm/meshes/palm.dae').getroot()
    points = np.fromstring(original.findtext(".//c:source[@id='positions']/c:float_array", namespaces=NS), sep=' ').reshape(-1, 3)
    local_bottom, local_top = float(points[:, 2].min()), float(points[:, 2].max())
    records = []
    for index, (tree, original_pose, xy, entry) in enumerate(zip(trees, poses, sites, flower_manifest['trees'])):
        scale = float(tree.findtext('link/visual/geometry/mesh/scale').split()[0])
        parameters = varied_parameters(index, seed)
        mesh = palm(parameters)
        crown_factor = match_crown_height(mesh, 5.2, local_top, local_bottom)
        filename = f'palm_{index:02d}.dae'
        mesh.write(ASSET / 'meshes' / filename)
        pose = [*xy, *original_pose[2:]]
        tree.find('pose').text = ' '.join(f'{v:.8f}' for v in pose)
        for uri in tree.findall("link[@name='surface']/*/geometry/mesh/uri"):
            uri.text = f'model://palm_farm_flowers/meshes/{filename}'
        flowers = []
        for flower in entry['flowers']:
            local_pose = flower['local_pose'][:]
            angle = local_pose[5]
            z = local_pose[2]/scale
            centre = trunk_centre(parameters, z)
            radius = parameters['radius']+.06
            local_pose[:3] = [(centre[0]+radius*math.cos(angle))*scale,
                             (centre[1]+radius*math.sin(angle))*scale, z*scale]
            tree.find(f"link[@name='{flower['link']}']/pose").text = ' '.join(f'{v:.8f}' for v in local_pose)
            yaw = pose[5]
            world_xyz = [pose[0]+math.cos(yaw)*local_pose[0]-math.sin(yaw)*local_pose[1],
                         pose[1]+math.sin(yaw)*local_pose[0]+math.cos(yaw)*local_pose[1], pose[2]+local_pose[2]]
            flower.update(local_pose=local_pose, attachment_world_xyz=world_xyz)
            flowers.append(dict(sex=entry['sex'], pose=local_pose, scale=flower['scale']))
        effective = parameters.copy()
        effective.update(height=5.2*scale, reach=parameters['reach']*scale)
        records.append(dict(tree=tree.get('name'), pose=pose, original_pose=original_pose,
            mesh=filename, scale=scale, parameters=effective, source_parameters=parameters,
            triangles=sum(len(faces) for faces in mesh.faces.values()), flowers=flowers,
            crown_height_factor=crown_factor, original_mesh_bounds_z=[local_bottom*scale, local_top*scale]))
        print(f'{tree.get("name")}: trunk {5.2*scale:.2f} m, top {local_top*scale:.2f} m, {parameters["fronds"]} fronds', flush=True)
    example = ET.Element('sdf', version='1.9')
    example.append(copy.deepcopy(trees[0]))
    example.find('model/pose').text = '0 0 0 0 0 0'
    ET.ElementTree(example).write(ASSET / 'model.sdf', encoding='utf-8', xml_declaration=True)
    ET.indent(sdf, space='  ')
    sdf.write(world_path, encoding='utf-8', xml_declaration=True)
    flower_manifest['tree_geometry'] = 'individual_realistic_palms_at_original_trunk_and_total_heights'
    flower_manifest['layout'] = dict(seed=seed+7741, max_displacement_m=MAX_DISPLACEMENT, min_spacing_m=MIN_SPACING)
    (ASSET / 'flower_manifest.json').write_text(json.dumps(flower_manifest, indent=2)+'\n')
    scene = dict(format_version=1, seed=seed, scenario='palm_farm_flowers', total_trees=len(records),
        trees=records, flowering=flower_manifest['counts'], layout=flower_manifest['layout'],
        height_policy='preserve_each_original_mesh_z_bounds_and_5.2m_scaled_trunk',
        collision_policy='visual_and_collision_share_all_meshes_and_poses',
        protected_asset_sha256=before)
    (ASSET / 'scene_manifest.json').write_text(json.dumps(scene, indent=2)+'\n')
    config = json.loads((ROOT / 'polinasi_nav/config/palm_farm_navigation.json').read_text())
    for target in config['trees']:
        matching = next(record for record in records if np.allclose(record['original_pose'][:2], target[:2]))
        target[:2] = matching['pose'][:2]
    (ROOT / 'polinasi_nav/config/palm_farm_flowers_navigation.json').write_text(json.dumps(config, indent=2)+'\n')
    assert protected_hashes() == before, 'protected worlds changed'
    return flower_manifest
