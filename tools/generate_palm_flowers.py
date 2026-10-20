"""Create the flower farm with varied realistic palms and mild planting jitter.

Meshes are authored here, not copied from reference photographs. SDF visual and
collision use the same closed geometry. The manifest is scene ground truth for
asset audits, never sensor detections or navigation goals.
"""
import argparse
import json
import math
import random
import shutil
from pathlib import Path
import xml.etree.ElementTree as ET

from generate_palm_farm import Mesh, ROOT, plus, sub, mul, norm, cross
from generate_worlds import add

DEFAULT_SEED = 36035
SOURCE_WORLD = ROOT/'polinasi_nav/worlds/palm_farm.sdf'
OUTPUT_WORLD = ROOT/'polinasi_nav/worlds/palm_farm_flowers.sdf'
ASSET = ROOT/'polinasi_nav/models/palm_farm_flowers'
REFERENCES = [
    'https://powo.science.kew.org/taxon/urn:lsid:ipni.org:names:666802-1/general-information',
    'https://journal.umt.edu.my/umtjur/article/view/440',
    'https://horizon.documentation.ird.fr/exl-doc/pleins_textes/divers21-03/010081641.pdf',
]


def ellipsoid(mesh, centre, radii, material, colour, sides=6, rings=3):
    """Closed low-poly florets, with explicit poles and no degenerate faces."""
    levels = []
    for level in range(1, rings+1):
        latitude = math.pi*level/(rings+1)
        levels.append([plus(centre, (radii[0]*math.sin(latitude)*math.cos(i*math.tau/sides),
                                    radii[1]*math.sin(latitude)*math.sin(i*math.tau/sides),
                                    radii[2]*math.cos(latitude))) for i in range(sides)])
    top, bottom = plus(centre, (0, 0, radii[2])), sub(centre, (0, 0, radii[2]))
    for i in range(sides):
        j = (i+1) % sides
        mesh.triangle([top, levels[0][i], levels[0][j]], material, colour)
        mesh.triangle([bottom, levels[-1][j], levels[-1][i]], material, colour)
        for first, second in zip(levels, levels[1:]):
            mesh.quad([first[i], second[i], second[j], first[j]], material, colour)


def inflorescence(sex):
    """Metre-scale axillary bunch: peduncle, rachis, rachillae, and florets."""
    if sex not in ('male', 'female'):
        raise ValueError('sex must be male or female')
    mesh, rng = Mesh(), random.Random(71 if sex == 'male' else 83)
    stem_colour = (.30, .35, .09)
    mesh.cylinder((0, 0, 0), (0, 0, .20), .027, 'peduncle', stem_colour,
                  sides=10, tip_radius=.023)
    # Split remnants of the protective spathe at the leaf-axil attachment.
    for a in (0., 2.1, 4.2):
        mesh.leaf((0, 0, .08), (.075*math.cos(a), .075*math.sin(a), .32),
                  .024, 'spathe', (.33, .24, .11))
    if sex == 'male':
        mesh.cylinder((0, 0, .18), (0, 0, .59), .028, 'rachis', stem_colour,
                      sides=10, tip_radius=.012)
        # About 50 slender, finger-like spikelets; anthesis is pale yellow.
        for i in range(48):
            a = i*2.399963
            base = (.026*math.cos(a), .026*math.sin(a), .22+.34*i/47)
            length = rng.uniform(.13, .20)
            axis = (.87*math.cos(a), .87*math.sin(a), .49)
            tip = plus(base, mul(axis, length))
            mesh.cylinder(base, tip, .008, 'male_spikelet', (.65, .55, .20),
                          sides=7, tip_radius=.0045)
            u = (-math.sin(a), math.cos(a), 0.)
            v = cross(axis, u)
            for j in range(28):
                phase = j*2.399963
                p = plus(plus(base, mul(axis, length*(j+.5)/28)),
                         plus(mul(u, .008*math.cos(phase)), mul(v, .008*math.sin(phase))))
                ellipsoid(mesh, p, (.0028, .0028, .0033), f'male_anthesis{j%3}',
                          [( .88, .73, .30), (.96, .86, .53), (.78, .64, .24)][j%3], rings=1)
            mesh.cylinder(tip, plus(tip, mul(axis, .008)), .0025,
                          'spikelet_tip', (.44, .37, .11), sides=5, tip_radius=.0008)
    else:
        # Female rachillae are shorter; a broad, compact ovoid receptacle.
        ellipsoid(mesh, (0, 0, .37), (.052, .052, .16), 'female_core',
                  (.35, .24, .12), sides=12, rings=6)
        mesh.cylinder((0, 0, .18), (0, 0, .57), .032, 'rachis', stem_colour,
                      sides=10, tip_radius=.012)
        for i in range(48):
            a = i*2.399963
            z = .23+.28*i/47
            envelope = math.sqrt(max(.1, 1-((z-.37)/.19)**2))
            base = (.055*envelope*math.cos(a), .055*envelope*math.sin(a), z)
            axis = (.92*math.cos(a), .92*math.sin(a), .39)
            length = rng.uniform(.105, .15)*envelope
            tip = plus(base, mul(axis, length))
            mesh.cylinder(base, tip, .009, 'female_rachilla', (.38, .27, .12),
                          sides=6, tip_radius=.006)
            tangent = (-math.sin(a), math.cos(a), 0.)
            for j in range(12):
                p = plus(plus(base, mul(axis, length*(j+.6)/12)),
                         mul(tangent, (-1 if j%2 else 1)*.009))
                ellipsoid(mesh, p, (.006, .006, .008), 'female_flower_base',
                          (.42, .24, .17), rings=2)
                # Three cream stigma lobes distinguish receptive flowers from
                # the existing orange/red mature fruit bunches in palm.dae.
                stigma = plus(p, mul(axis, .009))
                for k in range(3):
                    b = a+k*math.tau/3
                    end = plus(stigma, (.006*math.cos(b), .006*math.sin(b), .004))
                    mesh.cylinder(stigma, end, .0026, 'female_receptive_stigma',
                                  (.94, .83, .64), sides=5, tip_radius=.0017)
            mesh.cylinder(tip, plus(tip, mul(axis, .016)), .003,
                          'female_bract_tip', (.37, .26, .13), sides=5, tip_radius=.0008)
    return mesh


def flowering_counts(total):
    """Nearest equal integer quotas for 35% male + 35% female trees."""
    per_sex = int(math.floor(total*.35+.5))
    return per_sex, per_sex


def add_flowers(world, seed=DEFAULT_SEED):
    """Modify only flower links; preserve existing models, poses and geometry."""
    palms = sorted((m for m in world.findall('model') if m.get('name', '').startswith('oil_palm_')),
                   key=lambda m: m.get('name'))
    if not palms:
        raise ValueError('world contains no oil_palm_* models')
    rng = random.Random(seed)
    shuffled = list(palms)
    rng.shuffle(shuffled)
    male_count, female_count = flowering_counts(len(palms))
    assignments = {m.get('name'): ('male' if i < male_count else 'female')
                   for i, m in enumerate(shuffled[:male_count+female_count])}
    entries = []
    for tree in palms:
        # Regeneration replaces only links owned by this generator.
        for link in list(tree.findall('link')):
            if link.get('name', '').startswith('flower_'):
                tree.remove(link)
        name = tree.get('name')
        sex = assignments.get(name)
        entry = dict(tree=name, sex=sex, flowers=[])
        entries.append(entry)
        if sex is None:
            continue
        scale = float(tree.findtext('link/visual/geometry/mesh/scale').split()[0])
        tree_pose = list(map(float, tree.findtext('pose').split()))
        count, start_angle = rng.randint(1, 3), rng.uniform(-math.pi, math.pi)
        for index in range(count):
            a = start_angle+index*math.tau/count+rng.uniform(-.18, .18)
            # Attached to the upper trunk, below the crown. Rotation about Y
            # bends the bunch outward; it is not a floating navigation target.
            radius = .235*scale
            p = [radius*math.cos(a), radius*math.sin(a), rng.uniform(4.82, 5.02)*scale]
            pitch = rng.uniform(.85, 1.15)
            pose = [*p, 0., pitch, a]
            bunch_scale = scale*rng.uniform(.9, 1.1)
            link_name = f'flower_{sex}_{index:02d}'
            link = add(tree, 'link', name=link_name)
            add(link, 'pose', ' '.join(f'{v:.8f}' for v in pose))
            uri = f'model://palm_farm_flowers/meshes/flower_{sex}.dae'
            for kind in ('visual', 'collision'):
                part = add(link, kind, name=kind)
                mesh = add(add(part, 'geometry'), 'mesh')
                add(mesh, 'uri', uri)
                add(mesh, 'scale', ' '.join([f'{bunch_scale:.8f}']*3))
                if kind == 'visual':
                    add(part, 'cast_shadows', 'true')
            yaw = tree_pose[5]
            world_p = [tree_pose[0]+math.cos(yaw)*p[0]-math.sin(yaw)*p[1],
                       tree_pose[1]+math.sin(yaw)*p[0]+math.cos(yaw)*p[1], tree_pose[2]+p[2]]
            entry['flowers'].append(dict(link=link_name, mesh=uri, local_pose=pose,
                scale=bunch_scale, attachment_world_xyz=world_p))
    return dict(format_version=1, seed=seed, total_trees=len(palms),
        requested_tree_fractions=dict(male=.35, female=.35, non_flowering=.30),
        counts=dict(male=male_count, female=female_count, non_flowering=len(palms)-male_count-female_count),
        realised_flowering_fraction=(male_count+female_count)/len(palms),
        rounding_policy='nearest_equal_integer_counts_per_sex',
        bunch_count=sum(len(e['flowers']) for e in entries), trees=entries,
        collision_policy='same_closed_triangle_mesh_as_visual',
        reference_urls=REFERENCES,
        purpose='simulation_scene_ground_truth_only_not_perception_or_navigation_input')


def generate(world_path=None, seed=DEFAULT_SEED, source=SOURCE_WORLD):
    """Create a second farm; never overwrite the source world or its assets."""
    world_path = Path(world_path or OUTPUT_WORLD)
    source = Path(source)
    if world_path.resolve() == source.resolve() or world_path.resolve() == SOURCE_WORLD.resolve():
        raise ValueError('output must be a separate world, never the original palm_farm')
    sdf = ET.parse(source)
    manifest = add_flowers(sdf.getroot().find('world'), seed)
    ASSET.mkdir(parents=True, exist_ok=True)
    shutil.copytree(ROOT/'polinasi_nav/models/palm_farm/meshes', ASSET/'meshes', dirs_exist_ok=True)
    for name in ('model.config', 'model.sdf'):
        text = (ROOT/'polinasi_nav/models/palm_farm'/name).read_text()
        (ASSET/name).write_text(text.replace('model://palm_farm/', 'model://palm_farm_flowers/'))
    # All farm geometry resolves to independent copies; the drone stays shared.
    for uri in sdf.getroot().iter('uri'):
        if uri.text and uri.text.startswith('model://palm_farm/'):
            uri.text = uri.text.replace('model://palm_farm/', 'model://palm_farm_flowers/', 1)
    for sex in ('male', 'female'):
        inflorescence(sex).write(ASSET/f'meshes/flower_{sex}.dae')
    ET.indent(sdf, space='  ')
    world_path.parent.mkdir(parents=True, exist_ok=True)
    sdf.write(world_path, encoding='utf-8', xml_declaration=True)
    (ASSET/'flower_manifest.json').write_text(json.dumps(manifest, indent=2)+'\n')
    # Import lazily: the realistic morphology module also uses inflorescence().
    from realistic_flower_farm import upgrade
    manifest = upgrade(world_path, manifest, seed)
    return manifest


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--world', type=Path, default=OUTPUT_WORLD)
    parser.add_argument('--seed', type=int, default=DEFAULT_SEED)
    args = parser.parse_args()
    result = generate(args.world, args.seed)
    print(json.dumps({k: result[k] for k in ('seed', 'total_trees', 'counts', 'bunch_count')}))
