"""Deterministic native mesh assets: no downloaded models or remote dependencies.

Run this generator to recreate the committed Collada/SDF farm assets. Meshes
are real geometry, including leaf thickness; visual and collision mesh match.
"""
import json
import math
import random
import sys
from pathlib import Path
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'polinasi_nav'))
from polinasi_nav.config import load_config
from generate_worlds import add

NS = 'http://www.collada.org/2005/11/COLLADASchema'


class Mesh:
    def __init__(self):
        self.vertices = []
        self.normals = []
        self.faces = {}
        self.colors = {}

    def triangle(self, points, material, color):
        offset = len(self.vertices)
        self.vertices.extend(points)
        normal = cross(sub(points[1], points[0]), sub(points[2], points[0]))
        normal = mul(normal, 1/max(norm(normal), 1e-12))
        self.normals.extend([normal]*3)
        self.faces.setdefault(material, []).append((offset, offset+1, offset+2))
        self.colors[material] = color

    def quad(self, points, material, color):
        self.triangle([points[0], points[1], points[2]], material, color)
        self.triangle([points[0], points[2], points[3]], material, color)

    def cylinder(self, start, end, radius, material, color, sides=8, tip_radius=None):
        axis = sub(end, start)
        length = norm(axis)
        axis = mul(axis, 1/length)
        cross_axis = cross(axis, (0, 0, 1) if abs(axis[2]) < .9 else (1, 0, 0))
        u = mul(cross_axis, 1/norm(cross_axis))
        v = cross(axis, u)
        rings = []
        for centre, r in [(start, radius), (end, radius if tip_radius is None else tip_radius)]:
            rings.append([plus(centre, plus(mul(u, r*math.cos(i*math.tau/sides)),
                                           mul(v, r*math.sin(i*math.tau/sides)))) for i in range(sides)])
        for i in range(sides):
            j = (i+1) % sides
            self.quad([rings[0][i], rings[0][j], rings[1][j], rings[1][i]], material, color)
            self.triangle([start, rings[0][j], rings[0][i]], material, color)
            self.triangle([end, rings[1][i], rings[1][j]], material, color)

    def leaf(self, base, tip, width, material, color):
        """Closed tapered six-vertex blade; visible and collidable on both sides."""
        d = sub(tip, base)
        u = (-d[1], d[0], 0)
        u = mul(u, width / max(norm(u), 1e-9))
        mid = plus(base, mul(d, .35))
        blade = [base, plus(mid, u), tip, sub(mid, u)]
        upper = [plus(p, (0, 0, .006)) for p in blade]
        lower = [sub(p, (0, 0, .006)) for p in blade]
        self.quad(upper, material, color)
        self.quad(list(reversed(lower)), material, color)
        for i in range(4):
            j = (i+1) % 4
            self.quad([upper[i], lower[i], lower[j], upper[j]], material, color)

    def write(self, path):
        ET.register_namespace('', NS)
        def element(parent, tag, text=None, **attrs):
            e = ET.SubElement(parent, f'{{{NS}}}{tag}', attrs)
            if text is not None:
                e.text = str(text)
            return e
        root = ET.Element(f'{{{NS}}}COLLADA', version='1.4.1')
        asset = element(root, 'asset')
        element(asset, 'created', '2026-10-09T00:00:00Z')
        element(asset, 'modified', '2026-10-09T00:00:00Z')
        element(asset, 'unit', name='meter', meter='1')
        element(asset, 'up_axis', 'Z_UP')
        effects, materials = element(root, 'library_effects'), element(root, 'library_materials')
        for name, color in self.colors.items():
            effect = element(effects, 'effect', id=name+'-fx')
            technique = element(element(effect, 'profile_COMMON'), 'technique', sid='common')
            phong = element(technique, 'phong')
            for prop, rgba in [('ambient', (*color, 1)), ('diffuse', (*color, 1)),
                               ('specular', (.025, .025, .025, 1))]:
                element(element(phong, prop), 'color', ' '.join(map(str, rgba)))
            element(element(phong, 'shininess'), 'float', '5')
            material = element(materials, 'material', id=name+'-mat', name=name)
            element(material, 'instance_effect', url='#'+name+'-fx')
        geometry = element(element(root, 'library_geometries'), 'geometry', id='surface', name='surface')
        mesh = element(geometry, 'mesh')
        source = element(mesh, 'source', id='positions')
        element(source, 'float_array', ' '.join(f'{x:.5f}' for p in self.vertices for x in p),
                id='position-array', count=str(len(self.vertices)*3))
        accessor = element(element(source, 'technique_common'), 'accessor',
                           source='#position-array', count=str(len(self.vertices)), stride='3')
        for name in 'XYZ':
            element(accessor, 'param', name=name, type='float')
        source = element(mesh, 'source', id='normals')
        element(source, 'float_array', ' '.join(f'{x:.6f}' for p in self.normals for x in p),
                id='normal-array', count=str(len(self.normals)*3))
        accessor = element(element(source, 'technique_common'), 'accessor',
                           source='#normal-array', count=str(len(self.normals)), stride='3')
        for name in 'XYZ':
            element(accessor, 'param', name=name, type='float')
        vertices = element(mesh, 'vertices', id='vertices')
        element(vertices, 'input', semantic='POSITION', source='#positions')
        for name, faces in self.faces.items():
            triangles = element(mesh, 'triangles', material=name, count=str(len(faces)))
            element(triangles, 'input', semantic='VERTEX', source='#vertices', offset='0')
            element(triangles, 'input', semantic='NORMAL', source='#normals', offset='1')
            element(triangles, 'p', ' '.join(f'{i} {i}' for face in faces for i in face))
        visual_scene = element(element(root, 'library_visual_scenes'), 'visual_scene', id='Scene')
        node = element(visual_scene, 'node', id='mesh-node', name='mesh-node', type='NODE')
        instance = element(node, 'instance_geometry', url='#surface')
        common = element(element(instance, 'bind_material'), 'technique_common')
        for name in self.colors:
            element(common, 'instance_material', symbol=name, target='#'+name+'-mat')
        element(element(root, 'scene'), 'instance_visual_scene', url='#Scene')
        path.parent.mkdir(parents=True, exist_ok=True)
        ET.ElementTree(root).write(path, encoding='utf-8', xml_declaration=True)


def plus(a, b): return tuple(x+y for x, y in zip(a, b))
def sub(a, b): return tuple(x-y for x, y in zip(a, b))
def mul(a, s): return tuple(x*s for x in a)
def norm(a): return math.sqrt(sum(x*x for x in a))
def cross(a, b): return (a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0])


def palm(seed=7):
    rng, mesh = random.Random(seed), Mesh()
    height = 5.2
    # Tapered trunk, annular ridges, and diamond-shaped retained leaf bases.
    for level in range(26):
        z = level*.2
        r = .33-.10*z/height
        mesh.cylinder((.035*math.sin(z), 0, z), (.035*math.sin(z+.2), 0, z+.2),
                      r, f'bark{level%3}', (.25+.035*(level%3), .15+.02*(level%3), .075),
                      sides=12, tip_radius=r-.004)
        for j in range(9):
            angle = j*math.tau/9+level*.33
            radial = (math.cos(angle), math.sin(angle), 0)
            tangent = (-math.sin(angle), math.cos(angle), 0)
            centre = plus(mul(radial, r+.025), (0, 0, z+.09))
            vertices = [plus(centre, (0, 0, .13)), plus(centre, mul(tangent, .065)),
                        plus(centre, (0, 0, -.12)), sub(centre, mul(tangent, .065))]
            peak = plus(centre, mul(radial, .055))
            for k in range(4):
                mesh.triangle([vertices[k], peak, vertices[(k+1)%4]], 'leaf_bases', (.38, .25, .12))
    # Radial arching fronds, several ages/heights, each with paired leaflets.
    for frond in range(18):
        angle = frond*math.tau/18+rng.uniform(-.10, .10)
        radial = (math.cos(angle), math.sin(angle), 0)
        side = (-math.sin(angle), math.cos(angle), 0)
        tier = frond % 3
        reach = [3.7, 3.1, 2.3][tier]+rng.uniform(-.2, .2)
        crown = height + .12*tier
        def stem(t):
            return plus(mul(radial, .1+reach*t), (0, 0, crown+2.5*t-(3.8-.75*tier)*t*t))
        for k in range(10):
            mesh.cylinder(stem(k/10), stem((k+1)/10), .038*(1-.8*k/10),
                          'rachis', (.38, .43, .095), sides=5, tip_radius=.038*(1-.8*(k+1)/10))
        for k in range(1, 18):
            t = k/18
            length = (.38+1.1*math.sin(math.pi*t))*(1-.22*tier)
            for sign in (-1, 1):
                base = stem(t)
                tip = plus(base, plus(mul(side, sign*length),
                                      plus(mul(radial, .18+.25*t), (0, 0, -.24-.38*t))))
                shade = (frond+k) % 5
                color = (.07+.019*shade, .22+.035*shade, .035+.01*shade)
                mesh.leaf(base, tip, .045+.025*math.sin(math.pi*t), f'green{shade}', color)
    # Visible orange/red fruit bunches near crown; mock events remain software.
    for bunch in range(3):
        angle = bunch*math.tau/3+.5
        centre = (.43*math.cos(angle), .43*math.sin(angle), height-.4)
        for n in range(30):
            phi = n*2.399963
            z = (n/29-.5)*.48
            radius = .21*math.sqrt(max(0, 1-(z/.28)**2))
            p = plus(centre, (radius*math.cos(phi), radius*math.sin(phi), z))
            mesh.cylinder(sub(p, (0, 0, .045)), plus(p, (0, 0, .045)), .045,
                          f'fruit{n%3}', [( .52, .075, .025), (.80, .20, .035), (.32, .035, .025)][n%3],
                          sides=5, tip_radius=.025)
    return mesh


def ground():
    rng, mesh = random.Random(19), Mesh()
    # Colour-varied small polygons are native materials, not external textures.
    for x in range(-40, 51, 2):
        for y in range(-40, 41, 2):
            shade = rng.randrange(6)
            lane = abs(y+4) < 2 or (-2 < x < 2 and -3 < y < 3)
            palette = [( .30+.005*shade, .19+.004*shade, .095+.003*shade),
                       (.19+.004*shade, .24+.004*shade, .085+.003*shade)]
            # Coherent patches, not randomly alternating checkerboard tiles.
            green = not lane and (math.sin(x*.17)+math.cos(y*.21)+math.sin((x+y)*.36)) > .3
            color = palette[int(green)]
            name = ('grass_ground' if green else 'earth')+str(shade)
            mesh.quad([(x, y, .002), (x+2, y, .002), (x+2, y+2, .002), (x, y+2, .002)], name, color)
    # Low scattered grass away from launch pad. Geometry is also collidable.
    for _ in range(900):
        x, y = rng.uniform(-38, 50), rng.uniform(-38, 40)
        if math.hypot(x, y) < 3 or abs(y+4) < 1.5:
            continue
        h = rng.uniform(.05, .16)
        for angle in (0, 1.05, 2.1):
            mesh.leaf((x, y, .003), (x+.04*math.cos(angle), y+.04*math.sin(angle), h),
                      .022, 'grass_blades', (.22, .32, .065))
    return mesh


def mesh_link(model, uri, ground_plane=False, scale=1.):
    link = add(model, 'link', name='surface')
    for kind in ('visual', 'collision'):
        part = add(link, kind, name=kind)
        geometry = add(part, 'geometry')
        mesh = add(geometry, 'mesh')
        add(mesh, 'uri', uri)
        add(mesh, 'scale', f'{scale} {scale} {scale}')
        if kind == 'visual':
            add(part, 'cast_shadows', 'true')
    if ground_plane:
        part = add(link, 'collision', name='solid_ground')
        add(part, 'pose', '5 0 -0.25 0 0 0')
        add(add(add(part, 'geometry'), 'box'), 'size', '92 82 0.5')


def create_assets():
    asset_dir = ROOT/'polinasi_nav/models/palm_farm'
    asset_dir.mkdir(parents=True, exist_ok=True)
    palm().write(asset_dir/'meshes/palm.dae')
    ground().write(asset_dir/'meshes/ground.dae')
    # Standalone reusable palm model, also referenced by farm-world mesh URIs.
    sdf = ET.Element('sdf', version='1.9')
    model = add(sdf, 'model', name='oil_palm')
    add(model, 'static', 'true')
    mesh_link(model, 'model://palm_farm/meshes/palm.dae')
    ET.indent(sdf, space='  ')
    ET.ElementTree(sdf).write(asset_dir/'model.sdf', encoding='utf-8', xml_declaration=True)
    metadata = ET.Element('model')
    add(metadata, 'name', 'Oil palm farm assets')
    add(metadata, 'version', '1.0')
    add(metadata, 'sdf', 'model.sdf', version='1.9')
    add(metadata, 'description', 'Procedural palms and ground; metre-scale static visual/collision meshes.')
    ET.ElementTree(metadata).write(asset_dir/'model.config', encoding='utf-8', xml_declaration=True)
    c = load_config()
    c.update({'bounds_min': [-5., -9., -.6], 'bounds_max': [25., 14., 9.4],
              'resolution': .2, 'trees': [[9., 0., 4.8], [18., 4.5, 4.8]],
              'orbit_radius': 4.2, 'orbit_altitude': 4.8,
              'candidate_distances': [4.2, 5.0], 'candidate_heights': [3.8, 4.8, 6.0],
              'camera_max_distance': 6.5, 'exploration_max_altitude': 7.0})
    (ROOT/'polinasi_nav/config/palm_farm_navigation.json').write_text(json.dumps(c, indent=2)+'\n')
    sdf = ET.Element('sdf', version='1.9')
    w = add(sdf, 'world', name='polinasi_palm_farm')
    physics = add(w, 'physics', name='1ms', type='ignore')
    add(physics, 'max_step_size', '.001')
    add(physics, 'real_time_factor', '1')
    for library, system in [('physics', 'Physics'), ('sensors', 'Sensors'), ('imu', 'Imu'),
                            ('user-commands', 'UserCommands'), ('scene-broadcaster', 'SceneBroadcaster')]:
        plugin = add(w, 'plugin', filename=f'gz-sim-{library}-system', name=f'gz::sim::systems::{system}')
        if system == 'Sensors':
            add(plugin, 'render_engine', 'ogre2')
    scene = add(w, 'scene')
    add(scene, 'ambient', '.55 .55 .55 1')
    add(scene, 'background', '.58 .76 .91 1')
    add(scene, 'shadows', 'true')
    light = add(w, 'light', name='tropical_sun', type='directional')
    add(light, 'pose', '0 0 30 0 0 0')
    add(light, 'diffuse', '.95 .91 .82 1')
    add(light, 'specular', '.15 .15 .15 1')
    add(light, 'cast_shadows', 'true')
    add(light, 'direction', '-.4 -.35 -1')
    ground_model = add(w, 'model', name='soil_and_grass')
    add(ground_model, 'static', 'true')
    mesh_link(ground_model, 'model://palm_farm/meshes/ground.dae', ground_plane=True)
    vehicle = add(w, 'include')
    add(vehicle, 'uri', 'model://iris_lidar')
    add(vehicle, 'pose', '0 0 .195 0 0 0')
    # Palms in staggered rows; launch area remains clear.
    positions = [(9., 0.), (18., 4.5)]
    for row in range(-2, 3):
        for col in range(-1, 5):
            x, y = 9.*col+4.5*(abs(row)%2), 9.*row
            if math.hypot(x, y) < 6 or any(math.hypot(x-a, y-b) < 6 for a, b in positions):
                continue
            positions.append((x, y))
    rng = random.Random(41)
    for i, (x, y) in enumerate(positions):
        model = add(w, 'model', name=f'oil_palm_{i:02d}')
        add(model, 'static', 'true')
        add(model, 'pose', f'{x} {y} 0 0 0 {rng.uniform(-math.pi, math.pi):.5f}')
        mesh_link(model, 'model://palm_farm/meshes/palm.dae', scale=1. if i < 2 else rng.uniform(.88, 1.13))
    # A visible launch pad: explicit diffuse fixes the old ambient-only black visuals.
    model = add(w, 'model', name='launch_pad')
    add(model, 'static', 'true')
    link = add(model, 'link', name='pad')
    for kind in ('visual', 'collision'):
        part = add(link, kind, name=kind)
        add(part, 'pose', '0 0 .006 0 0 0')
        add(add(add(part, 'geometry'), 'box'), 'size', '2.8 2.8 .012')
        if kind == 'visual':
            material = add(part, 'material')
            add(material, 'ambient', '.35 .37 .34 1')
            add(material, 'diffuse', '.55 .57 .52 1')
    gui = add(w, 'gui', fullscreen='false')
    plugin = add(gui, 'plugin', filename='MinimalScene', name='3D View')
    props = add(plugin, 'gz-gui')
    add(props, 'property', 'false', type='bool', key='showTitleBar')
    add(props, 'property', 'docked', type='string', key='state')
    add(plugin, 'engine', 'ogre2')
    add(plugin, 'scene', 'scene')
    add(plugin, 'ambient_light', '.55 .55 .55')
    add(plugin, 'background_color', '.58 .76 .91')
    add(plugin, 'camera_pose', '-12 -18 16 0 .55 .8')
    # Standard viewer interactions and transport scene updates.
    for name in ('GzSceneManager', 'InteractiveViewControl', 'CameraTracking', 'WorldControl', 'WorldStats'):
        plugin = add(gui, 'plugin', filename=name, name=name)
        props = add(plugin, 'gz-gui')
        add(props, 'property', 'false', type='bool', key='showTitleBar')
        add(props, 'property', 'floating', type='string', key='state')
        add(props, 'property', 'false', type='bool', key='resizable')
        if name in ('WorldControl', 'WorldStats'):
            add(props, 'property', '72' if name == 'WorldControl' else '110', type='double', key='height')
            add(props, 'property', '290', type='double', key='width')
            add(props, 'property', '1', type='double', key='z')
            anchors = add(props, 'anchors', target='3D View')
            edge = 'left' if name == 'WorldControl' else 'right'
            add(anchors, 'line', own=edge, target=edge)
            add(anchors, 'line', own='bottom', target='bottom')
            if name == 'WorldControl':
                add(plugin, 'play_pause', 'true')
                add(plugin, 'step', 'true')
                add(plugin, 'start_paused', 'false')
                add(plugin, 'use_event', 'true')
            else:
                for stat in ('sim_time', 'real_time', 'real_time_factor', 'iterations'):
                    add(plugin, stat, 'true')
        else:
            add(props, 'property', '5', type='double', key='width')
            add(props, 'property', '5', type='double', key='height')
    ET.indent(sdf, space='  ')
    ET.ElementTree(sdf).write(ROOT/'polinasi_nav/worlds/palm_farm.sdf', encoding='utf-8', xml_declaration=True)
    print(f'Generated {len(positions)} palms; mesh assets in {asset_dir}')


if __name__ == '__main__':
    create_assets()
