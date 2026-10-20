"""Independent, varied mature oil-palm scene; existing worlds are read-only.

Native metre-scale Collada meshes, deterministic individual trees, and a
documented flowering snapshot estimate rather than a universal field quota.
"""
import argparse
import copy
import hashlib
import json
import math
import random
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np

from generate_palm_farm import Mesh, ROOT, plus, sub, mul, norm, cross, mesh_link
from generate_palm_flowers import ellipsoid, inflorescence
from generate_worlds import add

ASSET = ROOT/'polinasi_nav/models/palm_farm_realistic'
WORLD = ROOT/'polinasi_nav/worlds/palm_farm_realistic.sdf'
SEED = 20261011
REFERENCES = [
    'https://powo.science.kew.org/taxon/urn:lsid:ipni.org:names:666802-1/general-information',
    'https://palmoilis.mpob.gov.my/publications/joprv15n2-henson.pdf',
    'https://journal.umt.edu.my/umtjur/article/view/440',
    'https://jopr.mpob.gov.my/wp-content/uploads/2013/07/joprv11n1-p31.pdf',
    'https://opb.mpob.gov.my/identification-of-etiolated-palms-for-thinning-via-ifsar/',
]


class IndexedMesh(Mesh):
    def write(self, path):
        # Preserve material groups and normals while avoiding triangle-soup XML.
        compact, remap, lookup = IndexedMesh(), [], {}
        compact.colors = self.colors.copy()
        for point, normal in zip(self.vertices, self.normals):
            key = (*point, *normal)
            if key not in lookup:
                lookup[key] = len(compact.vertices)
                compact.vertices.append(point)
                compact.normals.append(normal)
            remap.append(lookup[key])
        compact.faces = {material: [tuple(remap[i] for i in face) for face in faces]
                         for material, faces in self.faces.items()}
        Mesh.write(compact, path)
        compact_collada(path)


def compact_collada(path):
    """Separate position/normal indices; exact serialized geometry is preserved."""
    ns={'c':'http://www.collada.org/2005/11/COLLADASchema'}
    # gz-common's ColladaLoader expects unprefixed COLLADA element names.
    ET.register_namespace('',ns['c'])
    tree=ET.parse(path);root=tree.getroot();maps=[]
    for name in ('positions','normals'):
        source=root.find(f".//c:source[@id='{name}']",ns)
        array=source.find('c:float_array',ns)
        values=np.fromstring(array.text,sep=' ').reshape(-1,3)
        unique,inverse=np.unique(values,axis=0,return_inverse=True)
        precision=5 if name=='positions' else 6
        array.text=' '.join(f'{v:.{precision}f}' for row in unique for v in row)
        array.set('count',str(unique.size))
        source.find('c:technique_common/c:accessor',ns).set('count',str(len(unique)))
        maps.append(inverse)
    for triangles in root.findall('.//c:triangles',ns):
        indices=triangles.find('c:p',ns)
        pairs=np.fromstring(indices.text,sep=' ',dtype=int).reshape(-1,2)
        pairs[:,0]=maps[0][pairs[:,0]];pairs[:,1]=maps[1][pairs[:,1]]
        indices.text=' '.join(str(v) for pair in pairs for v in pair)
    tree.write(path,encoding='utf-8',xml_declaration=True)


def unit(v):
    return mul(v, 1/max(norm(v), 1e-12))


def blade(mesh, base, tip, width, colour, material, bend=.10, twist=.12):
    """Closed curved lanceolate leaflet with a folded midrib and thin edges."""
    d = sub(tip, base)
    side = unit(cross(d, (0, 0, 1)))
    if norm(side) < .5:
        side = (1, 0, 0)
    rings = []
    for t in (0., .28, .65, 1.):
        centre = plus(plus(base, mul(d, t)), (0, 0, -bend*math.sin(math.pi*t)))
        w = width*max(.018, math.sin(math.pi*t)**.65)
        rolled = unit(plus(side, (0, 0, twist*(2*t-1))))
        rings.append([plus(centre, mul(rolled, -w)),
                      plus(centre, (0, 0, w*.22+.0015)),
                      plus(centre, mul(rolled, w)), sub(centre, (0, 0, .0015))])
    for a, b in zip(rings, rings[1:]):
        for j in range(4):
            k = (j+1)%4
            mesh.quad([a[j], b[j], b[k], a[k]], material, colour)
    mesh.quad(list(reversed(rings[0])), material, colour)
    mesh.quad(rings[-1], material, colour)


def terrain_height(x, y):
    # Level launch zone blended into gentle relief; no flat invisible collider.
    blend = min(1., max(0., (math.hypot(x, y)-4.)/5.))
    return blend*(.30*math.sin(x*.095)*math.cos(y*.11)+.16*math.sin((x+y)*.13))


def road_y(x):
    return -4.8+1.5*math.sin(x*.075)


def planting(seed=SEED):
    """Irregular smallholder layout with minimum spacing and deliberate gaps."""
    rng, sites = random.Random(seed), []
    missing = [[23., 11.], [44., -14.]]
    for attempt in range(20000):
        x,y=rng.uniform(-15,56),rng.uniform(-20,20)
        if math.hypot(x,y)<8 or abs(y-road_y(x))<2.6:
            continue
        if any(math.hypot(x-a,y-b)<4.0 for a,b in missing):
            continue
        if any(math.hypot(x-a,y-b)<6.2 for a,b in sites):
            continue
        sites.append((x,y))
        if len(sites)==29:
            break
    if len(sites)!=29:
        raise ValueError('could not place 29 palms with required clearance')
    return sites, missing


def tree_parameters(index, seed=SEED):
    rng = random.Random(seed+104729*(index+1))
    height=rng.uniform(2.0,4.8) if rng.random()<.8 else rng.uniform(4.8,6.6)
    return dict(seed=seed+104729*(index+1), height=height,
        radius=rng.uniform(.23,.29), foot_radius=rng.uniform(.35,.44), lean=rng.uniform(.006,.065),
        lean_angle=rng.uniform(-math.pi,math.pi), retention=rng.uniform(.70,.98),
        fronds=rng.randint(42,52), reach=rng.uniform(4.3,5.8),
        leaf_pairs=rng.randint(80,105), handedness=rng.choice([-1,1]),
        old_fronds=rng.randint(1,4), epiphytes=rng.random()<.55,
        fruit_bunches=rng.randint(1,4), hue=rng.uniform(.84,1.12))


def trunk_centre(p,z):
    bend=p['lean']*z*(.4+.6*z/p['height'])
    return (bend*math.cos(p['lean_angle']),bend*math.sin(p['lean_angle']),z)


def palm(p):
    rng, mesh = random.Random(p['seed']), IndexedMesh()
    h, radius=p['height'],p['radius']
    levels=math.ceil(h/.18)
    for level in range(levels):
        z,z2=h*level/levels,h*(level+1)/levels
        r=(radius*(1-.10*z/h)+(p['foot_radius']-radius)*math.exp(-z/1.0))*(1+.04*math.sin(level*2.7))
        shade=level%5
        colour=(.23+.018*shade,.165+.012*shade,.105+.009*shade)
        mesh.cylinder(trunk_centre(p,z),trunk_centre(p,z2),r,
                      f'bark{shade}',colour,sides=14,tip_radius=r*.99)
        for j in range(8):
            if rng.random()>p['retention']*(.80+.20*z/h):
                continue
            a=j*math.tau/8+level*.39
            radial=(math.cos(a),math.sin(a),0)
            tangent=(-math.sin(a),math.cos(a),0)
            c=plus(trunk_centre(p,z+.10),mul(radial,r+.012))
            base=[plus(c,(0,0,.18)),plus(c,mul(tangent,.12)),
                  sub(c,(0,0,.15)),sub(c,mul(tangent,.12))]
            peak=plus(c,mul(radial,rng.uniform(.09,.16)))
            for k in range(4):
                mesh.triangle([base[k],peak,base[(k+1)%4]],'leaf_base',(.30,.22,.13))
            mesh.quad(list(reversed(base)),'leaf_base',(.30,.22,.13))
            # Broad upturned cut petiole stubs, not coconut-like annular scars.
            stump=plus(c,mul(radial,.075))
            mesh.cylinder(stump,plus(stump,plus(mul(radial,rng.uniform(.12,.22)),(0,0,.16))),
                          .067,'cut_petiole',(.34,.26,.16),sides=6,tip_radius=.050)
            for fibre in range(2):
                f=plus(peak,mul(tangent,rng.uniform(-.06,.06)))
                mesh.cylinder(f,plus(f,plus(mul(radial,.025),(0,0,rng.uniform(.05,.11)))),
                              .004,'fibres',(.24,.18,.105),sides=4,tip_radius=.002)
    crown=trunk_centre(p,h)
    phase=rng.uniform(-math.pi,math.pi)
    for i in range(p['fronds']):
        age=i/(p['fronds']-1)
        a=phase+p['handedness']*i*2.399963+rng.uniform(-.18,.18)
        radial=(math.cos(a),math.sin(a),0)
        side=(-math.sin(a),math.cos(a),0)
        reach=p['reach']*(.12+.88*math.sin(age*math.pi*.70))*rng.uniform(.82,1.1)
        lift=3.7*(1-.65*age)*rng.uniform(.8,1.15)*p.get('frond_lift_factor',1.)
        droop=(.35+3.45*age)*rng.uniform(.85,1.18)*p.get('frond_droop_factor',1.)
        attachment=trunk_centre(p,h-.75*age)
        droop=min(droop,attachment[2]+lift-1.4)
        petiole_radius=rng.uniform(.065,.090)
        sideways=rng.uniform(-.32,.32)*p.get('frond_sideways_factor',1.)
        old=i>=p['fronds']-p['old_fronds']
        def stem(t):
            return plus(attachment,plus(mul(radial,radius+.08+reach*t),
                plus(mul(side,sideways*t*t),(0,0,.16*(1-age)+lift*t-droop*t*t))))
        for k in range(12):
            t=k/12
            mesh.cylinder(stem(t),stem((k+1)/12),petiole_radius*(1-.93*t),
                'old_rachis' if old else 'rachis',(.36,.28,.12) if old else (.29,.35,.10),
                sides=7,tip_radius=petiole_radius*(1-.93*(k+1)/12))
        for k in range(p['leaf_pairs']):
            t=.15+.83*(k+.5)/p['leaf_pairs']
            for sign in (-1,1):
                if rng.random()<(.19 if old else p.get('leaflet_missing_fraction',.025)):
                    continue
                tt=min(.985,max(.12,t+rng.uniform(-.005,.005)))
                length=(.18+.92*math.sin(math.pi*tt))*rng.uniform(.78,1.12)*p.get('leaflet_length_factor',1.)
                base=stem(tt)
                # Irregular two-plane insertion and drooping flexible leaflets.
                vertical=rng.uniform(-.22,.13)-.15*age-.28*tt
                tip=plus(base,plus(mul(side,sign*length),
                    plus(mul(radial,.13+.19*tt),(0,0,vertical))))
                shade=rng.randrange(6)
                colour=(.24+.012*shade,.24+.010*shade,.08) if old else (
                    (.075+.009*shade)*p['hue'],(.23+.025*shade)*p['hue'],.055+.007*shade)
                blade(mesh,base,tip,rng.uniform(.028,.040)*p.get('leaflet_width_factor',1.),colour,
                      f'dry{shade}' if old else f'green{shade}',bend=rng.uniform(.04,.14)*p.get('leaflet_bend_factor',1.),twist=rng.uniform(-.4,.4)*p.get('leaflet_twist_factor',1.))
    # Closed spear leaves at the growing point.
    for i in range(2):
        a=phase+i*2.3
        blade(mesh,crown,plus(crown,(.24*math.cos(a),.24*math.sin(a),2.3-i*.4)),
              .026,(.25,.38,.12),'spear',bend=.03)
    for bunch in range(p['fruit_bunches']):
        a=phase+bunch*2.399963
        c=plus(trunk_centre(p,h-.45),((radius+.18)*math.cos(a),(radius+.18)*math.sin(a),0))
        ripeness=rng.choice(['unripe','ripening','ripe'])
        for n in range(100):
            phi=n*2.399963
            z=(n/99-.5)*.45
            r=.17*math.sqrt(max(.03,1-(z/.25)**2))
            xyz=plus(c,(r*math.cos(phi),r*math.sin(phi),z))
            colour={'unripe':(.12,.09,.06),'ripening':(.34,.13,.055),'ripe':(.58,.20,.035)}[ripeness]
            ellipsoid(mesh,xyz,(.022,.021,.030),ripeness,colour,sides=6,rings=2)
    if p['epiphytes']:
        for i in range(12):
            z=rng.uniform(.8,h*.7);a=rng.uniform(-math.pi,math.pi)
            base=plus(trunk_centre(p,z),(radius*math.cos(a),radius*math.sin(a),0))
            fern(mesh,base,a,rng.uniform(.25,.6),rng)
    return mesh


def fern(mesh,base,angle,length,rng):
    axis=(math.cos(angle),math.sin(angle),.22)
    tangent=(-math.sin(angle),math.cos(angle),0)
    for i in range(7):
        t=(i+1)/8
        p=plus(base,mul(axis,length*t))
        width=length*.30*math.sin(math.pi*t)
        for sign in (-1,1):
            blade(mesh,p,plus(p,plus(mul(tangent,sign*width),mul(axis,.10*length))),
                  .015,(.12,.28,.045),'fern',bend=.016)


def ground(sites,seed):
    rng,mesh=random.Random(seed+19),IndexedMesh()
    # Closed terrain shell. Root positions use exactly this piecewise-linear surface.
    for x in range(-26,63):
        for y in range(-30,31):
            lane=abs(y+.5-road_y(x+.5))<1.45
            shade=int(3+2*math.sin(x*.22)+math.cos(y*.19))%6
            green=not lane and math.sin(x*.14)+math.cos(y*.22)>.12
            colour=(.18+.01*shade,.22+.012*shade,.07+.006*shade) if green else (.29+.008*shade,.20+.007*shade,.12+.004*shade)
            points=[(x,y,terrain_height(x,y)),(x+1,y,terrain_height(x+1,y)),
                    (x+1,y+1,terrain_height(x+1,y+1)),(x,y+1,terrain_height(x,y+1))]
            mesh.quad(points,('cover' if green else 'soil')+str(shade),colour)
            mesh.quad([(x,y,-1),(x,y+1,-1),(x+1,y+1,-1),(x+1,y,-1)],'subsoil',(.25,.17,.09))
            for a,b in ((points[0],points[1]),(points[1],points[2]),(points[2],points[3]),(points[3],points[0])):
                if (a[0]==b[0] and a[0] in (-26,63)) or (a[1]==b[1] and a[1] in (-30,31)):
                    mesh.quad([a,(a[0],a[1],-1),(b[0],b[1],-1),b],'subsoil',(.25,.17,.09))
    for _ in range(1600):
        x,y=rng.uniform(-23,58),rng.uniform(-27,27)
        if math.hypot(x,y)<4 or abs(y-road_y(x))<1.7:
            continue
        if any(math.hypot(x-a,y-b)<1.1 for a,b in sites):
            continue  # Maintained palm circles, patchy inter-row vegetation.
        base=(x,y,surface_height(x,y)+.003)
        if rng.random()<.16:
            fern(mesh,base,rng.uniform(0,math.tau),rng.uniform(.35,.7),rng)
        else:
            for i in range(4):
                a=rng.uniform(0,math.tau);h=rng.uniform(.12,.38)
                blade(mesh,base,plus(base,(.12*math.cos(a),.12*math.sin(a),h)),
                      .013,(.17,.30,.065),'grass',bend=.015)
    for x,y in sites[::3]:
        for i in range(3):
            # Pruned fronds placed off the harvesting path, visibly irregular.
            a=rng.uniform(0,math.tau)
            base=(x+2.0*math.cos(a),y+2.0*math.sin(a),0)
            for k in range(14):
                q=plus(base,(k*.20*math.cos(a),k*.20*math.sin(a),0))
                q=(q[0],q[1],surface_height(q[0],q[1])+.04+i*.04)
                for sign in (-1,1):
                    tip=plus(q,(-sign*.5*math.sin(a),sign*.5*math.cos(a),.035))
                    blade(mesh,q,tip,.024,(.30,.25,.12),'litter',bend=.01)
    return mesh


def surface_height(x,y):
    # Match the diagonal used by Mesh.quad rather than analytical sine at roots.
    a,b=math.floor(x),math.floor(y);u,v=x-a,y-b
    h00,h10,h11,h01=[terrain_height(*p) for p in ((a,b),(a+1,b),(a+1,b+1),(a,b+1))]
    return h00+u*(h10-h00)+v*(h11-h10) if u>=v else h00+v*(h01-h00)+u*(h11-h01)


def flowering_model(total):
    rates={'male':5.40,'female':22.13}
    days={'male':4.5,'female':3.5}
    exposure={s:rates[s]*days[s]/365 for s in rates}
    combined=sum(exposure.values())
    probability=1-math.exp(-combined)
    count=int(total*probability+.5)
    male=int(count*exposure['male']/combined+.5)
    return dict(annual_inflorescences_per_palm=rates,anthesis_window_days=days,
        expected_active_bunches_per_palm=exposure,
        estimated_fraction_palms_with_active_anthesis=probability,
        counts=dict(male=male,female=count-male,without_active_anthesis=total-count),
        interpretation='model_estimate_not_a_reported_field_percentage',
        assumption='stationary_independent_Poisson_events; no seasonality; one represented active bunch per selected palm',
        formula='p=1-exp(-(5.40*4.5+22.13*3.5)/365)',
        evidence=dict(annual_rates_url=REFERENCES[1],table=3,density_palms_per_hectare=160,
                      averaging_years='1989-2001',anthesis_duration_url=REFERENCES[2]))


def preserved_assets():
    files=[]
    for name in ('palm_farm','palm_farm_flowers'):
        files.append(ROOT/f'polinasi_nav/worlds/{name}.sdf')
        files.extend(p for p in (ROOT/f'polinasi_nav/models/{name}').rglob('*') if p.is_file() and p.suffix != '.html')
    return {str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest() for p in files}


def generate(seed=SEED):
    before=preserved_assets()
    sites,missing=planting(seed)
    parameters=[tree_parameters(i,seed) for i in range(len(sites))]
    sdf=ET.parse(ROOT/'polinasi_nav/worlds/palm_farm.sdf')
    world=sdf.getroot().find('world')
    # Maintain internal world name for existing drone/bridge topics.
    for model in list(world.findall('model')):
        if model.get('name','').startswith('oil_palm_') or model.get('name')=='soil_and_grass':
            world.remove(model)
    ground_model=add(world,'model',name='soil_and_grass')
    add(ground_model,'static','true')
    mesh_link(ground_model,'model://palm_farm_realistic/meshes/ground.dae')
    ground(sites,seed).write(ASSET/'meshes/ground.dae')
    model=flowering_model(len(sites))
    rng=random.Random(seed+36035);order=list(range(len(sites)));rng.shuffle(order)
    types={i:('male' if rank<model['counts']['male'] else 'female')
        for rank,i in enumerate(order[:model['counts']['male']+model['counts']['female']])}
    for sex in ('male','female'):
        inflorescence(sex).write(ASSET/f'meshes/flower_{sex}.dae')
    records=[]
    for i,((x,y),p) in enumerate(zip(sites,parameters)):
        name=f'oil_palm_{i:02d}'
        filename=f'palm_{i:02d}.dae'
        mesh=palm(p);triangle_count=sum(len(f) for f in mesh.faces.values())
        mesh.write(ASSET/'meshes'/filename)
        tree=add(world,'model',name=name);add(tree,'static','true')
        yaw=rng.uniform(-math.pi,math.pi);z=surface_height(x,y)
        pose=[x,y,z,0,0,yaw];add(tree,'pose',' '.join(f'{v:.8f}' for v in pose))
        mesh_link(tree,f'model://palm_farm_realistic/meshes/{filename}')
        sex=types.get(i);flowers=[]
        if sex:
            a=rng.uniform(-math.pi,math.pi)
            centre=trunk_centre(p,p['height']-.34)
            local=plus(centre,((p['radius']+.16)*math.cos(a),(p['radius']+.16)*math.sin(a),0))
            flower_pose=[*local,0,rng.uniform(.9,1.2),a]
            link=add(tree,'link',name=f'flower_{sex}_00')
            add(link,'pose',' '.join(f'{v:.8f}' for v in flower_pose))
            scale=rng.uniform(.88,1.12)
            for kind in ('visual','collision'):
                part=add(link,kind,name=kind);geometry=add(part,'geometry');m=add(geometry,'mesh')
                add(m,'uri',f'model://palm_farm_realistic/meshes/flower_{sex}.dae')
                add(m,'scale',f'{scale:.8f} {scale:.8f} {scale:.8f}')
            flowers.append(dict(sex=sex,stage='anthesis',pose=flower_pose,scale=scale))
        records.append(dict(tree=name,pose=pose,mesh=filename,parameters=p,triangles=triangle_count,flowers=flowers))
        print(f'{name}: trunk {p["height"]:.2f} m, {p["fronds"]} fronds, {triangle_count} triangles',flush=True)
    # Independently packaged model family and reusable example tree.
    example=ET.Element('sdf',version='1.9');example.append(copy.deepcopy(world.find("model[@name='oil_palm_00']")))
    example.find('model/pose').text='0 0 0 0 0 0'
    ET.ElementTree(example).write(ASSET/'model.sdf',encoding='utf-8',xml_declaration=True)
    meta=ET.Element('model');add(meta,'name','Varied mature oil-palm plantation')
    add(meta,'version','1.0');add(meta,'sdf','model.sdf',version='1.9')
    add(meta,'description','Independent procedural tree family, irregular layout and collidable gentle terrain.')
    ET.ElementTree(meta).write(ASSET/'model.config',encoding='utf-8',xml_declaration=True)
    world.find('gui/plugin/camera_pose').text='-18 -26 20 0 .44 .68'
    world.find('scene/ambient').text='.42 .44 .40 1'
    world.find('gui/plugin/ambient_light').text='.42 .44 .40'
    ET.indent(sdf,space='  ');sdf.write(WORLD,encoding='utf-8',xml_declaration=True)
    manifest=dict(format_version=1,seed=seed,scenario='palm_farm_realistic',
        condition='photo_guided_stocky_palms_in_irregular_smallholder_layout',total_trees=len(sites),
        missing_planting_sites=missing,trees=records,flowering=model,reference_urls=REFERENCES,
        collision_policy='visual_and_collision_share_all_meshes_and_poses',
        detail_policy='80-105 leaflet pairs per frond, thick petioles and retained leaf bases; photo-guided stocky palm profile',
        terrain_policy='closed triangulated gently undulating shell with level launch zone',
        previous_asset_sha256=before,purpose='procedural_scene_ground_truth_not_navigation_or_perception_results')
    (ASSET/'scene_manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
    config=json.loads((ROOT/'polinasi_nav/config/palm_farm_navigation.json').read_text())
    # Config targets are example trunk viewpoints, not flower ground truth.
    near=sorted(records,key=lambda r:math.hypot(*r['pose'][:2]))[:2]
    config.update(trees=[[*r['pose'][:2],r['pose'][2]+r['parameters']['height']-.4] for r in near],
        bounds_min=[-26,-30,-1.2],bounds_max=[63,31,14.5],
        candidate_heights=[2.0,3.0,4.5,6.0,7.5],exploration_max_altitude=9.0,
        orbit_altitude=near[0]['pose'][2]+near[0]['parameters']['height']-.4,
        mock_flower_events=False)
    (ROOT/'polinasi_nav/config/palm_farm_realistic_navigation.json').write_text(json.dumps(config,indent=2)+'\n')
    assert preserved_assets()==before,'Existing worlds/assets changed!'
    print(json.dumps(dict(world=str(WORLD),trees=len(sites),flowering=model['counts'])),flush=True)
    return manifest


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seed',type=int,default=SEED)
    args=parser.parse_args();generate(args.seed)
