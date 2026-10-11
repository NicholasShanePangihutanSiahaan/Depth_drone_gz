"""Run-local backport of gz-rendering PR #1319; never edits /usr/share.

Only seven sampler address modes change. Scan geometry/data/filtering and
obstacles remain unchanged. Original resources retain their Apache licenses.
"""
import argparse
import hashlib
import json
import re
import shutil
from pathlib import Path

SOURCE = Path('/usr/share/gz/gz-rendering8')
MATERIAL = Path('ogre2/media/materials/scripts/gpu_rays.material')
UPSTREAM = 'https://github.com/gazebosim/gz-rendering/pull/1319'

def clamp_second_pass(text):
    start = text.index('material GpuRaysScan2nd\n')
    end = text.index('// GLSL shaders',start)
    block = text[start:end]
    pattern = r'(texture_unit\s+(cubeUVTex|tex[0-5])\s*\{)([^{}]*)(\})'
    names = []
    def update(match):
        names.append(match[2])
        body = match[3]
        if 'filtering none' not in body:
            raise ValueError('unexpected GPU lidar sampler filtering')
        if 'tex_address_mode' in body:
            if not re.search(r'tex_address_mode\s+clamp\b',body):
                raise ValueError('unexpected explicit GPU lidar sampler mode')
        else:
            body = body.replace('filtering none','filtering none\n        tex_address_mode clamp',1)
        return match[1]+body+match[4]
    fixed = re.sub(pattern,update,block)
    if sorted(names) != sorted(['cubeUVTex',*[f'tex{i}' for i in range(6)]]):
        raise ValueError('expected exactly seven second-pass GPU lidar samplers')
    return text[:start]+fixed+text[end:]

def prepare(destination):
    root = Path(__file__).resolve().parents[1]
    destination = destination.resolve()
    if destination.name!='rendering_resources' or destination.parent.parent!=root/'.dependencies':
        raise ValueError('destination must be a private simulation run resource directory')
    original = (SOURCE/MATERIAL).read_text()
    fixed = clamp_second_pass(original)
    if destination.exists():
        raise ValueError('refusing to overwrite existing rendering resource directory')
    shutil.copytree(SOURCE,destination)
    (destination/MATERIAL).write_text(fixed)
    metadata = dict(upstream=UPSTREAM,source=str(SOURCE),scope='private_Gazebo_process_only',
        source_sha256=hashlib.sha256(original.encode()).hexdigest(),
        fixed_sha256=hashlib.sha256(fixed.encode()).hexdigest(),changed=original!=fixed,
        action='clamp_seven_second_pass_texture_samplers_no_ray_removal')
    (destination/'lidar_sampler_fix.json').write_text(json.dumps(metadata,indent=2)+'\n')
    return metadata

if __name__=='__main__':
    p = argparse.ArgumentParser()
    p.add_argument('destination',type=Path)
    print(json.dumps(prepare(p.parse_args().destination)))
