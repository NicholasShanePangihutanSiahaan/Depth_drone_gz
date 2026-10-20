"""Prepare reproducible flower-farm resources and record their exact hashes."""
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]


def resource_paths(project):
    project = Path(project)
    world = project/'polinasi_nav/worlds/palm_farm_flowers.sdf'
    if not world.is_file():
        return [world]
    paths = {world}
    for uri in ET.parse(world).getroot().iter('uri'):
        value = uri.text or ''
        if value.startswith('model://palm_farm_flowers/'):
            paths.add(project/'polinasi_nav/models'/value.removeprefix('model://'))
    if len(paths) == 1:
        raise ValueError('flower world contains no flower-farm mesh resources')
    return sorted(paths)


def audit_assets(project):
    paths = resource_paths(project)
    missing = [str(p) for p in paths if not p.is_file() or p.stat().st_size == 0]
    if missing:
        raise FileNotFoundError('Missing flower-farm resources: '+', '.join(missing[:4]))
    hashes = {}
    for path in paths:
        digest = hashlib.sha256()
        with path.open('rb') as source:
            for block in iter(lambda: source.read(1024*1024), b''):
                digest.update(block)
        hashes[str(path.relative_to(project))] = digest.hexdigest()
    world = ET.parse(Path(project)/'polinasi_nav/worlds/palm_farm_flowers.sdf').getroot().find('world')
    return dict(scenario='palm_farm_flowers', world_name=world.get('name'),
                tree_count=sum(m.get('name', '').startswith('oil_palm_') for m in world.findall('model')),
                resource_sha256=hashes,
                purpose='simulation_resource_provenance_only_not_navigation_input')


def main():
    if any(not p.is_file() or p.stat().st_size == 0 for p in resource_paths(ROOT)):
        from generate_palm_flowers import generate
        generate()
    audit = audit_assets(ROOT)
    output = ROOT/'reports/palm_flowers/asset_audit.json'
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(audit, indent=2)+'\n')
    print(f'Flower world: {audit["tree_count"]} trees; resource audit: {output}', flush=True)


if __name__ == '__main__':
    main()
