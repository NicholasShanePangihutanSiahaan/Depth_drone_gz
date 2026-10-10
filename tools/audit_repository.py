"""Static discovery and console-script audit, without importing ROS nodes."""
import ast
import json
from pathlib import Path
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]


def audit():
    result = {'packages': [], 'missing_launch_executables': [], 'invalid_console_scripts': []}
    entrypoints = {}
    for manifest in sorted(ROOT.glob('*/package.xml')):
        directory = manifest.parent
        name = ET.parse(manifest).findtext('name')
        ignored = (directory/'COLCON_IGNORE').exists()
        result['packages'].append({'name': name, 'path': str(directory.relative_to(ROOT)), 'ignored': ignored})
        setup = directory/'setup.py'
        if setup.exists():
            for node in ast.walk(ast.parse(setup.read_text())):
                if isinstance(node, ast.Constant) and isinstance(node.value, str) and ' = ' in node.value and ':' in node.value:
                    executable, target = map(str.strip, node.value.split('=', 1))
                    module, function = target.split(':')
                    entrypoints.setdefault(name, set()).add(executable)
                    file = directory/Path(*module.split('.')).with_suffix('.py')
                    if not file.exists() or not any(isinstance(n, (ast.FunctionDef, ast.AsyncFunctionDef)) and n.name == function for n in ast.walk(ast.parse(file.read_text()))):
                        result['invalid_console_scripts'].append({'package': name, 'executable': executable, 'target': target})
    for launch in sorted(ROOT.glob('*/launch/*.launch.py')):
        tree = ast.parse(launch.read_text())
        for node in ast.walk(tree):
            if isinstance(node, ast.Call) and isinstance(node.func, ast.Name) and node.func.id == 'Node':
                values = {k.arg: k.value.value for k in node.keywords if isinstance(k.value, ast.Constant)}
                package, executable = values.get('package'), values.get('executable')
                if package in entrypoints and executable and executable not in entrypoints[package]:
                    result['missing_launch_executables'].append({'launch': str(launch.relative_to(ROOT)), 'package': package, 'executable': executable})
    return result


if __name__ == '__main__':
    result = audit()
    print(json.dumps(result, indent=2))
    raise SystemExit(bool(result['invalid_console_scripts'] or result['missing_launch_executables']))
