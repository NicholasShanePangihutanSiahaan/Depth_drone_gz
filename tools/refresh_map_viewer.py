"""Create a new path-toggle viewer without changing the original saved log."""
import argparse
import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'polinasi_nav'))
from polinasi_nav.map_log import VIEWER


def refresh(source, output, config=None):
    source, output = Path(source), Path(output)
    if output.resolve() in (source.resolve(), source.with_suffix('.html').resolve()):
        raise ValueError('keep the original log: choose a separate HTML output')
    data = json.loads(source.read_text())
    if config is not None and 'flight_paths' not in data:
        # Use the exact run manifest, not today's potentially edited preset.
        configuration = json.loads(Path(config).read_text())
        waypoints = configuration.get('survey_waypoints', [])
        if not waypoints:
            raise ValueError('run configuration has no nominal survey route')
        data['flight_paths'] = dict(frame='map',
            nominal_route=[configuration['home']]+waypoints,
            planned_path=[], executed_path=[],
            nominal_route_source=str(Path(config).resolve()),
            note='Nominal route from supplied run configuration; no historical actual path available.')
    output.write_text(VIEWER.replace('__MAP_DATA__', json.dumps(data, separators=(',', ':'), allow_nan=False)))
    return output


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('map_json', type=Path)
    parser.add_argument('--output', type=Path)
    parser.add_argument('--config', type=Path, help='Exact configuration saved in this simulation run')
    args = parser.parse_args()
    print(refresh(args.map_json, args.output or args.map_json.with_name('map_paths.html'), args.config))
