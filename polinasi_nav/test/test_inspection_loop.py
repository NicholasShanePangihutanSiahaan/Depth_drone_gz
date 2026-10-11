import importlib.util
import json
from pathlib import Path
import sys

TOOLS = Path(__file__).resolve().parents[2] / 'tools'
sys.path.insert(0, str(TOOLS))
spec = importlib.util.spec_from_file_location('inspection_loop', TOOLS / 'run_inspection_loop.py')
loop = importlib.util.module_from_spec(spec)
spec.loader.exec_module(loop)


def test_no_status_is_not_success(tmp_path):
    assert not loop.audit(tmp_path / 'missing.jsonl')['completed']


def test_fault_remains_failure_even_after_complete(tmp_path):
    path = tmp_path / 'events.jsonl'
    path.write_text(json.dumps({'watchdog_fault': 'stale_lidar'}) + '\n' +
                    json.dumps({'mission': 'COMPLETE'}) + '\n')
    result = loop.audit(path)
    assert not result['completed']
    assert result['failures'] == ['stale_lidar']


def test_success_is_explicit_and_sampled(tmp_path):
    path = tmp_path / 'events.jsonl'
    path.write_text(json.dumps({'mission': 'COMPLETE', 'flight': 'COMPLETE', 'armed': False,
                    'inspected_tree_ids': [1,2], 'tree_scan_records': [dict(tree_id=t,view=v) for t in (1,2) for v in range(6)],
                    'measured_speed_mps': .2}) + '\n')
    result = loop.audit(path)
    assert result['completed']
    assert result['event_sample_max_speed_mps'] == .2
    assert 'ground_truth' in result['localisation']


def test_wrapper_opens_only_rviz_and_camera():
    shell = (TOOLS / 'run_inspection_loop.sh').read_text()
    assert 'export POLINASI_GAZEBO_GUI=0' in shell
    for key in ('SIM_GUI','CAMERA_GUI'):
        assert 'export POLINASI_' + key + '=1' in shell
