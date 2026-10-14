"""Repeated stop must not terminate a launcher already inside its cleanup."""
import sys
from pathlib import Path
from unittest.mock import patch
ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'tools'))
from stop_simulation import request_stop


def test_stop_is_idempotent(tmp_path):
    data=dict(launcher_pid=123,run='/registered/simulation')
    with patch('stop_simulation.os.kill') as kill:
        request_stop(tmp_path/'session.json',data)
        request_stop(tmp_path/'session.json',data)
    assert kill.call_count==1
    assert data['stop_requested'] is True
