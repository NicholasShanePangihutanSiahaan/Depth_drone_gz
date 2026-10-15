from pathlib import Path
import sys
from unittest.mock import Mock,patch
sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'tools'))
from approach_viewer import ApproachViewer


def test_viewer_opens_once_after_approach_and_closes_only_owned_process(tmp_path,monkeypatch):
    monkeypatch.setenv('DISPLAY',':0')
    child=Mock(pid=12345);child.poll.return_value=None
    with patch('approach_viewer.subprocess.Popen',return_value=child) as spawn,patch('approach_viewer.os.killpg') as kill:
        viewer=ApproachViewer(tmp_path,True)
        viewer.update(dict(tree_phase='SEARCH'))
        viewer.update(dict(tree_phase='APPROACH'))
        spawn.assert_not_called()
        viewer.update(dict(tree_phase='ASCEND',simulation_time=12.))
        viewer.update(dict(tree_phase='SCAN',simulation_time=20.))
        spawn.assert_called_once()
        assert spawn.call_args.kwargs['start_new_session']
        assert viewer.events[0]['stamp']==12.
        viewer.close();assert kill.call_args.args[0]==12345


def test_no_viewer_for_failed_approach_or_disabled_mode(tmp_path):
    with patch('approach_viewer.subprocess.Popen') as spawn:
        viewer=ApproachViewer(tmp_path,True)
        viewer.update(dict(tree_phase='APPROACH'))
        viewer.update(dict(tree_phase='ASCEND',failure='sensor_failure'))
        disabled=ApproachViewer(tmp_path,False)
        disabled.update(dict(tree_phase='APPROACH'));disabled.update(dict(tree_phase='SCAN'))
        spawn.assert_not_called()
