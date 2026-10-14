import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'tools'))
from check_sensor_mission_completion import completion


def completed():
    return dict(stop_reason='',ground_truth_used_by_estimator=False,last_navigation=dict(
        mode='sensor',mission='COMPLETE',armed=False,failure='',inspected_tree_ids=[1,2],
        tree_scan_records=[dict(tree_id=tree,view=view,inspection_valid=True)
                           for tree in (1,2) for view in range(6)]))


def test_complete_disarmed_two_inspections_required():
    assert completion(completed())['passed']
    for key,value in [('armed',True),('mission','LAND'),('inspected_tree_ids',[1]),('failure','blocked_route')]:
        summary=completed();summary['last_navigation'][key]=value
        assert not completion(summary)['passed']


def test_duplicate_or_invalid_views_and_failed_audit_cannot_pass():
    summary=completed();summary['last_navigation']['tree_scan_records'][0]['inspection_valid']=False
    assert not completion(summary)['passed']
    summary=completed();summary['last_navigation']['tree_scan_records'][0]['view']=1
    assert not completion(summary)['passed']
    summary=completed();summary['stop_reason']='registered_simulation_stopped'
    assert not completion(summary)['passed']
