"""Read-only gate before testing a different world. No flight commands."""
import argparse
import json
from pathlib import Path


def completion(summary):
    nav=summary.get('last_navigation',{})
    trees=set(nav.get('inspected_tree_ids',[]))
    records=nav.get('tree_scan_records',[])
    views={tree:{r.get('view') for r in records
                 if r.get('tree_id')==tree and r.get('inspection_valid') is True} for tree in trees}
    checks=dict(no_failure=not summary.get('stop_reason') and not nav.get('failure'),
                complete=nav.get('mission')=='COMPLETE',disarmed=nav.get('armed') is False,
                two_trees=len(trees)>=2,
                six_valid_views_per_tree=len(trees)>=2 and all(len(v)>=6 for v in views.values()),
                sensor_mode=nav.get('mode')=='sensor',
                no_truth_input=summary.get('ground_truth_used_by_estimator') is False)
    return dict(passed=all(checks.values()),checks=checks,
                valid_view_counts={str(k):len(v) for k,v in views.items()})


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('summary',type=Path)
    args=parser.parse_args()
    result=completion(json.loads(args.summary.read_text()))
    print(json.dumps(result,indent=2))
    raise SystemExit(0 if result['passed'] else 1)


if __name__=='__main__':main()
