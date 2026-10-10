"""Read-only decision audit. A positive result is not full-flight certification."""
import argparse
from collections import Counter
import json
from pathlib import Path


def analyse(path):
    rows = [json.loads(line) for line in path.read_text().splitlines() if line.strip()]
    moving = [r for r in rows if r.get('commanded_speed_mps', 0.) > .01]
    speeds = [r.get('speed_diagnostics', {}) for r in moving]
    clips = [r.get('mapping_roi', {}).get('clipping', {}) for r in rows]
    fractions = [c['generated_samples']/c['full_trace_samples'] for c in clips if c.get('full_trace_samples', 0)]
    elapsed, commanded_distance, measured_distance, stopped = 0., 0., 0., 0.
    for a,b in zip(rows, rows[1:]):
        dt = b['simulation_time']-a['simulation_time']
        if a.get('flight') == 'MISSION' and a.get('mission') == 'SURVEY' and 0 < dt <= 2.:
            elapsed += dt
            commanded_distance += a.get('commanded_speed_mps', 0.)*dt
            measured_distance += a.get('measured_speed_mps', 0.)*dt
            stopped += dt if a.get('commanded_speed_mps', 0.) < .02 else 0.
    jobs = {r['planner_job_id']:r.get('planner_stages_last_wall_ms', {}) for r in rows if r.get('planner_job_id')}
    explores, previous = [], None
    for row in rows:
        if row.get('mission') == 'EXPLORE' and previous != 'EXPLORE':
            explores.append({k: row.get(k) for k in
                ('simulation_time', 'survey_waypoint', 'route_blockage')})
        previous = row.get('mission')
    return dict(scope='sampled navigation decision audit, not LiDAR localisation validation',
        source=str(path), samples=len(rows), final_status=rows[-1] if rows else {},
        exploration_events=explores,
        survey_sim_seconds=elapsed, survey_stopped_sim_seconds=stopped,
        survey_mean_commanded_speed_mps=commanded_distance/elapsed if elapsed else None,
        survey_mean_measured_speed_mps=measured_distance/elapsed if elapsed else None,
        planner_jobs=list(jobs.values()),
        processing_pipeline=rows[-1].get('processing_pipeline') if rows else None,
        mapping_generated_sample_fraction_mean=sum(fractions)/len(fractions) if fractions else None,
        mapping_corridor_expansions_max=max((r.get('mapping_roi', {}).get('region', {}).get('expansions', 0)
            for r in rows if r.get('mapping_roi', {}).get('region')), default=0),
        unexplained_exploration_events=sum(
            e.get('route_blockage', {}).get('reason') != 'route_unknown' or
            e.get('route_blockage', {}).get('unknown_count', 0) <= 0 for e in explores),
        adaptive_reasons=dict(Counter(r.get('adaptive_reason') for r in moving)),
        route_reasons=dict(Counter(r.get('route_blockage', {}).get('reason', '') for r in rows)),
        legacy_penalty_removed_samples=sum(s.get('legacy_space_scale', 1.) <= .3 and
                                          s.get('braking_scale', 0.) == 1. for s in speeds),
        directional_braking_limited_samples=sum(s.get('braking_scale', 1.) < 1. for s in speeds),
        maximum_command_speed_mps=max((r.get('commanded_speed_mps', 0.) for r in rows), default=0.),
        maximum_tracking_error_airborne_m=max((r.get('tracking_error_m', 0.) for r in rows
            if r.get('flight') == 'MISSION' and r.get('mission') in ('SURVEY', 'EXPLORE', 'RETURN')), default=0.),
        tracking_error_scope='SURVEY/EXPLORE/RETURN only; excludes takeoff and ArduPilot landing',
        failures=sorted({r['failure'] for r in rows if r.get('failure')}))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('trace', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    report = analyse(args.trace)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps({k:v for k,v in report.items() if k != 'final_status'}, indent=2))


if __name__ == '__main__':
    main()
