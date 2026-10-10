"""Partial-map gap recovery using analytic LiDAR rays and a point-mass plant.

NOT Gazebo/SITL or localisation validation. Outside one withheld gap, the map
is a surveyed fixture. Withheld cells change only through actual ray updates.
"""
import argparse
import json
from pathlib import Path
import numpy as np
from .benchmark import fixture_map, plot
from .config import load_config
from .control import MissionController
from .mapping import UNKNOWN, FREE
from .simulation import Scene, Plant


def run(case, output):
    c = load_config()
    c.update(home=[0., 0., 2.], inspection_mode='orbit', trees=[[4., 0., 2.]], free_ttl=120.,
             lidar_azimuth_samples=180, lidar_elevation_samples=24)
    scene = Scene(c, 'fully_blocked' if case == 'sealed_gap' else 'clear_orbit')
    grid = fixture_map(scene, c)
    ids = np.indices(grid.shape).reshape(3, -1).T
    points = grid.centers(ids)
    gap_min_x, gap_max_x = (4.5, 6.) if case == 'sealed_gap' else (1.5, 3.)
    gap = ((points[:, 0] > gap_min_x) & (points[:, 0] < gap_max_x)
           & (abs(points[:, 1]) < 1.2) & (points[:, 2] > (.8 if case == 'blind_gap' else 1.5))
           & (points[:, 2] < 3.2))
    # Never overwrite known occupied foliage/ground with unknown or free.
    gap &= grid.state[tuple(ids.T)] == FREE
    grid.state[tuple(ids[gap].T)] = UNKNOWN
    grid.rebuild(0.)
    initial_unknown = int((grid.state == UNKNOWN).sum())
    ctl = MissionController(c, grid)
    ctl.state = 'INSPECT'
    plant = Plant(c['home'], c)
    rows, events = [], []
    now, dt, resumed, reached = 0., .05, False, False
    previous = None
    selection_before_reveal = False
    for tick in range(1000):
        ctl.health.pose_stamp = now
        ctl.health.localisation_ok = True
        if tick % 4 == 0 and (case != 'dropout' or now < 3.):
            origin, endpoints, hits, _ = scene.scan(plant.p, phase=tick//4)
            # Withhold observations of the test gap until exploration starts.
            # This is an explicit sensor-test injection, never production code.
            if ctl.exploration_views == 0:
                # Do not integrate any pre-exploration rays: known fixture only.
                endpoints, hits = endpoints[:0], hits[:0]
            grid.integrate(origin, endpoints, hits, now)
            ctl.health.cloud_stamp = now
        grid.rebuild(now)
        before = ctl.state
        command, velocity = ctl.tick(now, dt, plant.p, plant.v)
        if before == 'INSPECT' and ctl.state == 'EXPLORE' and not selection_before_reveal:
            selection_before_reveal = initial_unknown == int((grid.state == UNKNOWN).sum())
        plant.step(command, velocity, dt)
        key = (ctl.state, ctl.exploration_phase)
        if key != previous:
            events.append({'time': round(now, 3), 'state': ctl.state,
                           'phase': ctl.exploration_phase, 'views': ctl.exploration_views,
                           'unknown_cells': int((grid.state == UNKNOWN).sum())})
            previous = key
            print(f'{case}: {now:.1f}s {ctl.state}/{ctl.exploration_phase} views={ctl.exploration_views}', flush=True)
        rows.append([now, *plant.p, *command, scene.surface_clearance(plant.p),
                     np.linalg.norm(plant.p-command), *ctl.command_v, *ctl.command_a, *plant.v, *plant.a])
        if ctl.exploration_views and ctl.state == 'INSPECT' and ctl.trajectory is not None:
            resumed = True
            if np.linalg.norm(plant.p-[2., 0., 2.]) < .12 and np.linalg.norm(plant.v) < .08:
                reached = True
                break
        if ctl.state == 'HOLD_ABORT' or ctl.failure.startswith('emergency_unavoidable'):
            break
        now += dt
    rows = np.asarray(rows)
    expected = reached if case == 'peek_gap' else (ctl.failure in ('blocked_route', 'stale_lidar', 'exploration_exhausted', 'exploration_timeout') and ctl.state == 'HOLD_ABORT')
    report = {'case': case, 'passed': bool(expected), 'mission_resumed': resumed,
              'reached_first_inspection_goal': reached,
              'state': ctl.state, 'failure': ctl.failure, 'exploration_views': ctl.exploration_views,
              'initial_unknown_cells': initial_unknown, 'remaining_unknown_cells': int((grid.state == UNKNOWN).sum()),
              'selection_before_gap_observations': selection_before_reveal,
              'minimum_surface_clearance_m': float(rows[:, 7].min()),
              'tracking_rmse_m': float(np.sqrt(np.mean(rows[:, 8]**2))),
              'command_max_speed_m_s': float(np.linalg.norm(rows[:, 9:12], axis=1).max()),
              'command_max_acceleration_m_s2': float(np.linalg.norm(rows[:, 12:15], axis=1).max()),
              'command_max_jerk_m_s3': float(np.linalg.norm(np.diff(rows[:, 12:15], axis=0)/dt, axis=1).max()),
              'events': events, 'simulation_seconds': now,
              'map_source': 'surveyed_fixture_with_withheld_gap; gap updated by analytic first-return rays',
              'sensor_simplification': 'finite max-range rays treated as valid no-return; Gazebo infinite returns remain discarded',
              'localisation': 'GROUND_TRUTH', 'plant': 'point_mass_NOT_ArduPilot',
              'airborne_start': True, 'full_mission_validated': False}
    output.mkdir(parents=True, exist_ok=True)
    np.savetxt(output/f'{case}.csv', rows, delimiter=',', header='time,x,y,z,cmd_x,cmd_y,cmd_z,clearance,error,vx,vy,vz,ax,ay,az,actual_vx,actual_vy,actual_vz,actual_ax,actual_ay,actual_az')
    plot(scene, grid, ctl, rows, output/f'{case}.png')
    (output/f'{case}.json').write_text(json.dumps(report, indent=2)+'\n')
    return report


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', type=Path, default=Path('reports/exploration'))
    parser.add_argument('--case', choices=('all', 'peek_gap', 'sealed_gap', 'blind_gap', 'dropout'), default='all')
    args = parser.parse_args()
    cases = ('peek_gap', 'sealed_gap', 'blind_gap', 'dropout') if args.case == 'all' else (args.case,)
    reports = [run(case, args.output) for case in cases]
    (args.output/'validation.json').write_text(json.dumps(reports, indent=2)+'\n')
    print(json.dumps(reports, indent=2))
    raise SystemExit(0 if all(r['passed'] for r in reports) else 1)


if __name__ == '__main__':
    main()
