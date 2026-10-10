"""Run planner scenarios and a separate sensor-only localisation experiment."""
import argparse
import json
import time
from pathlib import Path
import numpy as np
from .config import load_config
from .control import MissionController
from .localisation import LidarImuOdometry
from .mapping import FREE, UNKNOWN, OCCUPIED, VoxelMap
from .simulation import Plant, SCENARIOS, Scene


def fixture_map(scene, config):
    """Ground-truth, fully observed map fixture ONLY for planner isolation tests."""
    grid = VoxelMap(config)
    grid.state[:] = FREE
    grid.seen[:] = 0.
    ids = np.indices(grid.shape).reshape(3, -1).T
    p = grid.centers(ids)
    for lo, hi in scene.boxes:
        # Mark every voxel whose volume touches an obstacle, not only its center.
        overlap = np.all((p+grid.res/2 >= lo-1e-9) & (p-grid.res/2 <= hi+1e-9), axis=1)
        grid.state[tuple(ids[overlap].T)] = OCCUPIED
    grid.rebuild(0.)
    return grid


def run_scenario(name, config, output, plotting=True):
    scene = Scene(config, name)
    grid = fixture_map(scene, config)
    controller = MissionController(config, grid)
    plant = Plant(config['home'], config)
    rows = []
    dt, now = 0.05, 0.
    start = time.monotonic()
    for _ in range(30000):
        controller.health.pose_stamp = now
        controller.health.localisation_ok = True
        if name != 'sensor_dropout' or now < 8.:
            controller.health.cloud_stamp = now
        command, velocity = controller.tick(now, dt, plant.p, plant.v)
        plant.step(command, velocity, dt)
        rows.append([now, *plant.p, *command, scene.surface_clearance(plant.p),
                     np.linalg.norm(plant.p-command), *controller.command_v, *controller.command_a, *plant.v, *plant.a])
        now += dt
        if controller.state == 'LAND':
            # Landing dynamics belong to the flight manager/autopilot. Mark the
            # request, don't fabricate a touchdown from this point-mass plant.
            break
        if controller.failure and controller.state == 'HOLD_ABORT':
            break
    rows = np.array(rows)
    success = controller.state == 'LAND' and controller.buzzer_events == len(config['trees'])
    report = {
        'scenario': name, 'localisation_mode': 'GROUND_TRUTH_PLANNER_TEST',
        'map_source': 'fully_observed_ground_truth_fixture',
        'plant': 'acceleration_limited_point_mass; NOT SITL',
        'completed_to_land_request': success, 'touchdown_validated': False,
        'state': controller.state, 'failure': controller.failure,
        'minimum_surface_clearance_m': float(rows[:, 7].min()),
        'tracking_rmse_m': float(np.sqrt(np.mean(rows[:, 8]**2))),
        'tracking_max_m': float(rows[:, 8].max()),
        'command_max_speed_m_s': float(np.linalg.norm(rows[:, 9:12], axis=1).max()),
        'command_max_acceleration_m_s2': float(np.linalg.norm(rows[:, 12:15], axis=1).max()),
        'command_max_jerk_m_s3': float(np.linalg.norm(np.diff(rows[:, 12:15], axis=0)/dt, axis=1).max()),
        'executed_max_speed_m_s': float(np.linalg.norm(rows[:, 15:18], axis=1).max()),
        'executed_max_acceleration_m_s2': float(np.linalg.norm(rows[:, 18:21], axis=1).max()),
        'buzzer_events': controller.buzzer_events, 'replans': controller.replans,
        'simulated_seconds': now, 'compute_seconds': time.monotonic()-start
    }
    output.mkdir(parents=True, exist_ok=True)
    np.savetxt(output/f'{name}.csv', rows, delimiter=',', header='time,x,y,z,cmd_x,cmd_y,cmd_z,clearance,error,vx,vy,vz,ax,ay,az,actual_vx,actual_vy,actual_vz,actual_ax,actual_ay,actual_az')
    if plotting:
        plot(scene, grid, controller, rows, output/f'{name}.png')
    return report


def plot(scene, grid, controller, rows, path):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    figure = plt.figure(figsize=(12, 8))
    ax = figure.add_subplot(111, projection='3d')
    occupied = grid.centers(np.argwhere(grid.state == OCCUPIED))
    ax.scatter(*occupied[::6].T, s=4, alpha=0.12, label='occupied (ground + foliage retained)')
    unknown = grid.centers(np.argwhere(grid.state == UNKNOWN))
    if len(unknown):
        ax.scatter(*unknown[::4].T, s=5, color='grey', alpha=0.2, label='remaining unknown')
    origin, endpoints, hits, _ = scene.scan(scene.c['home'])
    cloud = endpoints[hits][::8]
    if len(cloud):
        ax.scatter(*cloud.T, s=5, alpha=0.4, label='MID-360 style returns')
    from .inspection import orbit
    for index, tree in enumerate(scene.c['trees']):
        points = np.array(orbit(tree, scene.c))
        ax.plot(*points.T, '--', color='orange', label='nominal orbit' if index == 0 else None)
    for index, points in enumerate(controller.paths):
        ax.plot(*points.T, color='cyan', label='planned path' if index == 0 else None)
    ax.plot(*rows[:, 4:7].T, color='blue', linewidth=1, label='commanded trajectory')
    ax.plot(*rows[:, 1:4].T, color='green', label='executed point-mass path')
    if controller.exploration_points:
        viewpoints = np.asarray(controller.exploration_points)
        ax.scatter(*viewpoints.T, color='purple', s=40, marker='x', label='exploration viewpoints')
    ax.set(xlabel='ENU x (m)', ylabel='ENU y (m)', zlabel='up (m)',
           title=f'{scene.scenario}: ground-truth planner test, NOT SITL/LIO validation')
    ax.legend(fontsize=8)
    figure.tight_layout()
    figure.savefig(path, dpi=130)
    plt.close(figure)


def run_localisation(config):
    """Only scan points + IMU enter estimator; truth is used AFTER for scoring."""
    scene = Scene(config)
    estimator = LidarImuOdometry(config['lidar_mount'], config['home'])
    now = 0.
    for _ in range(50):
        now += 0.005
        estimator.imu([0., 0., 0.], [0., 0., 9.81], now)
    errors, valid, samples = [], 0, 0
    # Constant low-speed traverse; acceleration enters through accelerometer.
    last_v = np.zeros(3)
    for frame in range(80):
        t = frame*0.1
        p = np.asarray(config['home'])+[0.5*(1-np.cos(t/4)), 0.25*np.sin(t/4), 0.15*np.sin(t/3)]
        v = np.array([0.125*np.sin(t/4), 0.0625*np.cos(t/4), 0.05*np.cos(t/3)])
        accel = (v-last_v)/0.1 if frame else np.zeros(3)
        for _ in range(20):
            now += 0.005
            estimator.imu([0., 0., 0.], accel+[0., 0., 9.81], now)
        _, _, hits, sensor = scene.scan(p, noise=0.01, phase=frame)
        accepted = estimator.scan(sensor[hits], now)
        samples += 1
        valid += accepted
        if accepted:
            errors.append(float(np.linalg.norm(estimator.p-p)))
        last_v = v
    return {'mode': 'SENSOR_ONLY_LOCALISATION_EXPERIMENT',
            'estimator': 'experimental_IMU_predicted_ICP; NOT FAST-LIO',
            'input': 'synthetic MID-360 style first returns + synthetic IMU',
            'ground_truth_used_by_estimator': False,
            'frames': samples, 'accepted_frames': valid,
            'position_rmse_m': float(np.sqrt(np.mean(np.array(errors)**2))) if errors else None,
            'position_max_m': max(errors) if errors else None,
            'last_status': estimator.reason,
            'full_sensor_based_flight_validated': False}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--config')
    parser.add_argument('--scenario', choices=(*SCENARIOS, 'all'), default='all')
    parser.add_argument('--output', type=Path, default=Path('reports'))
    parser.add_argument('--no-plots', action='store_true')
    parser.add_argument('--inspection-mode', choices=('orbit', 'viewpoints'))
    args = parser.parse_args()
    c = load_config(args.config)
    if args.inspection_mode:
        c['inspection_mode'] = args.inspection_mode
    cases = SCENARIOS if args.scenario == 'all' else (args.scenario,)
    reports = [run_scenario(name, c, args.output, not args.no_plots) for name in cases]
    lio = run_localisation(c)
    result = {'scenarios': reports, 'localisation_experiment': lio}
    args.output.mkdir(parents=True, exist_ok=True)
    (args.output/'validation.json').write_text(json.dumps(result, indent=2)+'\n')
    print(json.dumps(result, indent=2))
    expected = {'clear_orbit': True, 'leaf_blocked': True, 'vertical_detour': True,
                'fully_blocked': False, 'sensor_dropout': False}
    ok = all(r['completed_to_land_request'] == expected[r['scenario']]
             and r['minimum_surface_clearance_m'] >= c['clearance']
             and r['command_max_speed_m_s'] <= c['speed']+1e-3
             and r['command_max_acceleration_m_s2'] <= c['acceleration']+1e-3
             and r['command_max_jerk_m_s3'] <= c['jerk']+1e-2
             for r in reports)
    raise SystemExit(0 if ok else 1)


if __name__ == '__main__':
    main()
