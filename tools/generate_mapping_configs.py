"""Generate isolated survey presets without changing inspection configuration."""
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
from scipy.spatial import ConvexHull

ROOT = Path(__file__).resolve().parents[1]


def farm_region(margin=4.5):
    """Configured farm footprint, not obstacle/free-space knowledge."""
    world = ET.parse(ROOT/'polinasi_nav/worlds/palm_farm.sdf').getroot().find('world')
    trunks = np.asarray([list(map(float, model.findtext('pose').split()[:2]))
                         for model in world.findall('model') if model.get('name', '').startswith('oil_palm_')])
    equations = ConvexHull(trunks).equations.copy()
    equations[:, 2] -= margin  # Normals are unit length: expand canopy-edge margin.
    polygon = []
    for i, a in enumerate(equations):
        for b in equations[i+1:]:
            if abs(np.linalg.det(np.vstack((a[:2], b[:2])))) < 1e-9:
                continue
            point = np.linalg.solve(np.vstack((a[:2], b[:2])), -np.array([a[2], b[2]]))
            if np.all(equations[:, :2]@point+equations[:, 2] <= 1e-7):
                polygon.append(point)
    polygon = np.asarray(polygon)
    polygon = polygon[ConvexHull(polygon).vertices]
    bounds = [float(polygon[:, 0].min()), float(polygon[:, 0].max()),
              float(polygon[:, 1].min()), float(polygon[:, 1].max())]
    return trunks.tolist(), equations, polygon.tolist(), bounds


def coverage_route(bounds=None, spacing=8., step=2., altitude=2.):
    """Nominal lawnmower route only: never creates free occupancy evidence."""
    trunks, equations, polygon, farm_bounds = farm_region()
    xmin, xmax, ymin, ymax = farm_bounds if bounds is None else bounds
    def inside(point):
        return np.all(equations[:, :2]@np.asarray(point[:2])+equations[:, 2] <= 1e-7)
    def row_limits(y):
        lo, hi = xmin, xmax
        for nx, ny, offset in equations:
            if nx > 1e-9:
                hi = min(hi, (-offset-ny*y)/nx)
            elif nx < -1e-9:
                lo = max(lo, (-offset-ny*y)/nx)
        return lo, hi
    points, current = [], [0., 0., altitude]
    def append_segment(end):
        nonlocal current
        length = math.dist(current, end)
        count = max(1, math.ceil(length/step))
        start = current[:]
        for i in range(1, count+1):
            p = [a+(b-a)*i/count for a, b in zip(start, end)]
            # Layout guides nominal goals only; all actual paths require LiDAR.
            if any(math.hypot(p[0]-x, p[1]-y) < 2.2 for x, y in trunks):
                candidates = [q for q in ([p[0], p[1]+offset, altitude] for offset in (-2.5, 2.5)) if inside(q)]
                if not candidates:
                    candidates = [q for q in ([p[0]+offset, p[1], altitude] for offset in (-2.5, 2.5)) if inside(q)]
                p = max(candidates, key=lambda q: min(math.hypot(q[0]-x, q[1]-y) for x, y in trunks))
            if not points or math.dist(points[-1], p) > .01:
                points.append(p)
        current = end[:]
    # Progressively move away from the observed launch region, not a distant
    # first goal outside the rolling map.
    first_lo, first_hi = row_limits(ymin)
    append_segment([max(first_lo, min(0., first_hi)), ymin, altitude])
    append_segment([first_lo, ymin, altitude])
    rows = max(1, math.ceil((ymax-ymin)/spacing))
    for row in range(rows+1):
        y = ymin+(ymax-ymin)*row/rows
        left, right = row_limits(y)
        start_x, end_x = (left, right) if row % 2 == 0 else (right, left)
        append_segment([start_x, y, altitude])
        append_segment([end_x, y, altitude])
    # Return in short steps so home is always inside the rolling window.
    append_segment([0., 0., altitude])
    # Offsetting a goal around a trunk can create a long jump. Subdivide those
    # legs too, so every next goal remains within the rolling navigation window.
    dense, start = [], [0., 0., altitude]
    for end in points:
        legs = [end]
        dx, dy = end[0]-start[0], end[1]-start[1]
        for x, y in trunks:
            f = max(0., min(1., ((x-start[0])*dx+(y-start[1])*dy)/max(dx*dx+dy*dy, 1e-9)))
            if math.hypot(start[0]+f*dx-x, start[1]+f*dy-y) < 2.2:
                a = math.atan2(start[1]-y, start[0]-x)
                b = math.atan2(end[1]-y, end[0]-x)
                delta = math.atan2(math.sin(b-a), math.cos(b-a))
                count = max(1, math.ceil(abs(delta)/(math.pi/8)))
                legs = [[x+2.4*math.cos(a+delta*i/count), y+2.4*math.sin(a+delta*i/count), altitude]
                        for i in range(count+1)] + [end]
                break
        if not all(inside(q) for q in legs):
            raise ValueError('nominal trunk bypass exits farm footprint')
        previous = start
        for goal in legs:
            count = max(1, math.ceil(math.dist(previous, goal)/step))
            dense.extend([[a+(b-a)*i/count for a, b in zip(previous, goal)] for i in range(1, count+1)])
            previous = goal
        start = end
    return dense


def main():
    base = json.loads((ROOT/'polinasi_nav/config/palm_farm_navigation.json').read_text())
    base.update(mock_flower_events=False, camera_pitch_enabled=False,
                terrain_following=False, command_speed=.2, speed=.4,
                survey_dwell=.2, planner_max_expansions=4000, home=[0., 0., 2.],
                bounds_min=[-40., -40., -.6], bounds_max=[52., 42., 10.2],
                navigation_window_size=[10., 10., 8.], map_timeout=1.,
                continuous_survey=True, survey_batch_points=4, survey_pass_radius=.2,
                exploration_route_guard=True, exploration_map_wait_seconds=.8,
                exploration_map_wait_scans=2, exploration_min_relevant_gain=1,
                adaptive_directional_clearance=True,
                local_speed_profile=True, trajectory_geometry_cache=True,
                survey_prefetch=True, survey_prefetch_seconds=3.,
                simulation_no_return_rays=True,
                mapping_no_return_stride=12,
                mapping_corridor_enabled=True, mapping_corridor_radius=1.5,
                mapping_near_radius=2., mapping_corridor_lookahead=20.,
                mapping_corridor_expand_step=1., mapping_corridor_max_radius=4.,
                mapping_corridor_expand_interval=.8,
                adaptive_min_scale=.25, adaptive_rate_limit=.25, adaptive_rate_acceleration=.4,
                adaptive_clearance_slow=.05, adaptive_clearance_full=.45, adaptive_turn_acceleration=.25,
                map_log_max_points=1000000,
                lidar_range=20.,
                # Static simulation scene only: bounded mission-scale memory,
                # not a silent relaxation of live sensor/dropout checks.
                free_ttl=10800., map_environment='static_simulated_farm')
    routes = {
        'tour': coverage_route(),
        'smoke': [[.25, 0., 2.], [.25, .25, 2.], [0., .25, 2.], [0., 0., 2.]],
        'check': [[0., -2., 2.], [0., -4., 2.], [0., -2., 2.], [0., 0., 2.]],
    }
    for name, points in routes.items():
        config = dict(base, survey_waypoints=points)
        if name == 'smoke':
            config.update(survey_dwell=.2)
        elif name == 'tour':
            trunks, equations, polygon, region = farm_region()
            config.update(command_speed=.3, survey_dwell=.15,
                coverage_roi=region, coverage_polygon=polygon,
                coverage_region_source='configured_simulated_tree_layout', coverage_tree_centres=trunks,
                coverage_edge_margin=4.5, coverage_lane_spacing=8.,
                # Stop-to-stop quintic timing is longer than distance/speed.
                # Budget the full nominal tour; still a bounded static-scene
                # history policy, not permission to navigate on stale sensors.
                survey_max_duration=18000., free_ttl=21600.)
        else:
            config.update(command_speed=.3, survey_max_duration=600.)
        (ROOT/f'polinasi_nav/config/mapping_{name}.json').write_text(json.dumps(config, indent=2)+'\n')
    mavros = (ROOT/'polinasi_nav/config/mavros.yaml').read_text()
    mavros = mavros.replace('frame_id: map', 'frame_id: fcu_local').replace('setpoint_position', 'setpoint_raw')
    mavros = mavros.replace('    mav_frame: LOCAL_NED\n', '').replace('    tf.listen: false\n', '')
    mavros = mavros.replace('/**/setpoint_raw:\n  ros__parameters:\n',
                            '/**/setpoint_raw:\n  ros__parameters:\n    use_sim_time: true\n')
    (ROOT/'polinasi_nav/config/mavros_mapping.yaml').write_text(mavros)


if __name__ == '__main__':
    main()
