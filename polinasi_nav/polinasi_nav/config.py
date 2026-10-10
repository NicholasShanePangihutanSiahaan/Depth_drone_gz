"""Shared configuration for ROS and standalone validation."""
import json
import math
from pathlib import Path


def load_config(path=None):
    if path is None:
        path = Path(__file__).resolve().parents[1] / 'config' / 'navigation.json'
        if not path.exists():
            from ament_index_python.packages import get_package_share_directory
            path = Path(get_package_share_directory('polinasi_nav'))/'config/navigation.json'
    with open(path, encoding='utf8') as stream:
        c = json.load(stream)
    if not isinstance(c['resolution'], (int, float)) or isinstance(c['resolution'], bool) or not math.isfinite(c['resolution']) or c['resolution'] <= 0:
        raise ValueError('resolution must be finite and positive')
    for key in ('resolution', 'speed', 'acceleration', 'jerk', 'cloud_timeout',
                'pose_timeout', 'free_ttl', 'orbit_radius', 'orbit_speed'):
        if c[key] <= 0:
            raise ValueError(f'{key} must be positive')
    if c['inspection_mode'] not in ('orbit', 'viewpoints'):
        raise ValueError('inspection_mode must be orbit or viewpoints')
    if c['viewpoint_count'] < 3 or c['clearance'] < 0 or c['tracking_margin'] < 0:
        raise ValueError('invalid inspection or clearance configuration')
    if c['tracking_limit'] > c['tracking_margin'] or c['mapping_stride'] < 1:
        raise ValueError('tracking_limit must fit tracking_margin; mapping_stride >= 1')
    if not 0 < c['command_speed'] <= c['speed']:
        raise ValueError('command_speed must be positive and no greater than speed')
    if not isinstance(c['exploration_enabled'], bool):
        raise ValueError('exploration_enabled must be boolean')
    if 'navigation_window_size' in c:
        size = c['navigation_window_size']
        if len(size) != 3 or any(not isinstance(v, (int, float)) or not math.isfinite(v) or v <= 0 for v in size):
            raise ValueError('navigation_window_size must contain three finite positive values')
        if c.get('map_timeout', 0) <= 0:
            raise ValueError('rolling navigation requires a positive map_timeout')
    if c.get('continuous_survey', False):
        if not isinstance(c['continuous_survey'], bool):
            raise ValueError('continuous_survey must be boolean')
        for key in ('survey_pass_radius', 'adaptive_rate_limit', 'adaptive_rate_acceleration',
                    'adaptive_clearance_full', 'adaptive_turn_acceleration'):
            if not math.isfinite(c[key]) or c[key] <= 0:
                raise ValueError(f'{key} must be finite and positive')
        if not isinstance(c['survey_batch_points'], int) or c['survey_batch_points'] < 2:
            raise ValueError('survey_batch_points must be an integer >= 2')
        if not 0 < c['adaptive_min_scale'] <= 1 or not 0 <= c['adaptive_clearance_slow'] < c['adaptive_clearance_full']:
            raise ValueError('invalid adaptive speed scales/clearance')
        a_reserve = c['acceleration']-c['command_speed']*c['adaptive_rate_limit']
        if a_reserve <= 0 or c['jerk']-3*a_reserve*c['adaptive_rate_limit']-c['command_speed']*c['adaptive_rate_acceleration'] <= 0:
            raise ValueError('adaptive speed requires acceleration/jerk reserve')
    for key in ('exploration_speed', 'exploration_radius', 'exploration_min_move',
                'exploration_look_range', 'exploration_observe_seconds',
                'exploration_timeout', 'exploration_retry_seconds',
                'exploration_revisit_distance', 'exploration_min_gain',
                'exploration_candidates', 'exploration_max_views'):
        if not isinstance(c[key], (int, float)) or isinstance(c[key], bool) or not math.isfinite(c[key]) or c[key] <= 0:
            raise ValueError(f'{key} must be positive')
    for key in ('exploration_min_altitude', 'exploration_max_altitude', 'exploration_backtrack'):
        if not isinstance(c[key], (int, float)) or isinstance(c[key], bool) or not math.isfinite(c[key]):
            raise ValueError(f'{key} must be finite')
    for key in ('exploration_min_gain', 'exploration_candidates', 'exploration_max_views'):
        if not isinstance(c[key], int) or isinstance(c[key], bool):
            raise ValueError(f'{key} must be an integer')
    if (c['exploration_speed'] > c['command_speed']
            or c['exploration_min_move'] > c['exploration_radius']
            or c['exploration_look_range'] > c['lidar_range']
            or c['exploration_min_altitude'] >= c['exploration_max_altitude']
            or c['exploration_backtrack'] < 0):
        raise ValueError('invalid exploration limits')
    if 'mapping_no_return_stride' in c:
        value = c['mapping_no_return_stride']
        if isinstance(value, bool) or not isinstance(value, int) or value < 1:
            raise ValueError('mapping_no_return_stride must be a positive integer')
    for key in ('exploration_route_guard', 'adaptive_directional_clearance', 'simulation_no_return_rays', 'mapping_corridor_enabled',
                'local_speed_profile', 'trajectory_geometry_cache', 'survey_prefetch'):
        if key in c and not isinstance(c[key], bool):
            raise ValueError(f'{key} must be boolean')
    if c.get('survey_prefetch', False):
        value = c.get('survey_prefetch_seconds', 3.)
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or value <= 0:
            raise ValueError('survey_prefetch_seconds must be finite and positive')
    if c.get('mapping_corridor_enabled', False):
        for key in ('mapping_corridor_radius', 'mapping_near_radius', 'mapping_corridor_lookahead',
                    'mapping_corridor_expand_step', 'mapping_corridor_max_radius', 'mapping_corridor_expand_interval'):
            value = c.get(key)
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or value <= 0:
                raise ValueError(f'{key} must be finite and positive')
        footprint = math.sqrt(sum((v/2)**2 for v in c['drone_dimensions'][:2]))
        required = max(footprint, c['drone_dimensions'][2]/2)+c['clearance']+c['tracking_margin']+math.sqrt(3)*c['resolution']
        if c['mapping_corridor_radius'] < required or c['mapping_near_radius'] < required:
            raise ValueError('mapping ROI must contain inflated drone and voxel margin')
        if c['mapping_corridor_max_radius'] < c['mapping_corridor_radius']:
            raise ValueError('maximum corridor radius must cover initial radius')
    if c.get('exploration_route_guard', False):
        wait = c.get('exploration_map_wait_seconds', .8)
        if not isinstance(wait, (int, float)) or not math.isfinite(wait) or not 0 < wait < c['blocked_timeout']:
            raise ValueError('map wait must be positive and shorter than blocked_timeout')
        for key in ('exploration_map_wait_scans', 'exploration_min_relevant_gain'):
            value = c.get(key, 2 if key.endswith('scans') else 1)
            if isinstance(value, bool) or not isinstance(value, int) or value < 1:
                raise ValueError(f'{key} must be a positive integer')
    return c
