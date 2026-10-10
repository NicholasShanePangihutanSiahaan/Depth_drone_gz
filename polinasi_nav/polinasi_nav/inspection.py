"""Hardcoded trees, nominal orbit and scored camera viewpoints."""
import math
import itertools
import numpy as np


def orbit(tree, config):
    angles = np.linspace(math.pi, math.pi+2*math.pi, config['viewpoint_count']+1)
    return [np.array([tree[0]+config['orbit_radius']*math.cos(a),
                      tree[1]+config['orbit_radius']*math.sin(a),
                      config['orbit_altitude']]) for a in angles]


def view_candidates(tree, angle, current, grid, config):
    """Return valid camera views ranked by clearance, visibility, scale and travel."""
    flower = np.asarray(tree)+[0., 0., 0.4]
    views = []
    for height, radius, offset in itertools.product(config['candidate_heights'], config['candidate_distances'], config.get('candidate_angle_offsets', [0.])):
            candidate_angle = angle+offset
            p = np.array([tree[0]+radius*math.cos(candidate_angle), tree[1]+radius*math.sin(candidate_angle), height])
            # Choose the voxel centre: an exact cell boundary can leave zero
            # room for a finite-precision stop even when the cell is free.
            p = grid.centers(grid.indices(p))
            if not grid.safe(p):
                continue
            yaw = math.atan2(tree[1]-p[1], tree[0]-p[0])
            mount = np.asarray(config['camera_mount'])
            camera = p+np.array([math.cos(yaw)*mount[0]-math.sin(yaw)*mount[1],
                                math.sin(yaw)*mount[0]+math.cos(yaw)*mount[1], mount[2]])
            delta = flower-camera
            pitch = math.atan2(delta[2], np.linalg.norm(delta[:2]))
            camera_pitch = float(np.clip(pitch, config['camera_pitch_min'], config['camera_pitch_max'])) if config['camera_pitch_enabled'] else 0.
            distance = np.linalg.norm(delta)
            visible = grid.visible(camera, flower)
            useful = abs(pitch-camera_pitch) < config['camera_vertical_fov']/2 and distance <= config['camera_max_distance']
            score = 3.*visible + 2.*useful + min(grid.clearance(p), 1.) - 0.25*np.linalg.norm(p-current) - 0.2*distance - abs(offset)
            views.append((score, p, yaw, camera_pitch, visible and useful))
    return sorted(views, key=lambda x: x[0], reverse=True)


def view_quality(point, tree, yaw, grid, config):
    mount = config['camera_mount']
    camera = np.asarray(point)+[math.cos(yaw)*mount[0]-math.sin(yaw)*mount[1],
                                math.sin(yaw)*mount[0]+math.cos(yaw)*mount[1], mount[2]]
    delta = np.asarray(tree)+[0., 0., 0.4]-camera
    pitch = math.atan2(delta[2], np.linalg.norm(delta[:2]))
    selected = float(np.clip(pitch, config['camera_pitch_min'], config['camera_pitch_max'])) if config['camera_pitch_enabled'] else 0.
    useful = abs(pitch-selected) <= config['camera_vertical_fov']/2 and np.linalg.norm(delta) <= config['camera_max_distance']
    return selected, useful and grid.visible(camera, np.asarray(tree)+[0., 0., 0.4])
