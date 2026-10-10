"""Collision, frame, unknown-space, and sensor-data regression tests."""
import math

import numpy as np
import pytest

from beehive_drone.depth_navigation import (
    DepthView, LocalGrid, ObstacleMemory, decode_depth, rotation,
)


def view(depth=None, yaw=0.0, origin=(0.0, 0.0, 1.5), inf=False):
    if depth is None:
        depth = np.full((80, 100), 5.0)
    return DepthView(depth, (60.0, 60.0, 49.5, 39.5), origin,
                     rotation((0, 0, math.sin(yaw/2), math.cos(yaw/2))),
                     inf_is_clear=inf)


def test_decode_padded_big_endian_depth_and_mm():
    raw = np.array([[1000, 2000, 999], [3000, 4000, 999]], dtype='>u2')
    decoded = decode_depth(raw.tobytes(), 2, 2, 6, '16UC1', True)
    assert np.allclose(decoded, [[1, 2], [3, 4]])
    with pytest.raises(ValueError):
        decode_depth(b'\x00', 4, 3, 16, '32FC1')


def test_yaw_and_pitch_transform_ground_not_false_obstacle():
    cloud = view(yaw=math.pi/2).points()
    assert np.allclose(cloud[:, 1], 5.0)
    pitched = rotation((0, math.sin(math.pi/8), 0, math.cos(math.pi/8)))
    transformed = pitched @ np.array([2, 0, 0])
    assert np.allclose(transformed, [math.sqrt(2), 0, -math.sqrt(2)])


def test_invalid_depth_is_not_free_and_blind_side_is_not_free():
    start, end = [0, 0, 1.5], [0.6, 0, 1.5]
    assert view().corridor_seen(start, end, 0.4, 0.3)
    assert not view(np.full((80, 100), np.nan)).corridor_seen(start, end, 0.4, 0.3)
    assert not view(np.full((80, 100), np.inf)).corridor_seen(start, end, 0.4, 0.3)
    assert view(np.full((80, 100), np.inf), inf=True).corridor_seen(start, end, 0.4, 0.3)
    assert not view().corridor_seen(start, [0, 0.6, 1.5], 0.4, 0.3)


def test_memory_keeps_obstacles_out_of_view_but_clears_observed_free():
    memory = ObstacleMemory()
    memory.update(view(np.full((80, 100), 2.0)), np.array([0, 0, 1.5]))
    occupied = len(memory.cells)
    assert occupied > 0
    memory.update(view(yaw=math.pi), np.array([0, 0, 1.5]))
    assert len(memory.cells) >= occupied
    for _ in range(3):
        memory.update(view(), np.array([0, 0, 1.5]))
    obstacles = memory.slice(1.5, 0.3)
    assert not np.any((obstacles[:, 0] > 1.8) & (obstacles[:, 0] < 2.2)
                      & (abs(obstacles[:, 1]) < 0.5))


def test_planner_routes_around_frond_and_preserves_clearance():
    wall = np.column_stack((np.full(11, 2.0), np.linspace(-0.5, 0.5, 11)))
    grid = LocalGrid((0, 0), wall, clearance=0.8)
    path = grid.plan((0, 0), (4, 0))
    assert len(path) > 2
    assert max(abs(p[1]) for p in path) > 1.3
    for a, b in zip(path, path[1:]):
        assert grid.segment_free(a, b)
    command = grid.next_waypoint(np.array([0., 0.]), path, 0.65)
    assert grid.segment_free((0, 0), command)


def test_no_path_through_sealed_wall_or_occupied_goal():
    wall = np.column_stack((np.full(81, 2.0), np.linspace(-8, 8, 81)))
    grid = LocalGrid((0, 0), wall, clearance=0.8)
    assert grid.plan((0, 0), (4, 0)) == []
    assert grid.plan((0, 0), (2, 0)) == []


def test_target_trunk_remains_an_obstacle():
    points = np.array([[2.6, 0.0]])
    grid = LocalGrid((0, 0), points, clearance=0.8)
    assert not grid.free(grid.cell((2.6, 0)))
    assert not grid.segment_free((0, 0), (5, 0))
