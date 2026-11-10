import numpy as np
from beehive_drone.depth_navigation import LocalGrid
from beehive_drone.priest_local_planner import PriestLocalPlanner, path_valid


def test_detour_around_wall_preserves_clearance_and_endpoints():
    obstacles = np.column_stack((np.full(17, 2.), np.linspace(-.8, .8, 17)))
    grid = LocalGrid(np.array([0., 0.]), obstacles, .15, 7., .65)
    seed = grid.plan(np.array([0., 0.]), np.array([4., 0.]))
    result = PriestLocalPlanner(budget_ms=200).plan(grid, seed)
    assert path_valid(grid, result.path)
    assert np.allclose(result.path[0], [0., 0.])
    assert np.allclose(result.path[-1], [4., 0.])
    assert max(abs(p[1]) for p in result.path) > 1.4


def test_optimizer_projects_free_reference_and_rejects_new_obstacle():
    grid = LocalGrid(np.array([0., 0.]), np.empty((0, 2)), .2, 5., .6)
    planner = PriestLocalPlanner(budget_ms=200)
    result = planner.plan(grid, [np.array([0., 0.]), np.array([3., 0.])])
    assert result.backend == 'priest_projection'
    assert path_valid(grid, result.path)
    blocked = LocalGrid(np.array([0., 0.]), np.array([[1.5, 0.]]), .2, 5., .6)
    assert not path_valid(blocked, result.path)
    assert not planner.plan(blocked, result.path).path


def test_budget_expiry_retains_valid_detour_not_unchecked_sample():
    grid = LocalGrid(np.array([0., 0.]), np.array([[2., 0.]]), .2, 6., .7)
    seed = grid.plan(np.array([0., 0.]), np.array([4., 0.]))
    result = PriestLocalPlanner(budget_ms=.00001).plan(grid, seed)
    assert result.backend == 'astar_fallback'
    assert path_valid(grid, result.path)


def test_enclosed_goal_has_no_path():
    theta = np.linspace(0, 2*np.pi, 80)
    obs = np.column_stack((3+np.cos(theta), np.sin(theta)))
    grid = LocalGrid(np.array([0., 0.]), obs, .2, 6., .4)
    assert not grid.plan(np.array([0., 0.]), np.array([3., 0.]))
