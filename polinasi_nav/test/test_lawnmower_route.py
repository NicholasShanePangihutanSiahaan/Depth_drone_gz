"""Nominal coverage does not contain privileged tree-avoidance geometry."""
import importlib.util
from pathlib import Path
import numpy as np
from polinasi_nav.config import load_config
from polinasi_nav.mapping import VoxelMap, FREE, OCCUPIED
from polinasi_nav.planning import plan


def generator():
    path = Path(__file__).resolve().parents[2]/'tools/generate_mapping_configs.py'
    spec = importlib.util.spec_from_file_location('mapping_generator', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_straight_lanes_do_not_depend_on_tree_positions(monkeypatch):
    module = generator()
    # Fixed rectangular farm extent; a tree deliberately sits on a lane.
    equations = np.array([[1,0,-4],[-1,0,-4],[0,1,-4],[0,-1,-4]])
    region = ([[0.,-4.]], equations, [], [-4.,4.,-4.,4.])
    monkeypatch.setattr(module, 'farm_region', lambda: region)
    first = module.coverage_route(spacing=4., step=1.)
    region = ([[100.,100.]], equations, [], [-4.,4.,-4.,4.])
    second = module.coverage_route(spacing=4., step=1.)
    np.testing.assert_allclose(first, second)
    assert [0.,-4.,2.] in first  # No hardcoded displacement around the tree.
    route = np.vstack(([0.,0.,2.], first))
    assert np.max(np.linalg.norm(np.diff(route, axis=0), axis=1)) <= 1.+1e-8
    # Only the terminal return-to-home connector may be diagonal.
    last_lane_end = next(i for i,p in enumerate(route) if np.allclose(p,[4.,4.,2.]))
    delta = np.diff(route[:last_lane_end+1], axis=0)
    assert np.all((np.abs(delta[:,0]) < 1e-8) | (np.abs(delta[:,1]) < 1e-8))


def test_live_planner_not_generator_detours_around_measured_obstacle():
    c = load_config()
    c.update(bounds_min=[-4.,-4.,0.], bounds_max=[4.,4.,4.],
             drone_dimensions=[.2,.2,.2], clearance=.1, tracking_margin=.05)
    grid = VoxelMap(c)
    grid.state[:] = FREE
    grid.seen[:] = 0.
    grid.state[tuple(grid.indices([0.,0.,2.]))] = OCCUPIED
    grid.rebuild(0.)
    start, goal = [-2.,0.,2.], [2.,0.,2.]
    assert not grid.line_safe(start, goal)
    route = plan(grid, start, goal)
    assert route is not None and len(route) > 2
    assert all(grid.line_safe(a,b) for a,b in zip(route,route[1:]))
    # A goal inside the obstacle is not bypassed by declaring it free.
    assert plan(grid, start, [0.,0.,2.]) is None
