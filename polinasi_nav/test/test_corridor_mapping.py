import copy
import numpy as np
from pathlib import Path
from polinasi_nav.corridor_mapping import contains, sample_intersections, CorridorRegion, lookahead
from polinasi_nav.coverage_mapping import SparseHistory
from polinasi_nav.config import load_config
from polinasi_nav.mapping import FREE, OCCUPIED


def roi():
    return dict(centre=[0., 0., 2.], near_radius=2., radius=.5,
        routes=[[[0., 0., 2.], [6., 0., 2.]]], expansion_centres=[], expansion_radius=1.)


def test_far_hit_keeps_near_free_evidence_without_mapping_far_obstacle():
    c = load_config()
    c.update(bounds_min=[-5., -5., 0.], bounds_max=[20., 8., 6.])
    h = SparseHistory(c)
    h.integrate(np.array([0., 0., 2.]), [[0., 15., 2.]], [True], 1., roi())
    assert h.cells[int(np.ravel_multi_index(h.indices([0., 1., 2.]), h.shape))][0] == FREE
    assert int(np.ravel_multi_index(h.indices([0., 3., 2.]), h.shape)) not in h.cells
    assert not h.occupied_keys
    stats = h.last_integration_stats['clipping']
    assert stats['generated_samples'] < stats['full_trace_samples']/4


def test_clipped_sampling_matches_full_sampling_filtered_to_exact_roi():
    rng = np.random.default_rng(7)
    origin = np.array([.013, -.019, 2.007])
    points = origin+rng.normal(size=(80, 3))*8
    region = roi()
    region['expansion_centres'] = [[5., 3., 2.]]
    region['expansion_radius'] = 1.3
    spacing = .2/3
    lengths = np.linalg.norm(points-origin, axis=1)
    expected = np.vstack([origin+np.arange(0., length, spacing)[:, None]/length*(p-origin)
                          for p, length in zip(points, lengths)])
    expected = expected[contains(expected, region)]
    actual, stats = sample_intersections(origin, points, region, spacing)
    np.testing.assert_allclose(actual, expected, atol=1e-12)
    assert stats['generated_samples'] < stats['full_trace_samples']


def test_hit_in_roi_is_obstacle_and_no_space_is_cleared_behind_it():
    c = load_config()
    c.update(bounds_min=[-5., -5., 0.], bounds_max=[20., 8., 6.])
    h = SparseHistory(c)
    h.integrate(np.array([0., 0., 2.]), [[3., 0., 2.], [np.nan]*3], [True, False], 1., roi())
    key = int(np.ravel_multi_index(h.indices([3., 0., 2.]), h.shape))
    assert h.cells[key][0] == OCCUPIED
    assert int(np.ravel_multi_index(h.indices([4., 0., 2.]), h.shape)) not in h.cells


def test_parallel_ray_outside_all_regions_is_discarded():
    points, stats = sample_intersections([0., 5., 2.], [[15., 5., 2.]], roi(), .1)
    assert len(points) == 0 and stats['rays_without_roi_intersection'] == 1


def test_disjoint_regions_do_not_clear_gap_between_them():
    r = roi()
    r['routes'] = []
    r['expansion_centres'] = [[6., 0., 2.]]
    points, _ = sample_intersections([0., 0., 2.], [[15., 0., 2.]], r, .1)
    assert np.any(points[:, 0] > 5.)
    assert not np.any((points[:, 0] > 2.01) & (points[:, 0] < 4.99))


def test_expansion_requires_occupied_blockage_and_fresh_progress_is_bounded():
    c = load_config(Path(__file__).resolve().parents[1]/'config/predictive_tour_check.json')
    region = CorridorRegion(c)
    status = dict(route_blockage=dict(reason='route_unknown', unknown_count=20, occupied_count=0))
    a = region.region([0., 0., 2.], status, [[3., 0., 2.]], [], 1.)
    assert a['expansions'] == 0
    status['route_blockage'].update(occupied_count=1, occupied_samples=[[2., 0., 2.]])
    b = region.region([0., 0., 2.], status, [[3., 0., 2.]], [], 2.)
    assert b['expansion_radius'] > a['expansion_radius']
    assert region.region([0., 0., 2.], status, [], [], 2.)['expansions'] == 1
    for stamp in range(3, 20):
        region.region([0., 0., 2.], status, [], [], stamp)
    assert region.radius == c['mapping_corridor_max_radius']
    # Selecting a larger region creates no measured free voxels by itself.
    h = SparseHistory(c)
    assert not h.cells


def test_lookahead_follows_route_length_and_roi_keeps_near_360():
    path = lookahead([0., 0., 2.], [[3., 0., 2.], [3., 4., 2.]], 5.)
    np.testing.assert_allclose(path[-1], [3., 2., 2.])
    assert contains([[-1., 0., 2.], [0., 1., 2.], [5., 0., 2.], [5., 2., 2.]], roi()).tolist() == [True, True, True, False]


def test_observed_expansion_opens_detour_but_not_fully_blocked_route():
    # Known-map fixture, not a LiDAR/ArduPilot flight validation.
    from polinasi_nav.mapping import VoxelMap, UNKNOWN
    from polinasi_nav.planning import plan
    c = load_config()
    c.update(bounds_min=[-1., -3., 0.], bounds_max=[5., 3., 5.], resolution=.2,
             drone_dimensions=[.2, .2, .2], clearance=0., tracking_margin=0., planner_max_expansions=20000)
    g = VoxelMap(c)
    ids = np.indices(g.shape).reshape(3, -1).T
    centres = g.centers(ids)
    r = dict(centre=[0., 0., 2.], near_radius=.8, radius=.8,
             routes=[[[0., 0., 2.], [4., 0., 2.]]], expansion_centres=[], expansion_radius=.8)
    def observed_grid(region, fully_blocked=False):
        g.state[:] = UNKNOWN
        g.seen[:] = 1.
        mask = contains(centres, region)
        g.state[tuple(ids[mask].T)] = FREE
        wall = (np.abs(centres[:, 0]-2.) < .21) & ((np.abs(centres[:, 1]) < 1.1) | fully_blocked)
        g.state[tuple(ids[wall].T)] = OCCUPIED
        g.rebuild(1.)
    observed_grid(r)
    assert plan(g, [0., 0., 2.], [4., 0., 2.]) is None
    r.update(expansion_centres=[[2., 0., 2.]], expansion_radius=2.5)
    observed_grid(r)
    route = plan(g, [0., 0., 2.], [4., 0., 2.])
    assert route is not None
    assert all(g.line_safe(a, b) for a, b in zip(route, route[1:]))
    observed_grid(r, fully_blocked=True)
    assert plan(g, [0., 0., 2.], [4., 0., 2.]) is None
