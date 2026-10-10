import numpy as np
import pytest
from polinasi_nav.simulated_rays import directions, normalize


def scan():
    d = directions()
    return d * 5., np.repeat(np.arange(32), 360)


def test_no_return_budget_rotates_without_reducing_obstacle_sampling():
    from polinasi_nav.mapping import select_rays
    hits = np.arange(120)%3 == 0
    all_selected = set()
    for sequence in range(12):
        selected = select_rays(hits, sequence, 4, 12)
        assert hits[selected].sum() == 10
        assert (~hits[selected]).sum() <= 7
        all_selected.update(selected.tolist())
    assert all_selected == set(range(120))
    assert select_rays([True], 11, 4, 12).tolist() == [0]
    assert len(select_rays([], 0, 4, 12)) == 0


def test_only_verified_positive_infinity_becomes_finite_range():
    points, rings = scan()
    expected = directions()
    points[100] = np.copysign(np.inf, expected[100])
    points[101] = -np.copysign(np.inf, expected[101])
    points[102] = np.nan
    original = points.copy()
    decoded, stats = normalize(points, rings, 20.)
    np.testing.assert_allclose(decoded[100], expected[100]*20.)
    np.testing.assert_array_equal(decoded[:100], original[:100])
    assert np.isinf(decoded[101]).all() and np.isnan(decoded[102]).all()
    np.testing.assert_array_equal(points, original)
    assert stats['explicit_no_return'] == 1 and stats['invalid_rays'] == 2


@pytest.mark.parametrize('bad', ['shape', 'ring', 'geometry', 'missing_hits', 'range'])
def test_unverified_data_cannot_be_assumed_free(bad):
    points, rings = scan()
    maximum = 20.
    if bad == 'shape':
        points = points[:-1]
    elif bad == 'ring':
        rings = rings[::-1]
    elif bad == 'geometry':
        points = points[:, [1, 0, 2]]
    elif bad == 'missing_hits':
        points[:] = np.nan
    else:
        maximum = np.inf
    with pytest.raises(ValueError):
        normalize(points, rings, maximum)


def test_decoded_ray_clears_only_to_range_and_preserves_obstacles():
    from polinasi_nav.config import load_config
    from polinasi_nav.mapping import VoxelMap, FREE, UNKNOWN, OCCUPIED
    c = load_config()
    c.update(bounds_min=[-6., -6., -2.], bounds_max=[6., 6., 8.], resolution=.2)
    grid = VoxelMap(c)
    points, rings = scan()
    ray = 18*360 + 180
    points[ray] = np.copysign(np.inf, directions()[ray])
    decoded, _ = normalize(points, rings, 4.)
    direction = directions()[ray]
    obstacle = tuple(grid.indices(direction*2.))
    grid.state[obstacle] = OCCUPIED
    grid.integrate(np.zeros(3), [decoded[ray], [np.nan]*3], [False, False], 1.)
    assert grid.state[tuple(grid.indices(direction))] == FREE
    assert grid.state[obstacle] == OCCUPIED
    assert grid.state[tuple(grid.indices(direction*5.))] == UNKNOWN
    assert not np.any(grid.state == OCCUPIED) or np.sum(grid.state == OCCUPIED) == 1
