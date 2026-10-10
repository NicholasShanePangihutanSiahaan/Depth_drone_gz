import numpy as np
import pytest
from polinasi_nav.config import load_config
from polinasi_nav.coverage_mapping import navigation_window
from polinasi_nav.map_transport import encode_window, decode_window


def fixture_grid():
    c = load_config()
    c.update(navigation_window_size=[4., 4., 4.])
    grid = navigation_window(c, [0., 0., 2.])
    grid.state[4:8, 4:8, 4:8] = 0
    grid.seen[4:8, 4:8, 4:8] = 3.
    grid.last_observation_stamp, grid.version = 3., 7
    grid.rebuild(3.)
    return grid


def test_snapshot_roundtrip_preserves_numeric_origin_unknown_and_inflation():
    grid = fixture_grid()
    decoded, statistics = decode_window(encode_window(grid, {'global_known_voxels': 64}), grid)
    assert statistics['global_known_voxels'] == 64
    assert decoded.version == 7 and decoded.last_observation_stamp == 3.
    np.testing.assert_array_equal(decoded.lo, grid.lo)
    for name in ('state', 'seen', 'blocked', 'blocked_actual', 'distance', 'free_distance'):
        np.testing.assert_array_equal(getattr(decoded, name), getattr(grid, name))
    decoded.state[0, 0, 0] = 0
    assert grid.state[0, 0, 0] == -1


def test_wrong_shape_and_oversize_are_rejected():
    grid = fixture_grid()
    payload = encode_window(grid, {})
    grid.shape = (1, 1, 1)
    with pytest.raises(ValueError, match='shape/type'):
        decode_window(payload, grid)
    with pytest.raises(ValueError, match='bounded'):
        decode_window(bytes(8_000_001), grid)
    with pytest.raises(ValueError, match='archive'):
        decode_window(b'not a map', grid)


def test_invalid_measurement_and_distance_are_rejected():
    grid = fixture_grid()
    grid.state[0, 0, 0] = 5
    with pytest.raises(ValueError, match='measurement'):
        decode_window(encode_window(grid, {}), grid)
    grid.state[0, 0, 0] = -1
    grid.distance[0, 0, 0] = np.nan
    with pytest.raises(ValueError, match='distance'):
        decode_window(encode_window(grid, {}), grid)
