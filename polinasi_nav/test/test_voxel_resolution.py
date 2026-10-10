import json
import copy
from pathlib import Path
import numpy as np
import pytest
from scipy.ndimage import binary_dilation
from polinasi_nav.config import load_config
from polinasi_nav.mapping import VoxelMap
from polinasi_nav.map_log import MapLog

ROOT = Path(__file__).resolve().parents[1]


@pytest.mark.parametrize('resolution', [.2, .3, .4])
def test_fast_inflation_is_exactly_original_safety_kernel(resolution):
    c = load_config()
    c.update(resolution=resolution, bounds_min=[-2., -2., 0.], bounds_max=[2., 2., 4.])
    grid = VoxelMap(c)
    source = np.random.default_rng(2).random(grid.shape) < .004
    source[0, 0, 0] = True
    for kernel in (grid.kernel, grid.actual_kernel):
        reference = binary_dilation(source, structure=kernel, border_value=1)
        np.testing.assert_array_equal(grid.inflate(source, kernel), reference)


def test_navigation_and_export_use_twenty_centimetre_voxels(tmp_path):
    for name in ('navigation.json', 'palm_farm_navigation.json'):
        c = load_config(ROOT/'config'/name)
        assert c['resolution'] == .2
        c.update(bounds_min=[0., 0., 0.], bounds_max=[2., 2., 2.])
        grid = VoxelMap(c)
        assert grid.shape == (10, 10, 10)
        assert np.linalg.norm(grid.centers([1, 0, 0])-grid.centers([0, 0, 0])) == pytest.approx(.2)
        log = MapLog(tmp_path/name)
        log.write(log.snapshot(grid, 'ground_truth', 0.))
        saved = json.loads((tmp_path/name/'map.json').read_text())
        assert saved['resolution'] == .2
        assert len(saved['unknown']) == 1000  # No export downsampling.


@pytest.mark.parametrize('resolution', [float('nan'), float('inf'), 0, -1, True])
def test_invalid_voxel_resolution_rejected(tmp_path, resolution):
    c = load_config()
    c['resolution'] = resolution
    path = tmp_path/'bad.json'
    path.write_text(json.dumps(c))
    with pytest.raises(ValueError, match='resolution'):
        load_config(path)


def test_live_expiry_matches_full_rebuild_without_erasing_obstacles():
    c = load_config()
    c.update(bounds_min=[-2., -2., 0.], bounds_max=[2., 2., 4.])
    grid = VoxelMap(c)
    grid.state[:] = 0
    grid.seen[:] = 0.
    grid.state[4, 5, 6] = 1
    grid.seen[9:12, 9:12, 9:12] = c['free_ttl']
    grid.rebuild(0.)
    reference = copy.deepcopy(grid)
    grid.expire(c['free_ttl']+1.)
    reference.rebuild(c['free_ttl']+1.)
    np.testing.assert_array_equal(grid.state, reference.state)
    np.testing.assert_array_equal(grid.blocked, reference.blocked)
    np.testing.assert_array_equal(grid.blocked_actual, reference.blocked_actual)
    np.testing.assert_array_equal(grid.distance, reference.distance)
    assert grid.state[4, 5, 6] == 1
