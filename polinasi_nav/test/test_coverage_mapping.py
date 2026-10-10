import importlib.util
import json
import xml.etree.ElementTree as ET
from pathlib import Path
import numpy as np
import pytest
from polinasi_nav.config import load_config
from polinasi_nav.coverage_mapping import SparseHistory, RollingMapper, navigation_window
from polinasi_nav.mapping import FREE, OCCUPIED, UNKNOWN
from polinasi_nav.map_log import MapLog


def configuration():
    c = load_config()
    c.update(bounds_min=[-10., -10., 0.], bounds_max=[20., 20., 5.],
        navigation_window_size=[6., 6., 5.], map_timeout=1., free_ttl=10.,
        drone_dimensions=[.2, .2, .2], clearance=.05, tracking_margin=.05)
    return c


def test_global_hit_outside_local_window_is_saved_and_restored():
    c = configuration()
    mapper = RollingMapper(c)
    initial = navigation_window(c, [0., 0., 2.])
    grid, stamp, delta = mapper.integrate(initial, np.array([0., 0., 2.]),
        [[8., 0., 2.]], [True], 1., None, 1., [0., 0., 2.])
    assert not grid.inside(grid.indices([8., 0., 2.]))
    global_index = mapper.history.indices([8., 0., 2.])
    key = int(np.ravel_multi_index(global_index, mapper.history.shape))
    assert mapper.history.cells[key][0] == OCCUPIED
    later = navigation_window(c, [8., 0., 2.])
    mapper.history.restore(later, 2.)
    assert later.state[tuple(later.indices([8., 0., 2.]))] == OCCUPIED
    copy = SparseHistory(c)
    copy.apply_delta(delta)
    assert copy.cells == mapper.history.cells


def test_history_never_invents_free_and_expiry_survives_recentring():
    c = configuration()
    history = SparseHistory(c)
    history.update(history.indices([[0., 0., 2.]]), FREE, 1.)
    grid = navigation_window(c, [0., 0., 2.])
    history.restore(grid, 12.)
    assert grid.state[tuple(grid.indices([0., 0., 2.]))] == UNKNOWN
    assert not grid.safe([0., 0., 2.])
    assert np.all(grid.state == UNKNOWN)


def test_restore_only_looks_up_bounded_window_not_entire_history():
    class LookupOnly(dict):
        def __iter__(self):
            raise AssertionError('must not enumerate whole-farm history')
    c = configuration()
    history = SparseHistory(c)
    history.update(history.indices([[0., 0., 2.], [15., 15., 2.]]), FREE, 1.)
    history.cells = LookupOnly(history.cells)
    grid = navigation_window(c, [0., 0., 2.])
    history.restore(grid, 2.)
    assert grid.state[tuple(grid.indices([0., 0., 2.]))] == FREE
    assert np.sum(grid.state == FREE) == 1


def test_pad_prior_is_bounded_and_obstacle_wins():
    c = configuration()
    mapper = RollingMapper(c)
    initial = navigation_window(c, [0., 0., 2.])
    grid, _, _ = mapper.integrate(initial, np.array([0., 0., 2.]),
        [[8., 0., 2.]], [True], 1., (np.array([0., 0., .3]), .3), 1., [0., 0., 2.])
    prior_ids = np.column_stack(np.unravel_index(list(mapper.history.prior), mapper.history.shape))
    centres = mapper.history.lo+(prior_ids+.5)*mapper.history.res
    assert len(centres) and np.all(np.abs(centres[:, :2]) < 1.4)
    assert all(mapper.history.cells[k][0] != OCCUPIED for k in mapper.history.prior)
    # Ray-cleared cells beyond the pad must NOT accidentally become persistent priors.
    ray_key = int(np.ravel_multi_index(mapper.history.indices([2., 0., 2.]), mapper.history.shape))
    assert ray_key not in mapper.history.prior
    hit = mapper.history.indices([0., 0., 2.])
    mapper.history.update(hit, OCCUPIED, 2.)
    mapper.history.restore(grid, 100.)
    assert grid.state[tuple(grid.indices([0., 0., 2.]))] == OCCUPIED


def test_sparse_export_accounts_for_all_global_cells(tmp_path):
    history = SparseHistory(configuration())
    history.update([[1, 2, 3]], OCCUPIED, 1.)
    history.update([[2, 3, 4]], FREE, 2.)
    data = history.snapshot('ground_truth', 20., [[1., 2., 3.]], False, {})
    assert data['unknown_encoding'] == 'implicit_unlisted_cells'
    assert len(data['free'])+len(data['occupied'])+data['unknown_count'] == np.prod(history.shape)
    assert len(data['unknown']) <= 6000
    assert data['known_cell_observation_times'] == [1., 2.]
    MapLog(tmp_path).write(data)
    assert json.loads((tmp_path/'map.json').read_text())['shape'] == list(history.shape)
    from polinasi_nav.coverage_mapping import write_history_snapshot
    paths = MapLog(tmp_path).flight_paths([[0, 0, 2], [1, 0, 2]])
    write_history_snapshot(tmp_path, history, 'ground_truth', 20., [], False, {}, paths)
    assert json.loads((tmp_path/'map.json').read_text())['flight_paths'] == paths


def test_farm_preset_whole_scene_bounds_route_and_local_memory():
    root = Path(__file__).resolve().parents[2]
    c = load_config(root/'polinasi_nav/config/mapping_tour.json')
    assert c['bounds_min'][:2] == [-40., -40.]
    assert c['bounds_max'][:2] == [52., 42.]
    assert c['resolution'] == .2
    grid = navigation_window(c, c['initial_pose'])
    assert np.prod(grid.shape) < 200000
    route = np.asarray(c['survey_waypoints'])
    assert np.max(np.linalg.norm(np.diff(np.vstack(([0., 0., 2.], route)), axis=0), axis=1)) <= 2.000001
    assert len(route) > 100
    assert route[:, 0].min() > -20 and route[:, 0].max() < 50
    assert route[:, 1].min() > -25 and route[:, 1].max() < 25
    from scipy.spatial import ConvexHull
    polygon = np.asarray(c['coverage_polygon'])
    equations = ConvexHull(polygon).equations
    assert np.all(route[:, :2]@equations[:, :2].T+equations[:, 2] <= 1e-6)
    assert c['coverage_region_source'] == 'configured_simulated_tree_layout'
    np.testing.assert_allclose(route[-1], c['home'])
    length = np.sum(np.linalg.norm(np.diff(np.vstack((c['home'], route)), axis=0), axis=1))
    assert c['survey_max_duration'] > 1.875*length/c['command_speed']
    assert c['free_ttl'] > c['survey_max_duration']
    # Nominal geometric coverage only, not observed/collision-free certification.
    for x, y in c['coverage_tree_centres']:
        assert np.min(np.linalg.norm(route[:, :2]-[x, y], axis=1)) < 6.


def test_static_survey_memory_does_not_change_sensor_dropout_limits():
    root = Path(__file__).resolve().parents[2]
    c = load_config(root/'polinasi_nav/config/mapping_smoke.json')
    assert c['free_ttl'] > 15.
    assert c['cloud_timeout'] == .5 and c['pose_timeout'] == .3
    assert c['map_timeout'] == 1.


@pytest.mark.parametrize('factor', [.2, .3, .5, 1.])
def test_pacing_does_not_change_physics_geometry_or_original(tmp_path, factor):
    root = Path(__file__).resolve().parents[2]
    spec = importlib.util.spec_from_file_location('prepare_sim_world', root/'tools/prepare_sim_world.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    source = root/'polinasi_nav/worlds/palm_farm.sdf'
    before = source.read_bytes()
    target = tmp_path/'world.sdf'
    module.prepare(source, target, factor)
    original = ET.parse(source).getroot()
    changed = ET.parse(target).getroot()
    assert float(changed.findtext('world/physics/real_time_factor')) == factor
    assert changed.findtext('world/physics/max_step_size') == original.findtext('world/physics/max_step_size')
    for a, b in zip(original.findall('world/model'), changed.findall('world/model')):
        assert ET.tostring(a) == ET.tostring(b)
    assert source.read_bytes() == before


def test_mapping_sensor_runtime_copy_matches_range_without_changing_fov(tmp_path):
    root = Path(__file__).resolve().parents[2]
    spec = importlib.util.spec_from_file_location('prepare_sim_world_sensor', root/'tools/prepare_sim_world.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    target = tmp_path/'world.sdf'
    module.prepare(root/'polinasi_nav/worlds/palm_farm.sdf', target, .2,
                   root/'polinasi_nav/config/mapping_smoke.json')
    generated = ET.parse(tmp_path/'models/iris_lidar_mapping/model.sdf').getroot()
    original = ET.parse(root/'polinasi_nav/models/iris_lidar/model.sdf').getroot()
    path = ".//sensor[@name='mid360']/lidar"
    assert generated.findtext(path+'/range/max') == '20.0'
    assert ET.tostring(generated.find(path+'/scan')) == ET.tostring(original.find(path+'/scan'))
    assert json.loads((tmp_path/'mission_config.json').read_text())['lidar_range'] == 20.
    assert original.findtext(path+'/range/max') == '12'


def test_rolling_history_retention_does_not_allow_dropout_flight():
    from polinasi_nav.survey import SurveyController
    from polinasi_nav.planning import Health
    c = configuration()
    c.update(survey_waypoints=[[.25, 0., 2.]], home=[0., 0., 2.], free_ttl=10800.)
    grid = navigation_window(c, [0., 0., 2.])
    grid.seed_launch_pad([0., 0., .3], .3, 0.)
    grid.rebuild(10.)
    ctl = SurveyController(c, grid, asynchronous=False)
    ctl.state, ctl.command = 'SURVEY', np.array([0., 0., 2.])
    ctl.health = Health(0., 10., 10., True)
    ctl.tick(10., .05, ctl.command.copy(), np.zeros(3))
    assert 'stale_lidar' in ctl.failure
    assert not ctl.land_requested


def test_sitl_starts_only_supplied_imu_without_disabling_arming_checks():
    root = Path(__file__).resolve().parents[2]
    lines = (root/'polinasi_nav/config/sitl.parm').read_text().splitlines()
    params = dict(line.split()[:2] for line in lines if line.strip() and not line.startswith('#'))
    assert params['INS_ENABLE_MASK'] == '1'
    assert params['EK3_IMU_MASK'] == '1'
    for axis in 'XYZ':
        assert params['INS_ACC2OFFS_'+axis] == '0'
        assert params['INS_ACC2SCAL_'+axis] == '1'
    assert params.get('ARMING_CHECK') != '0'
