import copy
import numpy as np
import pytest
from polinasi_nav.config import load_config
from polinasi_nav.mapping import VoxelMap, FREE
from polinasi_nav.identification import Excitation, IdentificationController, protocol
from polinasi_nav.adaptive_model import GuardedRLS, stable
from polinasi_nav.model_fitting import samples, design, rollout_metrics


def config():
    c = load_config()
    c.update(identification_profile='train', command_speed=.3, home=[0., 0., 2.4],
             bounds_min=[-3., -3., 0.], bounds_max=[3., 3., 5.], free_ttl=10000.)
    return c


def test_excitation_analytic_derivatives_and_zero_endpoint_pva():
    signal = Excitation(14., .14, .1, .08)
    np.testing.assert_array_equal(signal.sample(0.), 0.)
    np.testing.assert_array_equal(signal.sample(14.), 0.)
    for t in np.linspace(.1, 13.9, 30):
        h = 1e-5
        np.testing.assert_allclose(signal.sample(t)[1:], (signal.sample(t+h)[:-1]-signal.sample(t-h)[:-1])/(2*h), atol=1e-7)


def test_identification_completes_and_never_calls_flower_or_pid_tuning():
    c = config()
    grid = VoxelMap(c)
    grid.state[:] = FREE
    grid.seen[:] = 0.
    grid.rebuild(0.)
    controller = IdentificationController(c, grid)
    controller.state = 'IDENTIFY'
    controller.health.localisation_ok = True
    for t in np.arange(.05, 102., .05):
        controller.health.pose_stamp = controller.health.cloud_stamp = t
        p, v = controller.command.copy(), controller.command_v.copy()
        controller.tick(t, .05, p, v)
        if controller.land_requested:
            break
    assert controller.completed_protocol and controller.state == 'LAND'
    assert controller.failure == '' and controller.buzzer_events == 0
    np.testing.assert_allclose(controller.command, c['home'])


def test_identification_dropout_uses_existing_brake_hold():
    c = config()
    grid = VoxelMap(c)
    grid.state[:] = FREE
    grid.seen[:] = 0.
    grid.rebuild(0.)
    controller = IdentificationController(c, grid)
    controller.state = 'IDENTIFY'
    controller.health.localisation_ok = True
    controller.health.pose_stamp = 1.
    controller.health.cloud_stamp = 0.
    controller.tick(1., .05, controller.command, np.zeros(3))
    assert controller.failure == 'stale_lidar'
    assert not controller.completed_protocol


def test_empty_arena_identification_needs_no_cloud_but_still_needs_localisation():
    from polinasi_nav.identification import empty_arena_fixture
    c = config()
    c.update(identification_simulation_only=True, identification_empty_arena=True,
             bounds_min=[-5., -5., -.6], bounds_max=[5., 5., 5.4])
    grid = empty_arena_fixture(c)
    controller = IdentificationController(c, grid)
    controller.state = 'IDENTIFY'
    controller.health.localisation_ok = True
    controller.health.pose_stamp = 1.
    controller.tick(1., .05, controller.command, np.zeros(3))
    assert controller.failure == ''
    assert grid.safe(c['home'])
    assert not grid.safe([0., 0., 0.])
    assert not grid.safe([5., 0., 2.4])
    # The ordinary navigation gate is unchanged even with this config.
    assert controller.health.reason(1., c) == 'stale_lidar'
    controller.tick(2., .05, controller.command, np.zeros(3))
    assert controller.failure == 'stale_localisation'
    with pytest.raises(ValueError, match='simulation'):
        empty_arena_fixture(config())


def test_identification_world_removes_lasers_and_camera_not_imu_or_physics(tmp_path):
    import importlib.util
    import json
    import xml.etree.ElementTree as ET
    from pathlib import Path
    root = Path(__file__).resolve().parents[2]
    spec = importlib.util.spec_from_file_location('prepare_sim_world', root/'tools/prepare_sim_world.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    module.prepare(root/'polinasi_nav/worlds/identification.sdf', tmp_path/'world.sdf', .2,
                   root/'polinasi_nav/config/identification_train.json')
    vehicle = ET.parse(tmp_path/'models/iris_lidar_mapping/model.sdf')
    sensors = vehicle.getroot().findall('.//sensor')
    assert sensors and all(s.get('type') == 'imu' for s in sensors)
    original = ET.parse(root/'polinasi_nav/models/iris_lidar/model.sdf')
    assert len(original.getroot().findall(".//sensor[@type='gpu_lidar']")) == 2
    assert ET.tostring(vehicle.getroot().find('.//inertial')) == ET.tostring(original.getroot().find('.//inertial'))
    assert len(vehicle.getroot().findall('.//plugin')) == len(original.getroot().findall('.//plugin'))
    assert json.loads((tmp_path/'mission_config.json').read_text())['identification_empty_arena']


def test_rls_freezes_without_excitation_and_on_invalid_data():
    model = GuardedRLS(window=40)
    seed, covariance = model.theta.copy(), model.P.copy()
    for _ in range(1000):
        model.update([0., 0., 0., 1.], 0., .05)
    np.testing.assert_array_equal(model.theta, seed)
    np.testing.assert_array_equal(model.P, covariance)
    assert model.reason == 'insufficient_excitation'
    assert not model.update([np.nan, 0., 0., 1.], 0., .05)
    assert not model.update([.1, .2, .3, 1.], 0., .2)
    assert not model.update([.1, .2, .3, 1.], 100., .05)
    assert not model.update([.1, .2, .3, 1.], 0., .05, healthy=False)


def test_rls_tracks_changed_effective_gains_and_remains_bounded_stable():
    rng = np.random.default_rng(4)
    model = GuardedRLS(window=40, forgetting=.99)
    target = np.array([4., 4., .8, .03])
    for i in range(3500):
        if i == 1500:
            target = np.array([2., 2.8, 1.4, -.04])
        phi = np.r_[rng.normal(0., .15, 3), 1.]
        old = model.theta.copy()
        model.update(phi, phi@target, .05)
        assert stable(model.theta)
        assert np.max(np.abs(model.theta-old)/np.array([2., 2., .5, .2])) <= .050001
        assert np.linalg.eigvalsh(model.P).max() <= 100.00001
    np.testing.assert_allclose(model.theta, target, atol=.08)
    assert model.accepted > 1000 and model.snapshot()['controls_flight'] is False


def test_correlated_inputs_freeze_estimator():
    model = GuardedRLS(window=40)
    for i in range(100):
        x = np.sin(i*.1)
        model.update([x, 2*x, 3*x, 1.], .1, .05)
    assert model.accepted == 0 and model.reason == 'correlated_inputs'


def test_shadow_seed_requires_independent_validation_and_matching_envelope(tmp_path):
    import json
    from polinasi_nav.adaptive_model import load_validated_seed, INITIAL
    path = tmp_path/'model.json'
    c = dict(command_speed=.3)
    seeds, delays, source = load_validated_seed(path, c)
    np.testing.assert_array_equal(seeds[0], INITIAL)
    assert source == 'nominal'
    report = dict(validation_passed=True, frame='map_ENU', interface='PVA_ENU',
        identification_envelope=dict(command_speed=.3), axes={axis: dict(
            validation_passed=True, parameters=[4., 3.5, 1.2, .02], delay_seconds=.1) for axis in 'xyz'})
    path.write_text(json.dumps(report))
    seeds, delays, source = load_validated_seed(path, c)
    assert delays == [.1]*3 and source == str(path)
    with pytest.raises(ValueError, match='mismatch'):
        load_validated_seed(path, dict(command_speed=.6))
    report['validation_passed'] = False
    path.write_text(json.dumps(report))
    with pytest.raises(ValueError, match='validated'):
        load_validated_seed(path, c)


def test_validation_rollout_has_no_teacher_forcing_inside_horizon():
    theta = np.array([3., 3., 1., 0.])
    commands, responses = [], []
    p = v = 0.
    for i in range(200):
        t = i*.05
        command = dict(t=t, p=[.1*np.sin(t), 0., 0.], v=[.1*np.cos(t), 0., 0.], a=[-.1*np.sin(t), 0., 0.])
        commands.append(command)
        responses.append(dict(t=t, p=[p, 0., 0.], v=[v, 0., 0.], healthy=True))
        a = theta[0]*(command['p'][0]-p)+theta[1]*(command['v'][0]-v)+theta[2]*command['a'][0]
        p, v = p+.05*v, v+.05*a
    data = dict(commands=commands, responses=responses)
    result = rollout_metrics(data, 0, theta, 0.)
    assert result['position_max_error_m'] < .002
    wrong = rollout_metrics(data, 0, [1., 1., .1, 0.], 0.)
    assert wrong['position_rmse_m'] > result['position_rmse_m']+.005


def test_offline_fit_recovers_synthetic_effective_model_and_rejects_partial():
    from polinasi_nav.model_fitting import fit, validate
    rng = np.random.default_rng(8)
    theta = np.array([4., 3.5, 1.2, .02])
    commands, responses = [], []
    p, v = np.zeros(3), np.zeros(3)
    for i in range(350):
        command = dict(t=i*.05, p=rng.normal(0., .1, 3).tolist(),
                       v=rng.normal(0., .1, 3).tolist(), a=rng.normal(0., .1, 3).tolist())
        commands.append(command)
        responses.append(dict(t=i*.05, p=p.tolist(), v=v.tolist(), healthy=True))
        a = theta[0]*(command['p']-p)+theta[1]*(command['v']-v)+theta[2]*np.asarray(command['a'])+theta[3]
        p, v = p+.05*v, v+.05*a
    data = dict(completed=True, one_final_publisher=True, commands=commands, responses=responses)
    models = fit(data)
    for item in models.values():
        np.testing.assert_allclose(item['parameters'], theta, atol=1e-6)
        assert item['delay_seconds'] == 0.
    data['completed'] = False
    with pytest.raises(ValueError, match='completed'):
        fit(data)
