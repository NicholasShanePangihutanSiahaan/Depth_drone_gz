import numpy as np
from polinasi_nav.predictive import predict, solve


def test_tour_preset_preserves_route_and_all_existing_safety_settings():
    from pathlib import Path
    from polinasi_nav.config import load_config
    root = Path(__file__).resolve().parents[1]/'config'
    original = load_config(root/'mapping_tour.json')
    tour = load_config(root/'predictive_tour.json')
    overrides = {'predictive_enabled', 'predictive_simulation_only',
                 'predictive_adaptation', 'command_speed', 'speed',
                 'identification_setpoint_interface', 'identification_payload_kg',
                 'adaptive_model_file'}
    assert {k: v for k, v in tour.items() if k not in overrides} == {
        k: v for k, v in original.items() if k not in overrides}
    assert len(tour['survey_waypoints']) == 326
    assert not original.get('predictive_enabled', False)
    assert tour['predictive_enabled'] and tour['predictive_simulation_only']
    assert not tour['predictive_adaptation']
    assert tour['command_speed'] == original['command_speed'] == .3
    assert tour['speed'] == original['speed'] == .4


def test_tour_mpc_accepts_identified_model_and_closes_worker():
    from pathlib import Path
    from polinasi_nav.config import load_config
    from polinasi_nav.predictive import PredictiveControl
    config = load_config(Path(__file__).resolve().parents[1]/'config/predictive_tour.json')
    mpc = PredictiveControl(config)
    try:
        assert mpc.theta.shape == (3, 4)
        assert len(mpc.delays) == 3
        assert mpc.worker._max_workers == 1
        assert not mpc.activated  # Constructing a preset is not flight validation.
    finally:
        mpc.close()


def test_tour_launcher_dispatches_mapping_with_ground_truth(tmp_path):
    import os
    import shutil
    import subprocess
    from pathlib import Path
    root = Path(__file__).resolve().parents[2]
    tools = tmp_path/'tools'
    tools.mkdir()
    shutil.copyfile(root/'tools/start_predictive.sh', tools/'start_predictive.sh')
    # Substitute only the downstream launcher: no ROS, flight, or hardware commands.
    (tools/'start_simulation.sh').write_text('printf "%s\\n" "$@"\n')
    result = subprocess.run(['bash', str(tools/'start_predictive.sh'), 'tour'],
                            env={**os.environ, 'POLINASI_SIM_RTF': '0.2'},
                            capture_output=True, text=True, check=True)
    assert result.stdout.splitlines() == [
        'ground_truth', 'palm_farm', 'mapping',
        str(tmp_path/'polinasi_nav/config/predictive_tour.json')]


def job():
    return dict(stamp=1., position=np.zeros(3), velocity=np.zeros(3),
        theta=np.tile([3., 3., 1., 0.], (3, 1)), delays=[0., 0., 0.],
        history=[np.zeros(10)], reference=np.zeros((10, 9)), step=.1,
        first_dt=.05, previous_acceleration=np.zeros(3), acceleration=.5,
        jerk=1., speed=.4, tube=.06, model_version=0)


def test_mpc_solves_hover_with_feasible_zero_commands():
    request = job()
    result = solve(request)
    assert result['ok'], result['reason']
    np.testing.assert_allclose(result['controls'], 0., atol=1e-8)
    assert result['wall_ms'] < 5000.  # Not a realtime performance claim.


def test_mpc_actively_compensates_known_bias_and_preserves_limits():
    request = job()
    request['theta'][:, 3] = [.03, -.02, .01]
    nominal, _ = predict(request, np.zeros((10, 3)))
    result = solve(request)
    assert result['ok'], result['reason']
    assert np.linalg.norm(result['states'][:, :3]) < np.linalg.norm(nominal[:, :3])
    assert np.linalg.norm(result['controls'][0]) > .005
    u = result['controls']
    assert np.max(np.linalg.norm(u, axis=1)) <= .500001
    limits = np.r_[.05, np.full(9, .1)]
    assert np.all(np.linalg.norm(np.diff(np.vstack((np.zeros(3), u)), axis=0), axis=1) <= limits+1e-5)


def test_mpc_delay_is_causal_and_changes_predicted_response():
    request = job()
    controls = np.tile([.2, 0., 0.], (10, 1))
    immediate, _ = predict(request, controls)
    request['delays'] = [.2, 0., 0.]
    delayed, _ = predict(request, controls)
    assert immediate[0, 3] > 0. and delayed[0, 3] == 0.
    assert immediate[-1, 0] > delayed[-1, 0]


def test_infeasible_speed_is_not_reported_as_a_solution():
    request = job()
    request['velocity'] = [10., 0., 0.]
    result = solve(request)
    assert not result['ok']


def active_fixture():
    import json
    from pathlib import Path
    from types import SimpleNamespace
    from unittest.mock import Mock
    from concurrent.futures import Future
    from polinasi_nav.predictive import PredictiveControl
    from polinasi_nav.identification import empty_arena_fixture
    path = Path(__file__).resolve().parents[1]/'config/predictive_arena.json'
    c = json.loads(path.read_text())
    mpc = PredictiveControl(c)
    mpc.worker.shutdown()
    mpc.worker = Mock()
    mpc.worker.submit.return_value = Future()  # Deliberately pending worker.
    controller = SimpleNamespace(state='IDENTIFY', command=np.array([0., 0., 2.4]), command_v=np.zeros(3),
        command_a=np.zeros(3), failure='', grid=empty_arena_fixture(c),
        health=SimpleNamespace(pose_stamp=1.), _braking=Mock(return_value=('brake', 'hold')))
    controller.fail = lambda reason: setattr(controller, 'failure', reason)
    reference = np.tile(np.r_[controller.command, np.zeros(6)], (10, 1))
    mpc.record(.95, controller.command, controller.command_v, controller.command_a)
    mpc.plan = dict(stamp=1., model_version=0, controls=np.tile([.02, 0., 0.], (10, 1)))
    return mpc, controller, reference


def test_final_command_is_actively_changed_but_jerk_limited():
    mpc, controller, reference = active_fixture()
    result = mpc.apply(1., .01, controller, controller.command, np.zeros(3), reference, 0.)
    assert controller.failure == '' and mpc.activated
    assert abs(controller.command_a[0]-.01) < 1e-8
    np.testing.assert_allclose(result[0], [0., 0., 2.4])
    mpc.close()


def test_stale_solution_and_frame_mismatch_brake_not_nominal_fallback():
    mpc, controller, reference = active_fixture()
    mpc.plan['stamp'] = .7
    assert mpc.apply(1., .05, controller, controller.command, np.zeros(3), reference, 0.) == ('brake', 'hold')
    assert controller.failure == 'mpc_stale_solution'
    controller.failure = ''
    mpc.apply(1., .05, controller, controller.command, np.zeros(3), reference, .02)
    assert controller.failure == 'mpc_frame_alignment'
    mpc.close()


def test_unknown_predicted_corridor_rejected_by_final_map_check():
    mpc, controller, reference = active_fixture()
    controller.grid.blocked_actual[:] = True
    mpc.apply(1., .05, controller, controller.command, np.zeros(3), reference, 0.)
    assert controller.failure == 'mpc_final_safety'
    assert not mpc.activated
    mpc.close()


def test_online_coefficients_promote_only_inside_validated_trust_region():
    mpc, controller, reference = active_fixture()
    mpc.c['predictive_adaptation'] = True
    mpc.delays = [0., 0., 0.]
    mpc.history.clear()
    rng = np.random.default_rng(31)
    velocity = np.zeros(3)
    target = mpc.base.copy()
    target[:, :3] *= 1.1
    target[:, 3] += .008
    mpc.observe(0., np.zeros(3), velocity)
    for i in range(1500):
        t = i*.05
        ep, ev, ac = rng.normal(0., .12, (3, 3))
        mpc.record(t, ep, velocity+ev, ac)
        acceleration = target[:, 0]*ep+target[:, 1]*ev+target[:, 2]*ac+target[:, 3]
        velocity = velocity+.05*acceleration
        mpc.observe(t+.05, np.zeros(3), velocity)
    assert mpc.model_version > 0 and mpc.updates > 100
    limits = np.column_stack((np.maximum(.2*np.abs(mpc.base[:, :3]), .02), np.full(3, .01)))
    assert np.all(np.abs(mpc.theta-mpc.base) <= limits+1e-8)
    assert np.linalg.norm(mpc.theta-mpc.base) > .01
    from polinasi_nav.adaptive_model import stable
    assert all(stable(row) for row in mpc.theta)
    mpc.close()


def test_worker_is_started_before_first_control_request():
    mpc, _, _ = active_fixture()
    assert mpc.ready()  # Fixture waited for shutdown of the real prewarm task.
    assert mpc.solves == 0
    mpc.close()


def test_state_transition_discards_running_previous_generation_result():
    from concurrent.futures import Future
    mpc, controller, reference = active_fixture()
    mpc.sync_state('SURVEY')
    old_generation = mpc.generation
    pending = Future()
    pending.set_running_or_notify_cancel()
    mpc.future = pending
    mpc.sync_state('EXPLORE')
    mpc.sync_state('SURVEY')
    assert mpc.plan is None and mpc.future is pending
    # Even an infeasible result from the obsolete state must be discarded.
    pending.set_result(dict(stamp=.8, ok=False, reason='obsolete', wall_ms=10.,
                            generation=old_generation, queue_wall_ms=2.))
    mpc.apply(1., .05, controller, controller.command, np.zeros(3), reference, 0.)
    assert not controller.failure and mpc.plan is None
    assert mpc.queue_ms == 2.
    old_failure = Future()
    old_failure.set_exception(RuntimeError('old state task failed'))
    mpc.future = old_failure
    mpc.future_generation = old_generation
    mpc.apply(1.05, .05, controller, controller.command, np.zeros(3), reference, 0.)
    assert not controller.failure
    mpc.close()


def recovery_fixture():
    from polinasi_nav.control import MissionController
    mpc, dummy, reference = active_fixture()
    controller = MissionController(mpc.c, dummy.grid)
    controller.state = 'SURVEY'
    controller.command = dummy.command.copy()
    controller.health.localisation_ok = True
    controller.health.pose_stamp = controller.health.cloud_stamp = 1.
    return mpc, controller, reference


def test_stale_survey_solution_brakes_then_replans_with_bounded_retries():
    mpc, controller, reference = recovery_fixture()
    mpc.plan['stamp'] = .7
    mpc.apply(1., .05, controller, controller.command, np.zeros(3), reference, 0.)
    assert controller.failure == 'mpc_recovering'
    assert mpc.awaiting_fresh and mpc.recoveries == 1
    assert mpc.solution_age > .25 and mpc.applied == 0
    for _ in range(10):
        if controller.state != 'HOLD_ABORT':
            controller._braking(.05)
    assert controller.state == 'HOLD_ABORT'
    controller.health.pose_stamp = controller.health.cloud_stamp = 1.5
    mpc.poll_recovery(1.5, controller, controller.command, np.zeros(3))
    assert controller.state == 'SURVEY' and not controller.failure
    assert controller.trajectory is None and controller.goal is None
    assert mpc.awaiting_fresh  # No nominal motion until new MPC passes checks.
    mpc.sync_state('SURVEY')
    mpc.plan = dict(stamp=1.5, model_version=0, controls=np.zeros((10, 3)))
    controller.health.pose_stamp = 1.5
    mpc.apply(1.5, .05, controller, controller.command, np.zeros(3), reference, 0.)
    assert not mpc.awaiting_fresh and mpc.applied == 1
    mpc.recoveries = 3
    mpc.recover(1.3, .05, controller)
    assert controller.failure == 'mpc_stale_solution'
    mpc.close()


def test_recovery_timeout_and_sensor_loss_cannot_resume_mission():
    mpc, controller, _ = recovery_fixture()
    mpc.recover(1., .05, controller)
    mpc.poll_recovery(5.1, controller)
    assert controller.failure == 'mpc_recovery_timeout'
    controller.failure = ''
    controller.state = 'SURVEY'
    mpc.recover(6., .05, controller)
    mpc.poll_recovery(6.1, controller, controller.command, np.zeros(3))
    assert controller.failure == 'stale_localisation'
    mpc.close()


def test_fresh_wait_freezes_route_but_keeps_sensor_safety_checks():
    mpc, controller, _ = recovery_fixture()
    controller.mpc_waiting_for_fresh = True
    command = controller.command.copy()
    controller.tick(1., .05, command, np.zeros(3))
    assert controller.trajectory is None and controller.elapsed == 0.
    np.testing.assert_allclose(controller.command, command)
    controller.health.cloud_stamp = -100.
    controller.tick(1.05, .05, command, np.zeros(3))
    assert controller.failure == 'stale_lidar'
    mpc.close()


def test_recovery_brake_is_bounded_and_unknown_space_prevents_resume():
    mpc, controller, _ = recovery_fixture()
    controller.command_v = np.array([.2, 0., 0.])
    mpc.recover(1., .05, controller)
    last_a = np.zeros(3)
    for _ in range(60):
        assert np.linalg.norm(controller.command_v) <= mpc.c['speed']+1e-8
        assert np.linalg.norm(controller.command_a) <= mpc.c['acceleration']+1e-8
        assert np.linalg.norm(controller.command_a-last_a) <= mpc.c['jerk']*.05+1e-8
        last_a = controller.command_a.copy()
        if controller.state == 'HOLD_ABORT':
            break
        controller._braking(.05)
    assert controller.state == 'HOLD_ABORT'
    controller.grid.blocked_actual[:] = True
    mpc.poll_recovery(1.2, controller, controller.command, np.zeros(3))
    assert controller.failure == 'insufficient_stopping_clearance'
    mpc.close()
