import copy
import time
from concurrent.futures import Future
import numpy as np
import pytest
from polinasi_nav.config import load_config
from polinasi_nav.mapping import VoxelMap, FREE, OCCUPIED, UNKNOWN
from polinasi_nav.planning import Health
from polinasi_nav.survey import SurveyController


def make(asynchronous=False):
    c = load_config()
    c.update(bounds_min=[-3., -3., 0.], bounds_max=[4., 3., 5.],
             drone_dimensions=[.2, .2, .2], clearance=.05, tracking_margin=.05,
             tracking_limit=.05, free_ttl=1000., survey_dwell=.1,
             survey_waypoints=[[.5, 0., 2.], [1., .5, 2.]],
             home=[0., 0., 1.5], exploration_enabled=False)
    grid = VoxelMap(c)
    grid.state[:] = FREE
    grid.seen[:] = 0.
    grid.state[:, :, 0] = OCCUPIED
    grid.rebuild(0.)
    return c, grid, SurveyController(c, grid, asynchronous=asynchronous)


def step(controller, now, observe=True):
    controller.health = Health(now, now, now, True)
    if observe:
        controller.grid.last_observation_stamp = now
    return controller.tick(now, .05, controller.command.copy(), controller.command_v.copy())


def test_takeoff_waypoints_return_descend_land_without_buzzer():
    c, grid, ctl = make()
    states = set()
    for i in range(3000):
        step(ctl, i*.05)
        ctl.flower_event(True)
        states.add(ctl.state)
        if ctl.land_requested:
            break
    assert {'TAKEOFF', 'SURVEY', 'RETURN', 'DESCEND', 'LAND'} <= states
    assert ctl.waypoint_index == len(c['survey_waypoints'])
    assert ctl.land_requested and not ctl.failure
    assert ctl.buzzer_events == 0 and ctl.buzzer_remaining == 0
    assert ctl.nominal == []
    assert c['mock_flower_events']  # Parent inspection config was not mutated.


def test_waypoint_requires_a_new_integrated_scan():
    c, grid, ctl = make()
    ctl.state = 'SURVEY'
    for i in range(300):
        step(ctl, i*.05, observe=False)
        if ctl.wait_for_scan is not None:
            break
    assert ctl.wait_for_scan is not None
    for j in range(20):
        step(ctl, (i+j+1)*.05, observe=False)
    assert ctl.waypoint_index == 0
    grid.last_observation_stamp = (i+21)*.05
    step(ctl, (i+21)*.05)
    assert ctl.waypoint_index == 1


def test_return_uses_checked_local_legs_and_does_not_descend_early():
    c, grid, ctl = make()
    ctl.state = 'RETURN'
    ctl.c['home'] = [-8., 0., 1.5]
    current = np.array([0., 0., 2.])
    assert ctl._choose_goal(current)
    np.testing.assert_allclose(ctl.goal, [-2., 0., 2.])
    np.testing.assert_allclose(ctl._mission_hint(current), ctl.goal)
    ctl._advance()
    assert ctl.state == 'RETURN' and not ctl.land_requested
    ctl.goal = np.array([-8., 0., 2.])
    ctl._advance()
    assert ctl.state == 'DESCEND'


def test_unknown_route_holds_without_counting_waypoints():
    c, grid, ctl = make()
    ctl.state = 'SURVEY'
    grid.state[grid.indices([.5, 0., 2.])[0]:, :, :] = UNKNOWN
    grid.rebuild(0.)
    ctl._prepare_trajectory(0., ctl.command)
    assert ctl.trajectory is None and ctl.waypoint_index == 0
    ctl._prepare_trajectory(c['blocked_timeout']+1., ctl.command)
    assert ctl.failure == 'blocked_route'


def test_planner_stage_breakdown_counts_braking_checks_without_changing_route():
    c, grid, ctl = make()
    ctl.state = 'SURVEY'
    assert ctl._prepare_trajectory(0., ctl.command)
    stages = ctl.planner_stages_ms
    for name in ('goal_and_route_search', 'trajectory_construction',
                 'smoothed_collision_check', 'braking_certification'):
        assert stages[name] >= 0.
    assert stages['braking_sample_checks'] > 0
    assert 1 <= stages['braking_attempts'] <= 12
    assert ctl.trajectory.safe(grid) and not ctl.failure


def test_dropout_stops_survey_and_does_not_request_unchecked_landing():
    c, grid, ctl = make()
    ctl.state = 'SURVEY'
    step(ctl, 0.)
    ctl.health = Health(-10., 1., 1., True)
    ctl.tick(1., .05, ctl.command, ctl.command_v)
    assert ctl.failure == 'stale_lidar'
    assert not ctl.land_requested and ctl.waypoint_index == 0


def test_asynchronous_plan_waits_at_rest_and_does_not_replace_live_health():
    c, grid, ctl = make(asynchronous=True)
    try:
        ctl.state = 'SURVEY'
        before = ctl.command.copy()
        step(ctl, 0.)
        np.testing.assert_array_equal(ctl.command, before)
        assert ctl.pending_plan is not None
        ctl.pending_plan.result(timeout=10.)
        health = ctl.health
        ctl._prepare_trajectory(.05, ctl.command)
        assert ctl.trajectory is not None and ctl.health is health
        np.testing.assert_array_equal(ctl.command, before)
    finally:
        ctl.close()


def test_prefetch_is_used_only_at_matching_stopped_boundary():
    c, grid, ctl = make(asynchronous=True)
    try:
        ctl.c.update(continuous_survey=True, survey_prefetch=True, survey_prefetch_seconds=100.,
            adaptive_rate_limit=.25, adaptive_rate_acceleration=.4, adaptive_turn_acceleration=.25)
        # Force one-target chunks to exercise a boundary in a small fixture.
        ctl.c['survey_batch_points'] = 1
        ctl.state = 'SURVEY'
        ctl._prepare_trajectory(0., ctl.command)
        ctl.pending_plan.result(timeout=10.)
        assert ctl._prepare_trajectory(.05, ctl.command)
        assert ctl.batch_end_index == 0
        ctl._prefetch_next(.1)
        assert ctl.prefetched_plan is not None
        ctl.prefetched_plan.result(timeout=10.)
        ctl.command = ctl.goal.copy()
        ctl.command_v = ctl.command_a = np.zeros(3)
        ctl.waypoint_index = 1
        ctl.goal = ctl.trajectory = None
        assert ctl._prepare_trajectory(.2, ctl.command)
        assert ctl.prefetch_used == 1 and ctl.prefetched_plan is None
        assert ctl.trajectory.safe(grid) and not ctl.failure
    finally:
        ctl.close()


def test_async_result_invalidated_by_new_obstacle_is_discarded():
    c, grid, ctl = make(asynchronous=True)
    try:
        ctl.state = 'SURVEY'
        ctl._prepare_trajectory(0., ctl.command)
        ctl.pending_plan.result(timeout=10.)
        grid.state[tuple(grid.indices(c['survey_waypoints'][0]))] = OCCUPIED
        grid.rebuild(.05)
        assert not ctl._prepare_trajectory(.05, ctl.command)
        assert ctl.trajectory is None and ctl.waypoint_index == 0
    finally:
        ctl.close()


@pytest.mark.parametrize('points', [[], [[float('nan'), 0., 2.]], [[100., 0., 2.]], [[1., 2.]]])
def test_bad_survey_waypoints_rejected(points):
    c, grid, _ = make()
    c['survey_waypoints'] = points
    with pytest.raises(ValueError, match='survey'):
        SurveyController(c, grid, asynchronous=False)
