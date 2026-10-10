import numpy as np
import pytest
from polinasi_nav.config import load_config
from polinasi_nav.mapping import VoxelMap, FREE, OCCUPIED, UNKNOWN
from polinasi_nav.continuous import ContinuousTrajectory, AdaptivePhase, curve_safe
from polinasi_nav.survey import SurveyController
from polinasi_nav.planning import Health


def setup():
    c = load_config()
    c.update(bounds_min=[-4., -4., 0.], bounds_max=[8., 5., 5.], resolution=.1,
             drone_dimensions=[.2, .2, .2], clearance=.05, tracking_margin=.1,
             tracking_limit=.1, free_ttl=10000., command_speed=.3, speed=.4,
             survey_waypoints=[[1., 0., 2.], [2., 0., 2.], [3., .5, 2.]],
             home=[0., 0., 2.], exploration_enabled=False, survey_dwell=.1,
             continuous_survey=True, survey_batch_points=4, survey_pass_radius=.15)
    grid = VoxelMap(c)
    grid.state[:] = FREE
    grid.seen[:] = 0.
    grid.state[:, :, 0] = OCCUPIED
    grid.rebuild(0.)
    return c, grid


def test_local_profile_long_straights_not_scaled_by_short_corner():
    c, grid = setup()
    points = [[0,0,2], [2,0,2], [4,0,2], [4.2,0,2], [4.2,.2,2], [6,.2,2]]
    legacy = ContinuousTrajectory(points, c)
    c.update(local_speed_profile=True, trajectory_geometry_cache=True)
    tr = ContinuousTrajectory(points, c)
    assert tr.duration < .75*legacy.duration
    assert tr.knot_speeds[1] > tr.knot_speeds[3]
    for end in tr.ends[:-1]:
        left, right = tr.derivatives(end-1e-7), tr.derivatives(end+1e-7)
        for order in range(3):
            np.testing.assert_allclose(left[order], right[order], atol=1e-6)
    v, a, j = tr.bounds()
    assert v <= c['command_speed']+1e-9
    assert a+v*.25 <= c['acceleration']+1e-9
    assert j+3*a*.25+v*.4 <= c['jerk']+1e-9
    assert tr.safe(grid)


@pytest.mark.parametrize('state', [UNKNOWN, OCCUPIED])
def test_geometry_certificate_cache_rechecks_changed_blocking_cells(state):
    c, grid = setup()
    c.update(local_speed_profile=True, trajectory_geometry_cache=True)
    tr = ContinuousTrajectory([[0,0,2], [2,0,2], [2,2,2]], c)
    assert tr.safe(grid)
    count = tr.geometry_checks
    assert tr.safe(grid) and tr.geometry_cache_hits == 1 and tr.geometry_checks == count
    # An unrelated change is fine; a changed cell in the curve's certified
    # boxes MUST invalidate the cache even if a caller forgets a version bump.
    hit = tr.sample(tr.ends[0]*.8)[0]
    grid.state[tuple(grid.indices(hit))] = state
    grid.rebuild(0.)
    assert not tr.safe(grid)
    assert tr.geometry_checks == count+1


def test_continuous_knots_have_shared_nonzero_velocity_and_acceleration():
    c, grid = setup()
    tr = ContinuousTrajectory([[0, 0, 2], [1, 0, 2], [2, 1, 2]], c)
    p, v, a = tr.sample(tr.ends[0])
    np.testing.assert_allclose(p, [1, 0, 2])
    assert np.linalg.norm(v) > .01
    left, right = tr.sample(tr.ends[0]-1e-6), tr.sample(tr.ends[0]+1e-6)
    np.testing.assert_allclose(left[1], right[1], atol=1e-6)
    np.testing.assert_allclose(left[2], right[2], atol=1e-6)
    np.testing.assert_allclose(tr.sample(0.)[1:], 0., atol=1e-9)
    np.testing.assert_allclose(tr.sample(tr.duration)[1:], 0., atol=1e-9)
    assert tr.safe(grid)


def test_bezier_derivatives_match_finite_differences_and_global_bounds():
    c, _ = setup()
    tr = ContinuousTrajectory([[0, 0, 2], [1, 0, 2], [2, 1, 2]], c)
    for t in np.linspace(.1, tr.duration-.1, 40):
        p, v, a, j = tr.derivatives(t)
        h = 1e-4
        before, after = tr.derivatives(t-h), tr.derivatives(t+h)
        np.testing.assert_allclose(v, (after[0]-before[0])/(2*h), atol=1e-7)
        np.testing.assert_allclose(a, (after[1]-before[1])/(2*h), atol=1e-7)
        np.testing.assert_allclose(j, (after[2]-before[2])/(2*h), atol=1e-7)
    v, a, j = tr.bounds()
    assert v <= c['command_speed']+1e-9
    assert a+v*.25 <= c['acceleration']+1e-9
    assert j+3*a*.25+v*.4 <= c['jerk']+1e-9


def test_short_detour_segments_do_not_inherit_long_leg_duration():
    c, grid = setup()
    tr = ContinuousTrajectory([[0, 0, 2], [.2, 0, 2], [.4, .1, 2], [2.4, .1, 2]], c)
    assert tr.durations[0] < .15*tr.durations[-1]
    for end in tr.ends[:-1]:
        left, right = tr.derivatives(end-1e-7), tr.derivatives(end+1e-7)
        np.testing.assert_allclose(left[0], right[0], atol=1e-6)
        np.testing.assert_allclose(left[1], right[1], atol=1e-6)
        np.testing.assert_allclose(left[2], right[2], atol=1e-6)
    v, a, j = tr.bounds()
    assert v <= c['command_speed']+1e-9
    assert a+v*.25 <= c['acceleration']+1e-9
    assert j+3*a*.25+v*.4 <= c['jerk']+1e-9
    assert tr.safe(grid)


def test_reversal_stops_instead_of_inventing_a_tight_uturn():
    c, _ = setup()
    tr = ContinuousTrajectory([[0, 0, 2], [1, 0, 2], [0, 0, 2]], c)
    np.testing.assert_allclose(tr.sample(tr.ends[0])[1], 0., atol=1e-9)


@pytest.mark.parametrize('state', [OCCUPIED, UNKNOWN])
def test_curve_checks_reject_thin_obstacle_or_unknown_between_knots(state):
    c, grid = setup()
    tr = ContinuousTrajectory([[0, 0, 2], [2, 0, 2], [2, 2, 2]], c)
    hit = tr.sample(tr.ends[0]*.8)[0]
    grid.state[tuple(grid.indices(hit))] = state
    grid.rebuild(0.)
    assert not tr.safe(grid)


def test_time_scaling_derivatives_are_continuous_bounded_and_consistent():
    c, _ = setup()
    tr = ContinuousTrajectory([[0, 0, 2], [1, 0, 2], [2, 1, 2]], c)
    phase = AdaptivePhase(c)
    elapsed, samples = 0., []
    for i in range(800):
        dt = .005
        elapsed += phase.step(dt, .3 if i < 450 else 1.)
        p, v, a, j = tr.derivatives(elapsed)
        actual_v = v*phase.rate
        actual_a = a*phase.rate**2+v*phase.rate_dot
        actual_j = j*phase.rate**3+3*a*phase.rate*phase.rate_dot+v*phase.rate_ddot
        samples.append((p, actual_v, actual_a))
        assert abs(phase.rate_dot) <= .25+1e-8 and abs(phase.rate_ddot) <= .4+1e-8
        assert np.linalg.norm(actual_v) <= c['command_speed']+1e-7
        assert np.linalg.norm(actual_a) <= c['acceleration']+1e-7
        assert np.linalg.norm(actual_j) <= c['jerk']+1e-7
    for before, after in zip(samples[:-1], samples[1:]):
        np.testing.assert_allclose((after[0]-before[0])/.005,
                                   (after[1]+before[1])/2, atol=2e-5)
        np.testing.assert_allclose((after[1]-before[1])/.005,
                                   (after[2]+before[2])/2, atol=2e-5)


def test_mapping_batches_pass_goals_without_stopping_and_still_land():
    c, grid = setup()
    ctl = SurveyController(c, grid, asynchronous=False)
    ctl.state = 'SURVEY'
    pass_speeds = []
    previous = 0
    for i in range(5000):
        now = i*.05
        ctl.health = Health(now, now, now, True)
        grid.last_observation_stamp = now
        ctl.tick(now, .05, ctl.command.copy(), ctl.command_v.copy())
        if ctl.continuous_passes > previous:
            pass_speeds.append(np.linalg.norm(ctl.command_v))
            previous = ctl.continuous_passes
        if ctl.land_requested or ctl.failure:
            break
    assert not ctl.failure
    assert len(pass_speeds) == 2 and min(pass_speeds) > .01
    assert ctl.waypoint_index == 3 and ctl.land_requested


def test_no_live_integrated_scan_cannot_complete_continuous_survey():
    c, grid = setup()
    ctl = SurveyController(c, grid, asynchronous=False)
    ctl.state = 'SURVEY'
    grid.last_observation_stamp = 0.
    for i in range(1000):
        now = i*.05
        ctl.health = Health(now, now, now, True)
        ctl.tick(now, .05, ctl.command.copy(), ctl.command_v.copy())
        if ctl.failure:
            break
    assert ctl.waypoint_index == 0 and ctl.failure == 'survey_waypoint_or_scan_missed'


def test_out_and_back_repeated_waypoint_uses_correct_later_knot():
    c, grid = setup()
    c['survey_waypoints'] = [[1., 0., 2.], [2., 0., 2.], [1., 0., 2.], [0., 0., 2.]]
    ctl = SurveyController(c, grid, asynchronous=False)
    ctl.state = 'SURVEY'
    for i in range(4000):
        now = i*.05
        ctl.health = Health(now, now, now, True)
        grid.last_observation_stamp = now
        ctl.tick(now, .05, ctl.command.copy(), ctl.command_v.copy())
        if ctl.land_requested or ctl.failure:
            break
    assert not ctl.failure
    assert ctl.waypoint_index == 4 and ctl.land_requested


def test_continuous_dropout_brakes_and_preserves_unknown():
    c, grid = setup()
    ctl = SurveyController(c, grid, asynchronous=False)
    ctl.state = 'SURVEY'
    ctl.health = Health(0., 0., 0., True)
    ctl.tick(0., .05, ctl.command, ctl.command_v)
    before = grid.state.copy()
    ctl.health = Health(-10., 1., 1., True)
    ctl.tick(1., .05, ctl.command, ctl.command_v)
    assert ctl.failure == 'stale_lidar' and not ctl.land_requested
    np.testing.assert_array_equal(grid.state, before)


def test_adaptive_tracking_and_clearance_reduce_speed_without_loosening_guards():
    c, grid = setup()
    ctl = SurveyController(c, grid, asynchronous=False)
    ctl.state = 'SURVEY'
    ctl._prepare_trajectory(0., ctl.command)
    grid.free_distance[:] = .06
    for _ in range(400):
        ctl._sample_next(.01, ctl.command+np.array([.07, 0, 0]), np.zeros(3))
    assert ctl.phase.rate < .6
    assert c['tracking_limit'] == .1


def test_async_continuous_result_keeps_batch_information():
    c, grid = setup()
    ctl = SurveyController(c, grid, asynchronous=True)
    try:
        ctl.state = 'SURVEY'
        ctl._prepare_trajectory(0., ctl.command)
        ctl.pending_plan.result(timeout=30.)
        assert ctl._prepare_trajectory(.05, ctl.command)
        assert ctl.batch_end_index == 2 and isinstance(ctl.trajectory, ContinuousTrajectory)
    finally:
        ctl.close()


def test_unknown_next_goal_does_not_extend_continuous_chunk():
    c, grid = setup()
    grid.state[tuple(grid.indices(c['survey_waypoints'][1]))] = UNKNOWN
    grid.rebuild(0.)
    ctl = SurveyController(c, grid, asynchronous=False)
    ctl.state = 'SURVEY'
    assert ctl._prepare_trajectory(0., ctl.command)
    assert ctl.batch_end_index == 0
    np.testing.assert_allclose(ctl.goal, c['survey_waypoints'][0])
    np.testing.assert_allclose(ctl.trajectory.sample(ctl.trajectory.duration)[1], 0., atol=1e-9)


def test_async_curve_rejected_when_live_map_changes():
    c, grid = setup()
    ctl = SurveyController(c, grid, asynchronous=True)
    try:
        ctl.state = 'SURVEY'
        ctl._prepare_trajectory(0., ctl.command)
        clone, ready = ctl.pending_plan.result(timeout=30.)
        assert ready
        hit = clone.trajectory.sample(clone.trajectory.duration*.5)[0]
        grid.state[tuple(grid.indices(hit))] = OCCUPIED
        grid.rebuild(.05)
        assert not ctl._prepare_trajectory(.05, ctl.command)
        assert ctl.trajectory is None and ctl.replans == 0
    finally:
        ctl.close()
