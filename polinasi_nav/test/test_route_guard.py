import json
import numpy as np
from unittest.mock import patch
from polinasi_nav.config import load_config
from polinasi_nav.control import MissionController
from polinasi_nav.mapping import VoxelMap, FREE, UNKNOWN, OCCUPIED
from polinasi_nav.exploration import choose_view
from polinasi_nav.planning import plan, Health
from polinasi_nav.survey import SurveyController
from polinasi_nav.continuous import ContinuousTrajectory
from polinasi_nav.map_log import MapLog


def fixture():
    c = load_config()
    c.update(bounds_min=[-3., -3., 0.], bounds_max=[6., 3., 5.],
        resolution=.2, drone_dimensions=[.2, .2, .2], clearance=.05,
        tracking_margin=.05, tracking_limit=.05, home=[.1, .1, 2.1],
        survey_waypoints=[[3.1, .1, 2.1]], exploration_route_guard=True,
        exploration_map_wait_seconds=.8, exploration_map_wait_scans=2,
        adaptive_directional_clearance=True)
    g = VoxelMap(c)
    g.state[:] = FREE
    g.seen[:] = 0.
    g.rebuild(0.)
    g.last_observation_stamp = 0.
    ctl = SurveyController(c, g, asynchronous=False)
    ctl.state = 'SURVEY'
    ctl.command = np.asarray(c['home'])
    return c, g, ctl


def test_corridor_only_reports_body_relevant_unknown_and_is_non_mutating():
    c, g, ctl = fixture()
    g.state[tuple(g.indices([1., 2.5, 2.]))] = UNKNOWN
    g.state[tuple(g.indices([1., .1, 2.]))] = OCCUPIED
    before = g.state.copy()
    d = g.corridor_blockers(ctl.command, c['survey_waypoints'][0])
    assert len(d['unknown']) == 0 and len(d['occupied']) == 1
    np.testing.assert_array_equal(g.state, before)


def test_guard_waits_for_two_actual_map_updates_before_searching_views():
    c, g, ctl = fixture()
    g.state[g.indices([2., 0., 2.])[0]:] = UNKNOWN
    g.rebuild(0.)
    with patch.object(ctl, '_begin_exploration', return_value=None) as search:
        ctl._guarded_exploration(0., ctl.command)
        ctl._guarded_exploration(1., ctl.command)
        assert search.call_count == 0  # Time passing is not a new scan.
        g.last_observation_stamp = 1.1
        ctl._guarded_exploration(1.1, ctl.command)
        assert search.call_count == 0
        g.last_observation_stamp = 1.2
        ctl._guarded_exploration(1.2, ctl.command)
        assert search.call_count == 1
        assert len(search.call_args.kwargs['relevant_unknown']) > 0


def test_new_map_can_remove_need_to_explore():
    c, g, ctl = fixture()
    g.state[g.indices([2., 0., 2.])[0]:] = UNKNOWN
    g.rebuild(0.)
    ctl.health = Health(0., 0., 0., True)
    ctl.tick(0., .05, ctl.command, np.zeros(3))
    assert ctl.state == 'SURVEY' and ctl.route_blockage['reason'] == 'waiting_for_map'
    g.state[:] = FREE
    g.rebuild(1.)
    g.last_observation_stamp = 1.
    ctl.health = Health(1., 1., 1., True)
    ctl.tick(1., .05, ctl.command, np.zeros(3))
    assert ctl.state == 'SURVEY' and ctl.trajectory is not None
    assert ctl.exploration_views == 0


def test_occupied_only_blockage_does_not_request_exploration():
    c, g, ctl = fixture()
    g.state[g.indices([2., 0., 2.])[0], :, :] = OCCUPIED
    g.rebuild(0.)
    with patch.object(ctl, '_begin_exploration') as search:
        ctl._guarded_exploration(0., ctl.command)
        for t in (.5, 1.):
            g.last_observation_stamp = t
            ctl._guarded_exploration(t, ctl.command)
        assert search.call_count == 0
        assert ctl.route_blockage['reason'] == 'occupied_blocked'


def test_exploration_rejects_gain_unrelated_to_route_blockers():
    c, g, ctl = fixture()
    g.state[g.indices([2., 0., 2.])[0]:] = UNKNOWN
    g.rebuild(0.)
    assert choose_view(g, ctl.command, [3., 0., 2.]) is not None
    assert choose_view(g, ctl.command, [3., 0., 2.], relevant_unknown=np.empty((0, 3), int)) is None


def test_planner_search_limit_is_not_mislabeled_unknown():
    c, g, ctl = fixture()
    g.state[tuple(g.indices([1.5, .1, 2.1]))] = OCCUPIED
    g.rebuild(0.)
    d = {}
    assert plan(g, ctl.command, c['survey_waypoints'][0], 1, d) is None
    assert d['reason'] == 'search_limit'


def test_directional_speed_does_not_penalize_safe_side_unknown():
    c, g, ctl = fixture()
    # Nearby unknown outside the inflated body's straight stopping corridor.
    g.state[:, g.indices([0., .9, 2.])[1]:, :] = UNKNOWN
    g.rebuild(0.)
    ctl.trajectory = ContinuousTrajectory([ctl.command, [3.1, .1, 2.1]], c)
    ctl.elapsed = ctl.trajectory.duration*.4
    p, v, _ = ctl.trajectory.sample(ctl.elapsed)
    ctl.command = p.copy()
    ctl._sample_next(.05, p, v)
    d = ctl.speed_diagnostics
    assert d['legacy_space_scale'] < .8
    assert d['braking_scale'] == d['target_scale'] == 1.
    assert not ctl.failure


def test_directional_braking_rejects_unknown_in_front():
    c, g, ctl = fixture()
    ctl.trajectory = ContinuousTrajectory([ctl.command, [3.1, .1, 2.1]], c)
    ctl.elapsed = ctl.trajectory.duration*.4
    p, v, a = ctl.trajectory.sample(ctl.elapsed)
    # Unknown intersects the actual body, not a side-only boundary.
    g.state[tuple(g.indices(p))] = UNKNOWN
    g.rebuild(0.)
    ctl.command = p.copy()
    ctl._sample_next(.05, p, v)
    assert ctl.failure == 'no_directional_stopping_corridor'


def test_debug_trace_is_throttled_but_preserves_decision_changes(tmp_path):
    log = MapLog(tmp_path)
    d = dict(simulation_time=0., mission='SURVEY', failure='', route_blockage={})
    log.record_debug(d)
    log.record_debug(dict(d, simulation_time=.1))
    log.record_debug(dict(d, simulation_time=.2, mission='EXPLORE'))
    rows = [json.loads(row) for row in (tmp_path/'navigation_debug.jsonl').read_text().splitlines()]
    assert len(rows) == 2 and rows[-1]['mission'] == 'EXPLORE'
