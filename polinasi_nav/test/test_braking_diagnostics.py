"""Failure witnesses explain inflation, not just the centreline's raw cells."""
import json
import numpy as np
import pytest
from polinasi_nav.config import load_config
from polinasi_nav.mapping import VoxelMap, FREE, UNKNOWN, OCCUPIED
from polinasi_nav.planning import brake_safe, Brake
from polinasi_nav.continuous import ContinuousTrajectory
from polinasi_nav.map_log import MapLog
from test_route_guard import fixture


def arena():
    c = load_config()
    c.update(bounds_min=[-3., -3., 0.], bounds_max=[3., 3., 5.],
             resolution=.2, drone_dimensions=[.2, .2, .2],
             clearance=.05, tracking_margin=.05)
    g = VoxelMap(c)
    g.state[:] = FREE
    g.seen[:] = 0.
    g.last_observation_stamp = 0.
    g.rebuild(0.)
    return c, g


@pytest.mark.parametrize('state,cause', [(UNKNOWN, 'unknown'), (OCCUPIED, 'occupied')])
def test_reaction_failure_reports_raw_inflation_cause_without_mutation(state, cause):
    c, g = arena()
    p = np.array([.1, .1, 2.1])
    obstacle = [.1, .5, 2.1]
    g.state[tuple(g.indices(obstacle))] = state
    g.rebuild(0.)
    before = g.state.copy()
    detail = {}
    assert not brake_safe(g, p, [.1, 0., 0.], c, diagnostics=detail)
    assert detail['reason'] == 'reaction_segment_blocked'
    assert cause in detail['blockage']['causes']
    assert detail['blockage'][cause+'_samples']
    assert g.state[tuple(g.indices(p))] == FREE
    assert detail['blockage']['footprint_'+cause+'_cells'] == 1
    np.testing.assert_array_equal(g.state, before)
    json.dumps(detail, allow_nan=False)


def test_braking_hull_reports_neighbor_obstacle_even_if_raw_box_is_free():
    c, g = arena()
    p = np.array([.1, .1, 2.1])
    g.state[tuple(g.indices([.1, .5, 2.1]))] = OCCUPIED
    g.rebuild(0.)
    detail = {}
    assert not Brake(p, [.1, 0., 0.], np.zeros(3), c).path_safe(g, diagnostics=detail)
    assert detail['raw_occupied_cells'] == 0
    assert detail['blockage']['footprint_occupied_cells'] == 1
    assert detail['interval_seconds'][0] == 0.
    assert 'not_confirmed_curve_collision' in detail['witness_policy']


def test_outside_window_and_stationary_have_witnesses():
    c, g = arena()
    detail = {}
    assert not brake_safe(g, [3.1, .1, 2.1], np.zeros(3), c, diagnostics=detail)
    assert detail['reason'] == 'stationary_blocked'
    assert 'outside_window' in detail['blockage']['causes']
    detail = {}
    assert not Brake(np.array([3.1, .1, 2.1]), [.1, 0., 0.], np.zeros(3), c).path_safe(g, diagnostics=detail)
    assert detail['path_reason'] == 'braking_outside_window'
    assert 'outside_window' in detail['blockage']['causes']


def test_survey_failure_overwrites_previous_success_and_records_every_factor(tmp_path):
    c, g, ctl = fixture()
    ctl.trajectory = ContinuousTrajectory([ctl.command, [2., .1, 2.1]], c)
    ctl.elapsed = ctl.trajectory.duration*.4
    p, v, a = ctl.trajectory.sample(ctl.elapsed)
    ctl.command = p.copy()
    ctl.speed_diagnostics = dict(stale_previous_tick=True)
    g.state[tuple(g.indices(p))] = UNKNOWN
    g.rebuild(0.)
    ctl._sample_next(.05, p, v)
    assert ctl.failure == 'no_directional_stopping_corridor'
    assert 'stale_previous_tick' not in ctl.speed_diagnostics
    assert ctl.speed_diagnostics['braking_scale'] == 0.
    checks = ctl.speed_diagnostics['stopping_checks']
    assert len(checks) == 4 and all(not d['safe'] for d in checks)
    assert all('unknown' in d['blockage']['causes'] for d in checks)
    json.dumps(ctl.speed_diagnostics, allow_nan=False)
    log = MapLog(tmp_path)
    log.record_debug(dict(simulation_time=1., mission=ctl.state, failure=ctl.failure,
                          speed_diagnostics=ctl.speed_diagnostics))
    saved = json.loads((tmp_path/'navigation_debug.jsonl').read_text())
    assert saved['speed_diagnostics']['stopping_checks'] == checks
