import json
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock
import numpy as np
import pytest
from polinasi_nav.config import load_config
from polinasi_nav.pid_control import PositionPID
from polinasi_nav.mapping import VoxelMap, FREE


def fixture():
    c = load_config(Path(__file__).resolve().parents[1]/'config/pid_tour_check.json')
    c.update(bounds_min=[-3,-3,0], bounds_max=[3,3,5])
    grid = VoxelMap(c)
    grid.state[:] = FREE
    grid.seen[:] = 0.
    grid.rebuild(0.)
    ctl = SimpleNamespace(state='SURVEY', failure='', grid=grid,
        command=np.array([0.,0.,2.]), command_v=np.zeros(3), command_a=np.zeros(3),
        _braking=Mock(return_value=('brake','hold')))
    ctl.fail = lambda reason: setattr(ctl, 'failure', reason)
    return c, ctl, PositionPID(c)


def test_pid_hover_and_no_optimizer_or_model_required():
    c, ctl, pid = fixture()
    result = pid.apply(.05, ctl, ctl.command, np.zeros(3))
    np.testing.assert_allclose(result[0], ctl.command)
    np.testing.assert_allclose(ctl.command_a, 0.)
    assert not ctl.failure and pid.status()['active_controller'] == 'PID_PVA_feedback'
    assert not hasattr(pid, 'worker') and not pid.status()['model_controls_flight']
    # Live status publication must accept both saturated and unsaturated PID.
    json.dumps(pid.status(), allow_nan=False)


def test_pid_correction_direction_bounds_and_anti_windup():
    c, ctl, pid = fixture()
    previous = np.zeros(3)
    for _ in range(50):
        ctl.command_a = np.zeros(3)
        pid.apply(.05, ctl, ctl.command-[.1,0,0], [-.1,0,0])
        assert ctl.command_a[0] > 0.
        assert np.linalg.norm(ctl.command_a) <= c['acceleration']+1e-8
        assert np.linalg.norm(ctl.command_a-previous) <= c['jerk']*.05+1e-8
        previous = ctl.command_a.copy()
    assert not ctl.failure and pid.last['saturated']
    json.dumps(pid.status(), allow_nan=False)
    np.testing.assert_allclose(pid.integral, 0.)
    ctl.state = 'RETURN'
    ctl.command_a[:] = 0.
    pid.apply(.05, ctl, ctl.command, np.zeros(3))
    np.testing.assert_allclose(pid.integral, 0.)


def test_pid_integrates_small_error_without_saturation():
    c, ctl, pid = fixture()
    pid.apply(.05, ctl, ctl.command-[.001,0,0], np.zeros(3))
    assert pid.integral[0] > 0. and not ctl.failure


def test_unknown_final_corridor_brakes_not_fallback():
    c, ctl, pid = fixture()
    ctl.grid.blocked[:] = ctl.grid.blocked_actual[:] = True
    assert pid.apply(.05, ctl, ctl.command, np.zeros(3)) == ('brake','hold')
    assert ctl.failure == 'pid_final_safety' and not pid.last['active']


@pytest.mark.parametrize('dt,position', [(float('nan'), [0,0,2]), (.05, [float('nan'),0,2])])
def test_invalid_input_cannot_generate_pid_command(dt, position):
    c, ctl, pid = fixture()
    assert pid.apply(dt, ctl, position, np.zeros(3)) == ('brake','hold')
    assert ctl.failure == 'pid_invalid_measurement_or_dt'


def test_pid_presets_keep_route_and_safety_limits_and_are_exclusive(tmp_path):
    root = Path(__file__).resolve().parents[1]/'config'
    original, pid = load_config(root/'mapping_tour.json'), load_config(root/'pid_tour.json')
    assert pid['survey_waypoints'] == original['survey_waypoints']
    for name in ('command_speed','speed','acceleration','jerk','tracking_limit','clearance',
                 'cloud_timeout','pose_timeout','map_timeout'):
        assert pid[name] == original[name]
    pid['predictive_enabled'] = True
    path = tmp_path/'conflict.json'
    path.write_text(json.dumps(pid))
    with pytest.raises(ValueError, match='simultaneously'):
        load_config(path)
