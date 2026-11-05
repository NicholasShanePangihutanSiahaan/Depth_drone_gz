import json
import math
import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from mid360_test.core.sectors import measure
from mid360_test.core.integration import ImuIntegrator
from mid360_test.core.health import StreamHealth
from mid360_test.core.recording import ReadingFiles


def test_axes_wrap_invalid_returns_and_elevation():
    p = [[1, 0, 0], [2, 0, 0], [0, -3, 0], [0, 4, 0], [-5, .01, 0],
         [0, 0, 0], [np.nan, 0, 0], [.1, 0, 0], [1, 0, 2]]
    result = measure(p)
    assert result['missing'] == []
    assert [result['sectors'][k]['distance_cm'] for k in ('front', 'right', 'left')] == [100, 300, 400]
    assert result['sectors']['back']['distance_cm'] == pytest.approx(math.hypot(5, .01)*100)
    assert result['zero_returns'] == 1 and result['nonfinite_points'] == 1
    assert result['sectors']['front']['points'] == 2


def test_empty_and_tag_filter_never_invent_zero_or_free():
    assert len(measure([])['missing']) == 4
    result = measure([[1, 0, 0], [2, 0, 0]], tags=[16, 0], reject_tag_mask=16)
    assert result['sectors']['front']['distance_cm'] == 200
    with pytest.raises(ValueError):
        measure([[1, 0, 0]], reject_tag_mask=16)


@pytest.mark.parametrize('kwargs', [dict(half_width=0), dict(half_width=60),
    dict(minimum=0), dict(elevation_min=20, elevation_max=10), dict(maximum=float('nan'))])
def test_bad_sector_configuration_rejected(kwargs):
    with pytest.raises(ValueError):
        measure([], **kwargs)


def calibrated(unit='m_s2', attitude=None):
    integrator = ImuIntegrator(unit, calibration_seconds=.2, min_samples=10)
    attitude = Rotation.identity() if attitude is None else attitude
    gravity = attitude.inv().apply([0, 0, 9.80665])
    acc = gravity/9.80665 if unit == 'g' else gravity
    for i in range(30):
        result = integrator.update(1+i*.01, [.001, -.002, .003], acc)
    assert result['valid']
    return integrator


def test_stationary_bias_calibration_and_no_gravity_velocity():
    integrator = calibrated()
    for i in range(100):
        result = integrator.update(1.30+i*.01, [.001, -.002, .003], [0, 0, 9.80665])
    assert result['speed_mps'] < 1e-9
    assert result['rpy_deg'] == pytest.approx([0, 0, 0], abs=1e-9)


def test_tilted_calibration_and_g_units():
    tilt = Rotation.from_euler('xy', [20, -15], degrees=True)
    integrator = calibrated('g', tilt)
    acc = tilt.inv().apply([0, 0, 1.])
    for i in range(100):
        result = integrator.update(1.30+i*.01, [.001, -.002, .003], acc)
    assert result['speed_mps'] < 1e-8


def test_yaw_rotation_and_constant_acceleration():
    integrator = calibrated()
    for i in range(100):
        result = integrator.update(1.30+i*.01, [.001, -.002, .003+math.pi/2], [0, 0, 9.80665])
    assert result['rpy_deg'][2] == pytest.approx(89.55, abs=.01)
    assert result['speed_mps'] < 1e-8
    integrator = calibrated()
    for i in range(100):
        result = integrator.update(1.30+i*.01, [.001, -.002, .003], [1, 0, 9.80665])
    assert result['velocity_mps'] == pytest.approx([.995, 0, 0], abs=1e-8)


@pytest.mark.parametrize('stamp,gyro,acc', [(1.29, [0, 0, 0], [0, 0, 9.8]),
    (2., [0, 0, 0], [0, 0, 9.8]), (1.30, [np.nan, 0, 0], [0, 0, 9.8])])
def test_invalid_imu_latches_and_does_not_resume(stamp, gyro, acc):
    integrator = calibrated()
    assert not integrator.update(stamp, gyro, acc)['valid']
    assert not integrator.update(2.01, [0, 0, 0], [0, 0, 9.80665])['valid']


def test_wrong_acceleration_unit_and_moving_start_do_not_calibrate():
    integrator = ImuIntegrator('m_s2', calibration_seconds=.2, min_samples=10)
    for i in range(100):
        result = integrator.update(1+i*.01, [0, 0, 0], [0, 0, 1])
    assert result['state'] == 'CALIBRATING' and not result['valid']


def test_missing_stream_not_pass_and_timeout_invalid_order():
    health = StreamHealth(.5, 10.)
    assert health.report(1)['status'] == 'NOT TESTED'
    health.receive(1, 10.)
    health.receive(1.3, 10.3)
    report = health.report(1.31)
    assert report['receive_hz'] == pytest.approx(1/.3)
    assert report['estimated_missing_messages'] == 2
    assert health.report(2)['status'] == 'FAIL'
    health.receive(2.1, 10.2)
    assert health.report(2.1)['status'] == 'FAIL'


def test_files_are_strict_json_compatible_and_missing_is_na(tmp_path):
    cloud = dict(measure([[1, 0, 0]]), stamp=3.)
    files = ReadingFiles(tmp_path)
    line = files.write(1., cloud, dict(valid=False, state='CALIBRATING'), 'MISSING_RETURNS')
    files.close()
    assert 'front = 100.0 cm' in line and 'back = N/A' in line
    assert 'Yaw_est = N/A' in line and 'Missing = right,left,back' in line
    assert len((tmp_path/'readings.csv').read_text().splitlines()) == 2
    json.dumps(cloud, allow_nan=False)
    with pytest.raises(FileExistsError):
        ReadingFiles(tmp_path)
