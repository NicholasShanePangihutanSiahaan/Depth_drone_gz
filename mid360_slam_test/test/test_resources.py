from pathlib import Path
import pytest
from mid360_slam_test.resources import read_temperatures


@pytest.mark.parametrize('failure', [TypeError("can't concat NoneType to bytes"),
    OSError('sensor unavailable'), ValueError('invalid sample'),
    UnicodeDecodeError('utf-8', b'\xff', 0, 1, 'invalid start byte')])
def test_unreadable_zone_does_not_hide_healthy_zone_or_crash(tmp_path, monkeypatch, failure):
    for name in ('thermal_zone0', 'thermal_zone1'):
        (tmp_path/name).mkdir()
        (tmp_path/name/'temp').write_text('42500')
    original = Path.read_text
    def read(path, *args, **kwargs):
        if path.parent.name == 'thermal_zone0':
            raise failure
        return original(path, *args, **kwargs)
    monkeypatch.setattr(Path, 'read_text', read)
    values, errors = read_temperatures(tmp_path)
    assert values == {'thermal_zone1': 42.5}
    assert errors['thermal_zone0']['error_type'] == type(failure).__name__


@pytest.mark.parametrize('text', ['', 'invalid', 'nan', 'inf'])
def test_invalid_temperature_reported_as_error_not_zero(tmp_path, text):
    (tmp_path/'thermal_zone0').mkdir()
    (tmp_path/'thermal_zone0'/'temp').write_text(text)
    values, errors = read_temperatures(tmp_path)
    assert values == {}
    assert 'thermal_zone0' in errors


def test_no_sensors_returns_no_measurements(tmp_path):
    assert read_temperatures(tmp_path) == ({}, {})
