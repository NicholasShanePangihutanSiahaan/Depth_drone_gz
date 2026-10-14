import subprocess
from pathlib import Path

ROOT=Path(__file__).resolve().parents[2]


def test_flower_world_cannot_launch_without_completed_old_baseline():
    result=subprocess.run(['bash','tools/run_sensor_localisation_validation.sh',
        'inspection','palm_farm_flowers','/tmp/nonexistent_polinasi_baseline_summary.json'],
        cwd=ROOT,capture_output=True,text=True,timeout=10)
    assert result.returncode==2 and 'baseline summary is required' in result.stdout
    assert 'Validation logs:' not in result.stdout


def test_unrecognised_world_never_starts_simulator():
    result=subprocess.run(['bash','tools/run_sensor_localisation_validation.sh',
        'inspection','not_a_world'],cwd=ROOT,capture_output=True,text=True,timeout=10)
    assert result.returncode==2 and 'Validation logs:' not in result.stdout
