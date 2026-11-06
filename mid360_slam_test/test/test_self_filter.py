import json
import numpy as np
import pytest
from scipy.spatial.transform import Rotation
from mid360_slam_test.self_filter import BodySphere


def test_level_mount_cm_to_m():
    f=BodySphere(a_cm=20.,b_cm=10.,radius_cm=30.,enabled=True)
    assert f.center==pytest.approx([-.1,0,-.2])
    assert f.radius_m==.3


def test_operator_hardware_geometry():
    from pathlib import Path
    import yaml
    config = yaml.safe_load((Path(__file__).parents[1]/'config/hardware.yaml').read_text())['/**']['ros__parameters']
    f = BodySphere(pitch_down_deg=config['lidar_mount_pitch_down_deg'],
                   radius_cm=config['body_sphere_radius_cm'],
                   enabled=config['body_sphere_filter_enabled'],
                   center_mode=config['body_sphere_center_mode'],
                   center_lidar_cm=config['body_sphere_center_lidar_cm'])
    assert f.enabled
    assert f.center == pytest.approx([0., 0., -.16])
    assert f.radius_m == pytest.approx(.44)
    assert f.inside([f.center, f.center+[0., .439, 0.],
                     f.center+[0., .441, 0.]]).tolist() == [True, True, False]


@pytest.mark.parametrize('pitch', [0., 20., -30.])
def test_native_center_is_not_rotated(pitch):
    f = BodySphere(pitch, radius_cm=44., enabled=True,
                   center_mode='lidar_coordinates', center_lidar_cm=[0., 0., -16.])
    assert f.center == pytest.approx([0., 0., -.16])
    assert f.metadata()['rotation'].startswith('none')


@pytest.mark.parametrize('kwargs', [dict(center_mode='typo'),
    dict(center_mode='lidar_coordinates', center_lidar_cm=[0., 0.]),
    dict(center_mode='lidar_coordinates', center_lidar_cm=[0., 0., float('nan')])])
def test_invalid_native_center_refused(kwargs):
    with pytest.raises(ValueError): BodySphere(**kwargs)


@pytest.mark.parametrize('degrees',[-45.,0.,10.,20.,30.,68.,90.])
def test_rotated_centre_matches_inverse_mount_rotation(degrees):
    f=BodySphere(degrees,20.,10.,30.,True)
    expected=Rotation.from_euler('y',-degrees,degrees=True).apply([-.1,0,-.2])
    assert f.center==pytest.approx(expected)
    # In body coordinates it must still be behind/below the LiDAR origin.
    assert Rotation.from_euler('y',degrees,degrees=True).apply(f.center)==pytest.approx([-.1,0,-.2])


def test_pitch_down_moves_downward_offset_toward_positive_x():
    f=BodySphere(30.,20.,0.,10.,True)
    assert f.center[0]>0 and f.center[2]<0


def test_boundary_inside_outside_invalid_and_nonzero_offset():
    f=BodySphere(0.,50.,100.,25.,True)
    points=f.center+np.array([[0,0,0],[.25,0,0],[.250001,0,0],[-.249999,0,0]])
    points=np.vstack([points,[0,0,0],[np.nan,0,0],[np.inf,0,0]])
    assert f.inside(points).tolist()==[True,True,False,True,False,False,False]
    assert f.inside([]).shape==(0,)
    json.dumps(f.metadata(),allow_nan=False)


def test_disabled_placeholder_does_not_filter_any_points():
    f=BodySphere()
    assert not f.inside([[0,0,0],[1,2,3]]).any()
    assert not f.metadata()['enabled']


@pytest.mark.parametrize('kwargs',[dict(pitch_down_deg=float('nan')),
    dict(a_cm=-1),dict(b_cm=-1),dict(radius_cm=-1),dict(pitch_down_deg=91),
    dict(radius_cm=float('inf')),dict(enabled=True,radius_cm=0),
    dict(a_cm=True),dict(enabled='true'),dict(pitch_down_deg='10')])
def test_invalid_parameters_refused(kwargs):
    with pytest.raises(ValueError):BodySphere(**kwargs)
