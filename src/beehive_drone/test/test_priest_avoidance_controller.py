"""Unit tests for the PRIEST-to-field-stack adapter helpers."""

from types import SimpleNamespace

from geometry_msgs.msg import Pose

from beehive_drone.priest_avoidance_controller import (
    cylinder_to_ellipsoid,
    select_lookahead_pose,
)


def pose(x, y, z=0.0):
    message = Pose()
    message.position.x = float(x)
    message.position.y = float(y)
    message.position.z = float(z)
    return message


def test_selects_first_pose_beyond_lookahead():
    path = SimpleNamespace(poses=[
        SimpleNamespace(pose=pose(0.1, 0.0)),
        SimpleNamespace(pose=pose(0.7, 0.0)),
        SimpleNamespace(pose=pose(1.2, 0.0)),
    ])

    selected = select_lookahead_pose(path, pose(0.0, 0.0), 0.6)

    assert selected.position.x == 0.7


def test_uses_last_pose_when_path_is_shorter_than_lookahead():
    path = SimpleNamespace(poses=[
        SimpleNamespace(pose=pose(0.1, 0.0)),
        SimpleNamespace(pose=pose(0.2, 0.0)),
    ])

    selected = select_lookahead_pose(path, pose(0.0, 0.0), 0.6)

    assert selected.position.x == 0.2


def test_valid_cylinder_becomes_ellipsoid_with_minimum_sizes():
    cylinder = SimpleNamespace(
        is_valid=True,
        radius=0.05,
        height=0.20,
        pose=pose(1.0, 2.0, 3.0),
    )
    tracked = SimpleNamespace(cylinder=cylinder)

    obstacle = cylinder_to_ellipsoid(tracked, 0.10, 0.25)

    assert obstacle.center.x == 1.0
    assert obstacle.radii.x == 0.10
    assert obstacle.radii.y == 0.10
    assert obstacle.radii.z == 0.25


def test_invalid_cylinder_is_ignored():
    tracked = SimpleNamespace(cylinder=SimpleNamespace(is_valid=False))

    assert cylinder_to_ellipsoid(tracked, 0.10, 0.25) is None
