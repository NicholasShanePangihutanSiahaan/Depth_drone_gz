import math

from beehive_drone.sim_zed_adapter import (
    quaternion_from_rpy,
    rotate_vector,
    world_to_camera,
)


def assert_vector_close(actual, expected, tolerance=1.0e-6):
    for actual_value, expected_value in zip(actual, expected):
        assert abs(actual_value - expected_value) <= tolerance


def test_world_to_camera_applies_mount_translation():
    point = world_to_camera(
        (7.0, 0.0, 3.0),
        (1.0, 0.0, 1.0),
        (0.0, 0.0, 0.0, 1.0),
        (0.14, 0.06, 0.02),
        (0.0, 0.0, 0.0, 1.0),
    )
    assert_vector_close(point, (5.86, -0.06, 1.98))


def test_world_to_camera_cancels_vehicle_yaw():
    yaw_90 = quaternion_from_rpy(0.0, 0.0, math.pi * 0.5)
    point = world_to_camera(
        (0.0, 7.0, 1.0),
        (0.0, 0.0, 1.0),
        yaw_90,
        (0.0, 0.0, 0.0),
        (0.0, 0.0, 0.0, 1.0),
    )
    assert_vector_close(point, (7.0, 0.0, 0.0))


def test_camera_mount_yaw_is_reversible():
    camera_yaw = quaternion_from_rpy(0.0, 0.0, math.radians(30.0))
    point_camera = world_to_camera(
        (5.0, 0.0, 0.0),
        (0.0, 0.0, 0.0),
        (0.0, 0.0, 0.0, 1.0),
        (0.0, 0.0, 0.0),
        camera_yaw,
    )
    point_base = rotate_vector(camera_yaw, point_camera)
    assert_vector_close(point_base, (5.0, 0.0, 0.0))
