import pytest

pytest.importorskip('rclpy')
from rclpy.qos import qos_profile_sensor_data
from polinasi_nav.ros_nodes import localisation_imu_qos


def test_imu_backlog_does_not_modify_global_sensor_profile_or_reliability():
    original_depth=qos_profile_sensor_data.depth
    profile=localisation_imu_qos()
    assert profile.depth==256
    assert profile.reliability==qos_profile_sensor_data.reliability
    assert profile.history==qos_profile_sensor_data.history
    assert profile.durability==qos_profile_sensor_data.durability
    assert qos_profile_sensor_data.depth==original_depth
    profile.depth=1
    assert localisation_imu_qos().depth==256
