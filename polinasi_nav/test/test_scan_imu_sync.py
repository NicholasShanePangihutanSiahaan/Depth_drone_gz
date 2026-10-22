from collections import deque
from types import SimpleNamespace
from unittest.mock import Mock
import pytest

pytest.importorskip('rclpy')
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import TransformStamped
from polinasi_nav.ros_nodes import LocalisationNode


def test_scan_waits_for_measurement_time_imu_without_restamping_or_consuming_cloud():
    msg=PointCloud2();msg.header.stamp.sec=2
    transform=TransformStamped();transform.transform.rotation.w=1.
    buffer=Mock();buffer.lookup_transform.return_value=transform
    owner=SimpleNamespace(pending_scan=deque([msg],maxlen=1),valid_time=lambda _:True,
        last_imu=1.955,mount_buffer=buffer,mount_rejection='',estimator=SimpleNamespace(),
        localisation_fcu_state=None,scan=Mock())
    LocalisationNode.rolling_scan(owner)
    assert owner.mount_rejection=='waiting_for_scan_time_imu'
    assert len(owner.pending_scan)==1
    owner.scan.assert_not_called();buffer.lookup_transform.assert_not_called()
    owner.last_imu=2.
    LocalisationNode.rolling_scan(owner)
    owner.scan.assert_called_once_with(msg)
    assert not owner.pending_scan and owner.mount_rejection==''
    assert msg.header.stamp.sec==2


def test_new_cloud_arriving_during_TF_lookup_is_retained_for_next_scan():
    old=PointCloud2();old.header.stamp.sec=2
    newer=PointCloud2();newer.header.stamp.sec=2;newer.header.stamp.nanosec=100000000
    transform=TransformStamped();transform.transform.rotation.w=1.
    owner=SimpleNamespace(pending_scan=deque([old],maxlen=1),valid_time=lambda _:True,
        last_imu=2.1,mount_buffer=Mock(),mount_rejection='',estimator=SimpleNamespace(),
        localisation_fcu_state=None,scan=Mock())
    def lookup(*args):owner.pending_scan.append(newer);return transform
    owner.mount_buffer.lookup_transform.side_effect=lookup
    LocalisationNode.rolling_scan(owner)
    owner.scan.assert_called_once_with(old)
    assert list(owner.pending_scan)==[newer]
