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
