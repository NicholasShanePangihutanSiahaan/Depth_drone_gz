from collections import deque
from types import SimpleNamespace
from unittest.mock import Mock
import numpy as np
import pytest

pytest.importorskip('rclpy')
pytest.importorskip('mavros_msgs')
from nav_msgs.msg import Odometry
from polinasi_nav.mapping_io import MappingIONode


def test_mapping_odometry_does_not_require_flight_controller_map_trust():
    owner=SimpleNamespace(controller=SimpleNamespace(health=SimpleNamespace(pose_stamp=-np.inf)),
        callback_timing=Mock(),map_log=None,executed=deque())
    msg=Odometry();msg.header.frame_id='odom';msg.child_frame_id='base_link'
    msg.header.stamp.sec=1;msg.pose.pose.orientation.w=1.
    msg.pose.pose.position.z=2.;msg.twist.twist.linear.x=.2
    MappingIONode.odom(owner,msg)
    assert owner.controller.health.pose_stamp==1.
    np.testing.assert_allclose(owner.pose,[0.,0.,2.])
    np.testing.assert_allclose(owner.velocity,[.2,0.,0.])
    assert len(owner.executed)==1
    assert not hasattr(owner.controller,'map_trust')
