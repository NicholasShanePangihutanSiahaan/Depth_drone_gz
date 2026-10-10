import numpy as np
import pytest
from types import SimpleNamespace

rclpy = pytest.importorskip('rclpy')
pytest.importorskip('mavros_msgs')
from rclpy.serialization import deserialize_message
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import MarkerArray
from nav_msgs.msg import Path
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from polinasi_nav.visual_worker import build_visual_snapshot


def test_visual_worker_serializes_frames_stamps_voxels_and_observed_paths():
    state = np.array([[[1]], [[0]], [[-1]]], dtype=np.int8)
    grid = SimpleNamespace(state=state, lo=np.zeros(3), res=.2)
    pose = PoseStamped()
    pose.header.frame_id = 'odom'
    pose.pose.position.x = 7.
    data = dict(stamp=(12, 34), grid=grid, trajectory=None,
                routes={'survey_pub': [[1., 2., 3.]]}, executed=[pose],
                global_points=[[4., 5., 6.]], resolution=.2,
                cloud_points=[[4., 5., 6.]])
    messages, duration = build_visual_snapshot(data)
    assert duration >= 0 and all(isinstance(m, bytes) for m in messages.values())
    markers = deserialize_message(messages['map_pub'], MarkerArray)
    assert [len(m.points) for m in markers.markers] == [1, 1, 1]
    assert [m.id for m in markers.markers] == [0, 1, 2]
    assert markers.markers[0].header.frame_id == 'map'
    assert markers.markers[0].header.stamp.sec == 12
    assert markers.markers[0].header.stamp.nanosec == 34
    route = deserialize_message(messages['survey_pub'], Path)
    assert route.poses[0].pose.position.z == 3.
    executed = deserialize_message(messages['executed_pub'], Path)
    assert executed.header.frame_id == 'odom' and executed.poses[0].pose.position.x == 7.
    cloud = deserialize_message(messages['global_cloud_pub'], PointCloud2)
    np.testing.assert_allclose(list(point_cloud2.read_points_list(cloud, field_names=('x', 'y', 'z'))), [[4, 5, 6]])
    np.testing.assert_array_equal(grid.state, state)  # Visualisation never modifies navigation cells.


@pytest.mark.parametrize('kind', ['local', 'global'])
def test_pending_visual_work_skips_new_snapshot_and_never_builds_backlog(kind):
    from concurrent.futures import Future
    from unittest.mock import Mock
    from polinasi_nav.ros_nodes import NavigationNode
    owner = SimpleNamespace(visual_worker=Mock(), visual_futures={kind: Future()}, history=object())
    function = NavigationNode.visualise if kind == 'local' else NavigationNode.global_visualise
    function(owner)  # No grid/history access needed while the existing job runs.
    owner.visual_worker.submit.assert_not_called()
