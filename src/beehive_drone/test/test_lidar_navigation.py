from types import SimpleNamespace
import numpy as np
import pytest
from beehive_drone.lidar_navigation import LidarView, decode_xyz


def test_decode_padding_endianness_and_invalid_buffer():
    raw = np.zeros((2, 4), dtype='>f4')
    raw[:, :3] = [[1, 2, 3], [4, 5, 6]]
    fields = [SimpleNamespace(name=n, offset=i*4, datatype=7) for i, n in enumerate('xyz')]
    msg = SimpleNamespace(fields=fields, width=1, height=2, point_step=12,
                          row_step=16, is_bigendian=True, data=raw.tobytes())
    assert np.allclose(decode_xyz(msg), [[1, 2, 3], [4, 5, 6]])
    msg.data = b'bad'
    with pytest.raises(ValueError): decode_xyz(msg)


def test_missing_returns_cannot_certify_corridor():
    view = LidarView(np.array([[np.inf, 0, 0], [np.nan, 0, 0]]), np.zeros(3), np.eye(3))
    assert not view.corridor_seen(np.zeros(3), np.array([1., 0., 0.]), .4, .3)


def test_close_thin_return_blocks_angular_cell():
    az, el = np.meshgrid(np.radians(np.arange(-15., 15., .5)),
                         np.radians(np.arange(-10., 10., .5)))
    xyz = np.column_stack((np.cos(el.ravel())*np.cos(az.ravel()),
                           np.cos(el.ravel())*np.sin(az.ravel()), np.sin(el.ravel())))
    points = xyz*8.
    clear = LidarView(points, np.zeros(3), np.eye(3))
    assert clear.corridor_seen(np.zeros(3), np.array([1., 0., 0.]), .4, .3)
    blocked = LidarView(np.vstack((points, [1., 0., 0.])), np.zeros(3), np.eye(3))
    assert not blocked.corridor_seen(np.zeros(3), np.array([1., 0., 0.]), .4, .3)
