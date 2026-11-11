import numpy as np
import pytest
from beehive_drone.lidar_trunk_geometry import fit_trunk


def trunk(height=4):
    z=np.repeat(np.linspace(.1,height,32),24)
    angle=np.tile(np.linspace(-1.4,1.4,24),32)
    return np.column_stack([3+.3*np.cos(angle),2+.3*np.sin(angle),z])

@pytest.mark.parametrize('height',[4,7])
def test_partial_arc_trunk_corrects_camera_center_and_estimates_visible_height(height):
    result=fit_trunk(trunk(height),(3.2,2.1,1))
    assert result is not None
    assert result[:2]==pytest.approx([3,2],abs=.03)
    assert result[3]==pytest.approx(height-.1,rel=.1)

def test_wrong_camera_candidate_not_associated():
    assert fit_trunk(trunk(),(0,0,1)) is None

def test_wall_is_not_a_trunk():
    y,z=np.meshgrid(np.linspace(1.5,2.5,35),np.linspace(.1,4,32))
    points=np.column_stack([np.full(y.size,3),y.ravel(),z.ravel()])
    assert fit_trunk(points,(3,2,1)) is None

def test_one_horizontal_ring_is_not_a_vertical_trunk():
    points=trunk();points[:,2]=1
    assert fit_trunk(points,(3,2,1)) is None
