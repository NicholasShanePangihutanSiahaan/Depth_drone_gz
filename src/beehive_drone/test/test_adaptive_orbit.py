import math
import pytest
from beehive_drone.adaptive_orbit import orbit_target_z

@pytest.mark.parametrize('height,expected', [(4,2),(7,3.5)])
def test_different_trunk_heights(height,expected):
    assert orbit_target_z(0,height)==expected

def test_hill_base_and_limits():
    assert orbit_target_z(5,7)==8.5
    assert orbit_target_z(5,0.8)==6
    assert orbit_target_z(5,20)==9

@pytest.mark.parametrize('height', [0,-1,math.nan,math.inf])
def test_invalid_height_rejected(height):
    with pytest.raises(ValueError): orbit_target_z(0,height)
