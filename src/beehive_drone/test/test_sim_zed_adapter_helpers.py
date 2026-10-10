import pytest

from beehive_drone.sim_zed_adapter import (
    discover_tree_positions_from_sdf,
    limit_source_trees,
    parse_tree_positions,
)
from beehive_drone.missions.virtual_tree_test import homeward_point


def test_empty_tree_list_preserves_legacy_single_tree():
    assert parse_tree_positions('', (7.0, 0.0, 0.35)) == [(7.0, 0.0, 0.35)]


def test_virtual_tree_is_six_metres_from_real_toward_home():
    x, y = homeward_point((8.0, 0.0), (0.0, 0.0), 6.0)

    assert x == 2.0
    assert y == 0.0


def test_source_tree_limit_keeps_first_sdf_tree_deterministically():
    trees = [('tree_01', (7.0, 0.0, 0.0)),
             ('tree_02', (14.0, 0.0, 0.0))]

    assert limit_source_trees(trees, 1) == trees[:1]
    assert limit_source_trees(trees, 0) == trees


def test_multi_tree_positions_inherit_or_override_ground_height():
    trees = parse_tree_positions('7,-3;7,3,0.8', (7.0, 0.0, 0.35))
    assert trees == [(7.0, -3.0, 0.35), (7.0, 3.0, 0.8)]


def test_tree_positions_are_discovered_and_sorted_from_sdf(tmp_path):
    world = tmp_path / 'plantation.sdf'
    world.write_text(
        '<sdf><world>'
        '<include><name>tree_02</name><pose>14 0 0 0 0 0</pose></include>'
        '<include><name>house</name><pose>1 2 0 0 0 0</pose></include>'
        '<include><name>tree_01</name><pose>7 -3 0.35 0 0 0</pose></include>'
        '</world></sdf>', encoding='utf-8')

    assert discover_tree_positions_from_sdf(str(world)) == [
        ('tree_01', (7.0, -3.0, 0.35)),
        ('tree_02', (14.0, 0.0, 0.0)),
    ]


def test_sdf_discovery_rejects_world_without_trees(tmp_path):
    world = tmp_path / 'empty.sdf'
    world.write_text('<sdf><world/></sdf>', encoding='utf-8')
    with pytest.raises(ValueError, match='no entities'):
        discover_tree_positions_from_sdf(str(world))
