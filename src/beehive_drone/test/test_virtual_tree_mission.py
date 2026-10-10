"""Tests for the dedicated real-plus-virtual mission profile."""

from types import SimpleNamespace

from beehive_drone.missions.virtual_tree_test import VirtualTreeTestMission


def real_tree():
    return SimpleNamespace(
        id=7, x=8.0, y=0.0, z=3.12, confidence=0.9,
        inspected=True, validated=True, orbit_count=1)


def test_profile_creates_second_target_six_metres_toward_home():
    profile = VirtualTreeTestMission(6.0, 9001)

    target = profile.tree_completed(real_tree(), (0.0, 0.0, 0.0))

    assert target.id == 9001
    assert target.x == 2.0
    assert target.y == 0.0
    assert profile.is_virtual(target)


def test_profile_creates_virtual_target_only_once():
    profile = VirtualTreeTestMission(6.0, 9001)

    first = profile.tree_completed(real_tree(), (0.0, 0.0, 0.0))
    second = profile.tree_completed(real_tree(), (0.0, 0.0, 0.0))

    assert first is not None
    assert second is None


def test_profile_decorates_map_without_duplicate_virtual_id():
    profile = VirtualTreeTestMission(6.0, 9001)
    profile.tree_completed(real_tree(), (0.0, 0.0, 0.0))

    once = profile.decorate_tree_map([real_tree()])
    twice = profile.decorate_tree_map(once)

    assert [tree.id for tree in twice].count(9001) == 1


def test_profile_supports_coordinates_relative_to_home():
    profile = VirtualTreeTestMission(
        position_mode='home_relative', position_x=6.0, position_y=-2.0)

    target = profile.tree_completed(real_tree(), (1.0, 3.0, 0.0))

    assert target.x == 7.0
    assert target.y == 1.0


def test_profile_supports_absolute_map_coordinates():
    profile = VirtualTreeTestMission(
        position_mode='map', position_x=12.5, position_y=-4.0)

    target = profile.tree_completed(real_tree(), (1.0, 3.0, 0.0))

    assert target.x == 12.5
    assert target.y == -4.0
