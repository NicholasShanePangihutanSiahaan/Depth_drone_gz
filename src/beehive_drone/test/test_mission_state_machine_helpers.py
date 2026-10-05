import math

from beehive_drone.mission_state_machine import (
    landing_command_due,
    select_nearest_tree,
    yaw_aligned,
)


def test_first_land_request_is_sent():
    assert landing_command_due('GUIDED', float('inf'), 1.0)


def test_land_request_is_throttled_while_mode_change_is_pending():
    assert not landing_command_due('GUIDED', 0.2, 1.0)


def test_land_request_stops_after_flight_controller_reports_land():
    assert not landing_command_due('LAND', float('inf'), 1.0)


def test_yaw_alignment_uses_shortest_path_across_wrap():
    assert yaw_aligned(math.radians(179), math.radians(-179), math.radians(3))


def test_yaw_alignment_rejects_large_error():
    assert not yaw_aligned(0.0, math.radians(12), math.radians(7.5))


class FakeTree:
    def __init__(self, tree_id, x, y, confidence=1.0, inspected=False):
        self.id = tree_id
        self.x = x
        self.y = y
        self.confidence = confidence
        self.inspected = inspected


def test_multi_tree_selection_uses_nearest_eligible_tree():
    trees = [FakeTree(1, 8.0, 0.0), FakeTree(2, 3.0, 0.0)]
    assert select_nearest_tree(trees, 0.0, 0.0, set(), 20.0, 0.5).id == 2


def test_multi_tree_selection_excludes_completed_and_rejected_entries():
    trees = [FakeTree(1, 3.0, 0.0), FakeTree(2, 4.0, 0.0)]
    assert select_nearest_tree(trees, 0.0, 0.0, {1}, 20.0, 0.5).id == 2
