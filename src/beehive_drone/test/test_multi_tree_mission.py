"""Regression tests for the minimal multi-tree extension of the Jetson FSM."""

from types import SimpleNamespace

from beehive_drone.mission_state_machine import (
    MissionStateMachine,
    continue_multi_tree,
)


def test_single_tree_never_selects_another_tree():
    assert not continue_multi_tree('single_tree', 0, 1)
    assert not continue_multi_tree('single_tree', 3, 1)


def test_multi_tree_obeys_limit_and_supports_unlimited_mode():
    assert continue_multi_tree('multi_tree', 3, 1)
    assert continue_multi_tree('multi_tree', 3, 2)
    assert not continue_multi_tree('multi_tree', 3, 3)
    assert continue_multi_tree('multi_tree', 0, 100)


def test_completed_tree_cannot_be_selected_again():
    fsm = SimpleNamespace(
        current_pose=SimpleNamespace(
            pose=SimpleNamespace(
                position=SimpleNamespace(x=0.0, y=0.0))),
        trees=[
            SimpleNamespace(id=1, x=2.0, y=0.0, inspected=False),
            SimpleNamespace(id=2, x=4.0, y=0.0, inspected=False),
        ],
        completed_tree_ids={1},
        explore_dir_x=1.0,
        require_tree_ahead=True,
    )
    fsm.distance = MissionStateMachine.distance.__get__(fsm)

    selected = MissionStateMachine.find_uninspected_tree(fsm)

    assert selected.id == 2


def test_virtual_test_can_select_tree_behind_exploration_direction():
    fsm = SimpleNamespace(
        current_pose=SimpleNamespace(
            pose=SimpleNamespace(
                position=SimpleNamespace(x=8.0, y=0.0))),
        trees=[SimpleNamespace(
            id=9001, x=2.0, y=0.0, inspected=False)],
        completed_tree_ids=set(),
        explore_dir_x=1.0,
        require_tree_ahead=False,
    )
    fsm.distance = MissionStateMachine.distance.__get__(fsm)

    selected = MissionStateMachine.find_uninspected_tree(fsm)

    assert selected.id == 9001
