import math
from pathlib import Path

from beehive_drone.world_ground_truth import (
    load_tree_ground_truth,
    nearest_ground_truth,
)


WORLD = (
    Path(__file__).parents[2]
    / "uav_plantation_sim"
    / "worlds"
    / "plantation.sdf"
)


def test_loads_only_tree_models_from_world():
    trees = load_tree_ground_truth(WORLD)
    assert len(trees) == 170
    assert all(tree[0].startswith("tree_") for tree in trees)


def test_known_and_ghost_coordinates_are_distinguished():
    trees = load_tree_ground_truth(WORLD)
    tree, error = nearest_ground_truth(7.1, -0.05, trees)
    assert tree[0] == "tree_05_08"
    assert error < 0.12

    _, ghost_error = nearest_ground_truth(7.3, -7.36, trees)
    assert ghost_error > 1.25
    assert math.isfinite(ghost_error)
