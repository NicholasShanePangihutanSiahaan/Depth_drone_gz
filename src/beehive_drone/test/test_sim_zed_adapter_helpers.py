from beehive_drone.sim_zed_adapter import parse_tree_positions


def test_empty_tree_list_preserves_legacy_single_tree():
    assert parse_tree_positions('', (7.0, 0.0, 0.35)) == [(7.0, 0.0, 0.35)]


def test_multi_tree_positions_inherit_or_override_ground_height():
    trees = parse_tree_positions('7,-3;7,3,0.8', (7.0, 0.0, 0.35))
    assert trees == [(7.0, -3.0, 0.35), (7.0, 3.0, 0.8)]
