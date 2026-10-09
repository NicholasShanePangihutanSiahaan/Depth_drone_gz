"""Mission profile for one detected tree followed by one virtual tree."""

import math
from copy import deepcopy

from uav_interfaces.msg import Tree


def homeward_point(tree_xy, home_xy, offset):
    """Place a point ``offset`` metres from a tree toward home."""
    dx = home_xy[0] - tree_xy[0]
    dy = home_xy[1] - tree_xy[1]
    distance = math.hypot(dx, dy)
    if distance < 1.0e-6:
        raise ValueError('tree and home positions are coincident')
    scale = float(offset) / distance
    return tree_xy[0] + scale * dx, tree_xy[1] + scale * dy


class VirtualTreeTestMission:
    """Own the synthetic second target without modifying basic-orbit data."""

    def __init__(self, offset_toward_home=6.0, virtual_tree_id=9001):
        self.offset_toward_home = max(0.1, float(offset_toward_home))
        self.virtual_tree_id = int(virtual_tree_id)
        self.virtual_tree = None

    def is_virtual(self, tree):
        return tree is not None and int(tree.id) == self.virtual_tree_id

    def tree_completed(self, completed_tree, home_pose):
        """Create the virtual target once, after the first real tree."""
        if completed_tree is None or self.is_virtual(completed_tree) or \
                self.virtual_tree is not None:
            return None
        if home_pose is None:
            raise ValueError('home pose is unavailable')
        virtual_x, virtual_y = homeward_point(
            (float(completed_tree.x), float(completed_tree.y)),
            (float(home_pose[0]), float(home_pose[1])),
            self.offset_toward_home)
        target = Tree()
        target.id = self.virtual_tree_id
        target.x = virtual_x
        target.y = virtual_y
        target.z = float(completed_tree.z)
        target.confidence = 1.0
        target.inspected = False
        target.validated = True
        target.orbit_count = 0
        self.virtual_tree = target
        return deepcopy(target)

    def decorate_tree_map(self, trees):
        """Expose the target to the existing selector and verifier."""
        result = list(trees)
        if self.virtual_tree is not None and not any(
                int(tree.id) == self.virtual_tree_id for tree in result):
            result.append(deepcopy(self.virtual_tree))
        return result
