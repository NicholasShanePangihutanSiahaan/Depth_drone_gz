"""Utilities for comparing mapped trees with Gazebo world ground truth."""

import math
import xml.etree.ElementTree as ET


PALM_MODEL_PREFIXES = ("oil_palm", "orchard_tree")


def load_tree_ground_truth(world_path):
    """Return ``(name, x, y, z)`` trees declared as SDF includes."""
    root = ET.parse(world_path).getroot()
    trees = []
    for include in root.findall(".//include"):
        uri = (include.findtext("uri") or "").strip()
        model_name = uri.rsplit("/", 1)[-1]
        if not model_name.startswith(PALM_MODEL_PREFIXES):
            continue
        values = (include.findtext("pose") or "").split()
        if len(values) < 3:
            continue
        name = (include.findtext("name") or model_name).strip()
        trees.append((name, float(values[0]), float(values[1]), float(values[2])))
    return trees


def nearest_ground_truth(x, y, trees):
    """Return the nearest tree tuple and its horizontal error."""
    if not trees:
        return None, math.inf
    nearest = min(trees, key=lambda tree: math.hypot(x - tree[1], y - tree[2]))
    return nearest, math.hypot(x - nearest[1], y - nearest[2])
