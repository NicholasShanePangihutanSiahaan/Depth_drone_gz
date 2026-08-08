#!/usr/bin/env python3
"""Compare mission CSV tree coordinates with tree poses in an SDF world."""

import argparse
import csv
import pathlib
import sys

from beehive_drone.world_ground_truth import (
    load_tree_ground_truth,
    nearest_ground_truth,
)


def read_mapped_trees(csv_path):
    lines = pathlib.Path(csv_path).read_text(encoding="utf-8").splitlines()
    try:
        header_index = next(
            index for index, line in enumerate(lines)
            if line.strip().startswith("ID,X,Y,")
        )
    except StopIteration as exc:
        raise ValueError("CSV tidak memiliki tabel pohon ID,X,Y") from exc
    return list(csv.DictReader(lines[header_index:]))


def evaluate(csv_path, world_path, tolerance):
    ground_truth = load_tree_ground_truth(world_path)
    mapped = read_mapped_trees(csv_path)
    results = []
    for row in mapped:
        x, y = float(row["X"]), float(row["Y"])
        truth, error = nearest_ground_truth(x, y, ground_truth)
        results.append((row["ID"], x, y, truth, error, error <= tolerance))
    return ground_truth, results


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("csv", help="mission_result CSV")
    parser.add_argument("world", help="Gazebo SDF world")
    parser.add_argument("--tolerance", type=float, default=1.25)
    args = parser.parse_args()

    truth, results = evaluate(args.csv, args.world, args.tolerance)
    valid = sum(result[-1] for result in results)
    print(f"Ground truth: {len(truth)} pohon; hasil map: {len(results)}")
    for tree_id, x, y, nearest, error, accepted in results:
        label = "BENAR" if accepted else "POHON_GAIB"
        nearest_name = nearest[0] if nearest else "-"
        print(
            f"ID {tree_id:>3}: ({x:7.2f},{y:7.2f}) -> {nearest_name:>12} "
            f"error={error:5.2f} m [{label}]"
        )
    precision = 100.0 * valid / len(results) if results else 0.0
    print(f"Koordinat benar: {valid}/{len(results)} ({precision:.1f}%)")
    return 0 if valid == len(results) else 1


if __name__ == "__main__":
    sys.exit(main())
