# Multi-tree simulation and per-tree recordings

Run Gazebo plantation_737c519.sdf, SITL using sitl_sim.parm, and MAVROS as
usual. Replace other mission/test launches with:

```bash
ros2 launch beehive_drone multi_tree_simulation.launch.py
```

It auto-starts, uses adaptive orbit, continues to uninspected detected trees,
and has no tree-count limit (max_trees=0). Backward candidates are allowed.
This is sensor-based exploration, not guaranteed exhaustive coverage of every
world tree. Trees outside sensor visibility or rejected by PCL can be missed.
No ground-truth tree list is injected. Full multi-tree flight is not yet validated;
normal return was unstable in earlier single-tree testing.

Records: ~/beehive_mission_reports/multi_tree/YYYYMMDD_HHMMSS/
- hover_altitudes.csv: 5 Hz during VERIFY_TREE, START_ORBIT, POST_ORBIT_HOVER.
  Includes world Z, FC local Z, rangefinder, filtered AGL, commanded Z,
  observed base/height, age of the tree's latest height observation.
- tree_NNNN/STATE_lidar.png: raw LiDAR scene XY and XZ views.
- tree_NNNN/STATE_lidar.npz: full scan XYZ points (sensor frame).
- tree_NNNN/STATE_metadata.json: frame, scan timestamp, height estimate and age.
Each image is captured once per tree/state when a fresh nonempty scan exists.
These are per-tree observation snapshots, not isolated segmented tree images.
Height estimates still come from camera-depth PCL cylinders, not LiDAR validation.
Missing LiDAR produces no fabricated image. Check ros2 topic hz /livox/top/points.
Missing/stale altitude telemetry is represented by NaN.

Only one mission/controller stack should run at a time.

## Hilly world and live LiDAR

Gazebo: world:=plantation_737c519_hilly.sdf.
The multi-tree launch disables use_absolute_trunk_base_gate because world Z=0
is not the ground surface on hills. Radius/height/confidence gates remain;
this is not terrain-ground validation and can admit additional false positives.
RViz: fixed frame livox_top_link, PointCloud2 topic /livox/top/points,
Reliability Best Effort, Color Transformer AxisColor/Z or FlatColor.
Use sensor frame to inspect the current scan without depending on world TF.
Full hilly multi-tree flight has not been validated.
