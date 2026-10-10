# Saved point cloud and voxel map

Build once, then run the usual simulation:

```bash
cd /home/abin/polinasi_lidar
bash tools/build_ros.sh
bash tools/start_simulation.sh ground_truth clear_orbit
```

The launcher prints a unique folder under `reports/ros_<mode>_<scenario>_<time>_<pid>/map_3d/`.
Open **map.html** in a web browser after the run. It works offline: no server,
installation, or recording playback is needed. Drag to rotate, scroll to zoom,
toggle layers, and use the height slider to look inside the voxel grid.

- Cyan points: accumulated LiDAR surface measurements.
- Red cubes: occupied voxels (obstacles, including foliage and ground).
- Green cubes: observed-free voxels, plus the explicit surveyed launch-pad prior.
  In mapping mode these are historical observations, not current safety certificates.
- Grey cubes: unknown voxels. Unknown does **not** mean empty.

## Flight-path checkboxes

New logs also save `flight_paths` in map coordinates with independent checkboxes:

- Yellow/dashed **Rute rencana**: nominal survey waypoints, not a certified safe path.
- Blue **Lintasan perintah terakhir**: the current/last trajectory snapshot,
  not the complete history of every replan.
- Green **Jalur aktual**: sampled localisation positions, including takeoff and
  landing. It is never reconstructed from targets. In sensor mode it is an
  estimated path, not proof of localisation accuracy.

Uncheck to hide each layer; voxel height slicing does not hide paths. Missing
localisation/transform intervals are not joined by invented lines. Measurement-
time transforms numerically convert odometry to `map`. The path archive samples
initially every 0.2 s and adaptively decimates above 20,000 samples, preserving
the first/latest stored sample; timestamps, sample period, gap segment IDs and decimation flag are
saved. This limit is separate from the existing RViz live-path buffer.

Old logs did not store actual paths. Make a **separate** updated viewer while
keeping their original HTML/JSON; use the exact saved run configuration if you
want its nominal route included:

```bash
python3 tools/refresh_map_viewer.py reports/RUN/map_3d/map.json \
  --config .dependencies/RUN/mission_config.json
```

This creates `map_paths.html`. It cannot restore missing actual-path data and
disables that checkbox with an explanation. Future simulations save paths in
the usual `map.html` automatically. No controller, route or MAVROS producer is
changed by these display/logging features.

The map is saved every 10 **simulation seconds** and again on graceful shutdown.
Reload an open page to see a newer snapshot. Stop the simulation with Ctrl+C;
forced termination can leave only the last periodic snapshot.
**map.json** contains the point coordinates, known voxel grid indices, origin,
resolution, timestamps, and localisation-mode label. Voxel centre is
`origin + (index + 0.5) * resolution`.

Points use the same measurement-time transform and drone self-filter as the
navigation map. Invalid rays and finite no-return endpoints are excluded from
the surface cloud. Points are downsampled to one per 0.1 m cell, with a 100,000
cell storage cap for inspection; mapping uses a 1,000,000-cell cap. The cap is
reported in the file. They are historical measurements, not
necessarily the current obstacle state. Drift in localisation can distort them.
The map shows **uninflated** cells, not the enlarged safety envelope.
New simulations use **20 cm** voxels, shared with real navigation; older saved
maps keep their original size. See [voxel resolution](VOXEL_RESOLUTION.md).

Mapping format 2 exports all known occupied/free cells sparsely over the full
farm. Every unlisted cell is explicitly **unknown**, with an exact unknown count
and a bounded sample for display. Older format 1 exports all local cells.
The browser displays at most 6,000 cells per visible
layer for responsiveness (uniformly sampled when needed). A height slice reduces
clutter. Occupied cells are initially enabled; free/unknown start hidden.
This is a 3D data snapshot, not a video, rosbag, complete environment survey, or
proof of successful navigation/LiDAR localisation. Existing logs cannot recreate
LiDAR measurements that were never saved.

RViz separates local navigation occupancy from global obstacle history:
`/mapping/global_occupied` and `/mapping/global_cloud`. For responsiveness,
these displays sample at most 12,000 occupied cells and 30,000 points;
the saved archive retains all stored observations up to its documented cap.

For a manually launched ROS stack, opt in with an absolute output directory:

```bash
ros2 launch polinasi_nav sitl_navigation.launch.py localisation:=ground_truth \
  map_log_dir:=/home/abin/polinasi_lidar/reports/manual_map_3d
```

The exporter adds no control publisher and never alters the navigation map.
Inspection disk writing runs on a worker thread; the separate mapping mission
uses a worker process for map export. Snapshot preparation and bounded
point downsampling still consume CPU and memory. Disk errors are logged to ROS.

## Verification

Four new tests cover filtering/downsampling/storage limits, exact export of all
voxel states without map mutation, offline file replacement, and JavaScript
drawing/rotate/zoom/layer/height controls (a mocked canvas, not visual browser QA).
The complete ROS regression suite passed: 43 tests.
An unarmed ground-truth Gazebo run saved a final snapshot at 19.63 simulation
seconds: 2,096 surface points, 533 occupied, 2,876 free, and 48,431 unknown cells.
All 51,840 cells were accounted for; graceful shutdown saved a newer snapshot
than the periodic export and left no temporary files. This was a stationary
logging check, **not** a flight or sensor-localisation validation.

Example: [saved Gazebo map](../reports/ros_ground_truth_clear_orbit_20261009_145457_16165/map_3d/map.html).
