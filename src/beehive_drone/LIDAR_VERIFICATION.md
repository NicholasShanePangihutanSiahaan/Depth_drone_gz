# Camera candidates, LiDAR verification

Simulation mission, adaptive orbit validation, and multi-tree simulation launches
now include lidar_tree_verifier. sim.yaml requires LiDAR verification; real.yaml
explicitly keeps it disabled pending real sensor/TF integration.

Flow: camera-depth PCL candidate -> approach -> VERIFY_TREE -> LiDAR-corrected
center and visible trunk extent -> altitude adjustment -> orbit.
LiDAR point clouds come from /livox/top/points and are transformed to odom at
their own timestamp using TF. Missing TF never falls back to an identity transform.

Conservative geometry gates:
- Search within 0.8 m XY of camera candidate.
- Circle RANSAC in 0.25 m vertical slices, radius 0.10–0.65 m.
- At least six inliers and 60% inlier fraction per accepted slice.
- At least 0.5 rad observed arc, >=4 consistent vertical slices.
- At least 0.8 m observed extent; consistent centers in >=3 separate fresh scans.
These are geometry checks, not semantic proof of an oil palm. Cylindrical poles
may still pass. Height is the visible trunk segment, not total tree height or a
verified ground-contact base. Association deliberately rejects large camera errors
rather than snapping to a nearby unrelated real tree.

Candidate request Float64MultiArray /mission/lidar_candidate:
[id, camera_x, camera_y, camera_z] (odom frame by contract).
Verified response /perception/lidar_verified_tree:
[id, corrected_x, corrected_y, observed_bottom_z, visible_height, radius].

FSM only accepts responses for its selected ID during VERIFY_TREE. It requires
fresh verification before orbit. A failed verification times out after 8 s and
blacklists that candidate ID for the flight. It does not orbit using camera height
as a fallback. The existing four-second stable verification counter only advances
while a fresh LiDAR result exists. Camera height updates no longer overwrite the
orbit estimate when LiDAR verification is required. Corrected map positions are
not subsequently moved by camera EMA after validation.

Restart the mission stack after build. Keep Gazebo/SITL/MAVROS running and run
exactly one of:
ros2 launch beehive_drone adaptive_orbit_validation.launch.py
ros2 launch beehive_drone multi_tree_simulation.launch.py

Recorder hover CSV now includes height_source. Before LiDAR validation this can
be camera; after validation it is lidar. Raw scan screenshots are not segmented
images. Full simulated flight of this new verification path remains unvalidated.

## YAML tuning

Edit the lidar_tree_verifier section in config/sim.yaml (simulation) or
config/real.yaml (real profile). simulation_mission,
adaptive_orbit_validation and multi_tree_simulation load this file automatically.
Restart the mission after edits; verifier tuning parameters are read-only while
the node is running. Defaults preserve the previous thresholds.

association_radius is the point ROI radius (0.8 m). max_center_offset is the
stricter fitted-center correction gate (0.6 m). FSM has a separate
lidar_max_position_correction gate (0.8 m); all gates must pass. Increasing
only association_radius does not increase the allowed fitted-center correction.
Keep target_frame=odom for these launches because candidate coordinates and
verified results use odom. A different frame requires changing the upstream FSM
contract too. Lengths are meters, times seconds, arc angles radians.

For standalone use:
ros2 run beehive_drone lidar_tree_verifier --ros-args --params-file src/beehive_drone/config/sim.yaml

Do not run standalone verifier alongside a launch that already starts it.

The real profile keeps require_lidar_verification disabled as before. Defining
tuning in real.yaml does not start a verifier or enable verification by itself.
