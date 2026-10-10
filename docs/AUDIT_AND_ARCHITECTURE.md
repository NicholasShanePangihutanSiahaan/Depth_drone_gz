# Current revision audit

Upstream HEAD was read from GitHub on 2026-10-07:
`151a6b3b8b916dada056db9f0f212036192c348b`. The original local checkout matched
and was clean. A local clone with `--no-hardlinks` preserved history independently;
its remote was removed. No push or hardware command was made.

| Finding | Verified evidence and effect |
| --- | --- |
| ROS 2 Humble / MAVROS / ArduPilot | `ReadME.md`, manifests and flight manager use these conventions. |
| ZED ExternalNav bridge | `vision_to_mavros.py` forwards pose to vision input; preserves nonzero stamps but does not validate frames or freshness before forwarding. |
| Orbit | `dynamic_orbit_controller.py` publishes `/control/dynamic_target`, faces the tree and labels poses `odom`. |
| Vortex | Uses `/map/trees` and XY corrections; branch subscription remains commented. No arbitrary 3D obstacle route search. |
| Final adapter | `position_setpoint_controller.py` consumes safe targets; publishes MAVROS local position. Its slew and terrain corrections occur after avoidance. It labels output `map` without a transform. |
| OctoMap | Launch/config exist in `point-cloud-test`; frame settings use `plantation` for both map and base. No planner consumes the map. Ground filtering in other perception paths is inappropriate for flight collision checks. |
| Simulator | Ground-truth adapter exists; Gazebo models/worlds and SITL parameters were absent. Adapter replaces stamps/frame names and publishes `/mavros/odometry/in`; ROS 2 MAVROS's current odometry plugin actually subscribes to `~/out` for FCU input. |
| Entry point | `mission.launch.py` and `mission_slim.launch.py` are near-identical real one-tree paths. `real_mission.launch.py` and `simulation_mission.launch.py` provide a different multi-node graph. There was no single reproducible simulator entry point. |
| Broken registration | Slim mission references `simple_single_tree_mission` and `pcl_tree_mapper`, missing from `setup.py`. Both are now registered; `ament_python` build dependency added. |
| Discovery | `point-cloud-test`, `pcl_cstm_msg`, `open3d_vis` contain `COLCON_IGNORE`; original README's package selection would not discover them. They stay ignored for the new prototype. |
| Perception dependencies | The inherited package declares ZED/PCL dependencies even when running flight-manager only. The documented minimal build deliberately skips these optional keys. |
| Legacy paths | Retained as historical code. Never combine their launches with the new launch. |

Run `python3 tools/audit_repository.py` for executable/manifest checks and
`colcon list --base-paths beehive_drone uav_interfaces polinasi_nav` for discovery.
Audit now reports no missing Python launch executable or invalid entry function.
This does not establish that optional ZED/PCL perception builds successfully.

# New control graph

```text
Gazebo /simulation/lidar -> sensor_gate -> /livox/lidar -----------+
Gazebo /livox/imu --------> sensor-only ICP estimator (sensor mode) |
Gazebo truth odometry ---> truth adapter (ground_truth mode)      |
                               |                                 |
                   /localisation/odometry -> ExternalNav          |
                               |          /mavros/odometry/out    |
                               +----> voxel ray map <-------------+
hardcoded trees + mock events ------> mission + viewpoints
voxel map -> 3D A* -> timed trajectory -> final map/braking gate
                                      -> /control/safe_target_pose (diagnostic)
                                      -> /mavros/setpoint_position/local
mission -> /flight/cmd/* -> inherited flight_manager -> MAVROS services
mission -> /actuators/mock_buzzer (software Bool only)
```

Only `polinasi_navigation` publishes the final position setpoints. There is no
vortex, dynamic-orbit node, velocity node, direct slim-mission publisher, ZED
vision bridge, or old slew/terrain adapter in this launch. Internal mission
goals have one owner. `/control/safe_target_pose` is a diagnostic copy, not a
second control chain. Exactly one localisation node selects its subscriptions
at startup. LiDAR dropout injection is another sole topic producer.

The separate mapping launch now selects `/mavros/setpoint_raw/local` instead
of the position interface, with one final producer and the inactive interface
unused. Its C2 quintic chunks carry position, velocity and acceleration together;
adaptive time scaling preserves their derivative relationship. MAVROS raw ROS
fields are ENU and are numerically converted to NED by MAVROS once. The fixed
map-to-FCU alignment rotates derivatives without translating them. This mapping
change does not activate both interfaces or replace the inspection controller.

The inherited flight manager remains a MAVROS service adapter. The new mission
checks FCU state and measured altitude/velocity instead of relying on the old
altitude-only 'hover' flag. Startup, takeoff timeout and operator mode takeover
are explicit. Landing completion requires disarmed state and a near-ground
range measurement. An abort holds; it does not choose an unchecked return route.

# Frames and time

World/map/odom are ENU: X east, Y north, Z up. Body is FLU: X forward, Y left,
Z up. The pinned Gazebo plugin applies a real ENU->NED rotation (180° roll,
90° yaw); body->FRD uses 180° roll. MAVROS owns the MAVLink conversions.
Numerically ENU `[1,2,3]` becomes NED `[2,1,-3]`; these are tested matrices.

`map -> odom` is explicitly identity in the fixed local simulation, which has
no loop closure or map jumps. `odom -> base_link` comes only from the selected
localisation source. Static body->MID-360 matches the actual SDF mount; the IMU
is colocated in this simplified model. A truth adapter applies the configured
`world_to_odom` translation/rotation numerically, preserves the measurement
stamp, and rejects unexpected frames. It never just renames an incompatible
coordinate system.

Odometry pose is in the parent frame; velocity is in the child/body frame.
Gazebo child velocity is rotated to world before prediction and rotated back
for ROS Odometry. MAVROS's required `odom_ned` and `base_link_frd` transforms are
provided by MAVROS's own static TF setup, with no duplicate broadcaster in this
package; its dynamic pose TF publication is disabled to avoid multiple base owners.
The planner also checks FCU pose/yaw agreement with ExternalNav before arming
and during flight. A mismatch aborts rather than changing a frame label.

All new ROS nodes require `use_sim_time=true` and consume Gazebo `/clock`.
Cloud transforms are looked up at acquisition time, never latest-time TF.
Health uses message timestamps; old, future and unordered estimates are rejected.
ExternalNav retains the source timestamp/covariance and stops forwarding an
unhealthy estimate. ROS sensor subscriptions use best effort (accepts either
best-effort or reliable publishers); commands/ExternalNav use reliable QoS.
MAVROS timesync is disabled for this shared-clock simulator configuration.

The ZED remains the intended forward inspection camera. The simulator supplies
a simple RGB placeholder, not a ZED SDK/stereo model. The optional pitch joint
reports actual joint angle; its dynamic TF uses feedback, not the desired angle.
Camera optical axes are handled separately. Camera motion never enters the
LiDAR body estimator. Failed/stale pitch feedback prevents mock confirmation.
RGB replacement and a physical pitch mount remain future decisions.

# Map and trajectory policy

Occupied/free/unknown cells stay distinct. A valid ray marks space before its
first return free and its endpoint occupied. Behind a hit and around invalid
rays stays unknown. Occupancy is sticky within a mission; free cells expire.
No leaf, ground or canopy filter removes collision obstacles. This favours
safety over progress when foliage moves. Map dimensions bound memory usage.

A yaw-independent cylinder covers the configured footprint and vertical body
extent, plus clearance and a tracking reserve. Voxel extent/quantisation are
also included. Planned paths reserve tracking error; measured-body checks use
the physical dimensions plus clearance. Unknown and map boundaries are blocked.
The configured tracking limit must stay within the reserve. Command speed is
lower than the measured-flight limit to leave room for tracking overshoot.

A* searches 3D neighbours and checks all diagonal corner cells. Shortening
uses exact grid-plane crossings; fixed-spacing samples missed a narrow corner
in an early return-path test, now covered by regression. Quintic straight
segments start/end with zero velocity and acceleration. Derivatives bound
speed, acceleration and jerk. Clear continuous orbits use a smooth analytic
angular trajectory. A blocked circuit falls back to checked segments and
3D detours; it can stop between segments.

Before execution the whole trajectory is tested for a stopping corridor and
slowed if needed. The jerk-bounded stop preserves initial velocity/acceleration.
Both actual measured motion and the next command receive checks. A changed
route brakes before planning again from rest; lost sensors/localisation latch
an abort. If an obstacle appears inside an unavoidable stopping envelope, the
node explicitly reports `emergency_unavoidable` rather than claiming safety.

Optional terrain correction modifies the goal BEFORE planning. During motion,
a changed required height triggers braking/replanning; stale range holds.
There is no downstream altitude modifier. CPU mapping runs on a worker and
swaps complete map snapshots; final setpoints do not wait for ray integration.
Planning deadlines and real Jetson performance still require ROS/SITL profiling.

References: [ROS 2 MAVROS odometry source](https://github.com/mavlink/mavros/blob/ros2/mavros_extras/src/plugins/odom.cpp),
[Gazebo odometry source](https://github.com/gazebosim/gz-sim/blob/gz-sim8/src/systems/odometry_publisher/OdometryPublisher.cc),
[ArduPilot ExternalNav](https://ardupilot.org/dev/docs/mavlink-nongps-position-estimation.html).
