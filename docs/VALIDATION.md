# What changed, why, and how it was tested

| Change | Reason | Evidence |
| --- | --- | --- |
| Independent local repository | Preserve original project | Original git status remains clean; new repository has no remote. Upstream hash recorded. |
| Slim executable registration / build manifest | Launch referenced missing executables | Static AST audit now passes; colcon discovers exactly the three selected prototype packages. |
| Gazebo worlds, LiDAR, IMU, camera/range assets | Simulator pieces were missing | Harmonic 8.15 loads the model; actual messages confirm sensor frames, stamp, 360×32 rays, −7°..52°, IMU acceleration. |
| Pinned ArduCopter/Gazebo dependencies | Reproducible baseline | Gazebo plugin and Copter 4.6.3 compiled locally. Direct smoke confirms disabled GPS and all navigation sources ExternalNav. |
| One map/trajectory/final-control path | Avoid XY-only and downstream unchecked changes | Unit tests check inflated 3D obstacles, unknown cells, trajectory limits, terrain correction and braking. Live ROS check confirms exactly one publisher per control interface. |
| Humble/MAVROS setup and supervised launcher | Make the ROS prototype runnable here | Humble/Harmonic bridge installed; MAVROS 2.15.1 source build passed. Restart confirms automatic telemetry; invalid infinite rays are filtered before transformation. Shutdown handling stops only launcher-owned processes. |
| Free/occupied/unknown ray map | Foliage/ground must remain collision obstacles | Occlusion, NaN no-return and free-age tests; occupied obstacles stay sticky. Actual full mission with the live map is pending. |
| Exact supercover segment checks | Sampling missed tiny corner intersections | Thin-corner and zero-width-contact regression tests. |
| 3D detours and stopping reserve | A safe path can have an unsafe stop | Vertical wall / foliage scenarios; trajectory slowdown checks braking at 20 Hz. Curved braking uses polynomial control-hull bounds instead of point samples alone. |
| Quintic translation / analytic orbit | A slew limiter does not prove feasibility | Numerical velocity, acceleration and jerk tests; final commanded and executed curves logged. Linear limits and yaw-rate limits are checked; full flight dynamics are not proven. |
| Camera candidates / mock confirmations | Navigation must not depend on detection | Different heights, radii and nearby angles scored for clearance, visibility, useful framing and travel. Buzzer requires multiple confirmations at a settled visible view. No real GPIO. |
| Sensor-mode ICP + IMU and ExternalNav bridge | Separate true sensor input from truth tests | 80-frame synthetic sensor-only experiment; rejects planar scans and stale/unordered IMU. ROS2 topic direction and transforms checked against primary source. Full LiDAR flight remains pending. |
| Failure handling | Stale sensors must not continue a route | Dropout, localisation loss, stale range, dynamic obstacle/replan and blocked-route tests. Buzzer stops on failure. |
| Bounded airborne gap observation | Observe unknown gaps from safe positions before abandoning a mission leg | 11 exploration regressions and four analytic-ray cases: observable gap recovered; sealed/blind gaps held; dropout braked. Full live gap-recovery flight is pending. |
| Packet freshness separated from map-measurement age | A sparse valid scan is not a disconnected LiDAR | ROS sparse-cloud regression; unknown/free expiry and stopping gates retained. Exploration observes only after a mapped scan acquired after arrival. |
| Confirmed origin helper | An early reference message can be ignored before EKF initialisation | Live startup exposed ignored origin; repeated reference allowed normal arming. Updated helper waited for matching GPS_GLOBAL_ORIGIN acknowledgement on loopback. No GPS measurements or forced arm. |
| Normal LAND / one simulated autopilot IMU | Inherited IRLOCK/sonar were absent; synthetic redundant IMUs disagreed | Initial baseline reported gyro inconsistency and precision-landing failsafe. Simulation overrides retain normal arming checks; repeat flight reached ground and automatically disarmed. |

# Tests actually run

## Separate ROS mapping process and tree-only survey footprint (2026-10-10)

The latest pre-change run `040712` still hit 222/245 ms simulation-time control
gaps despite spawned workers. Its main callbacks still applied global map
deltas and collected visual/export data. Mapping launch now starts a separate
`polinasi_mapping_io` ROS process owning those tasks. Flight control receives
only the already-inflated local grid (bounded numeric transport, depth one,
measurement-time freshness gate); mapper has no flight/control publishers.
Unknown/occupied inflation, tracking, braking, 200 ms gap and 1 s map-age guards
remain active. This is process isolation, not a hard-real-time guarantee.

**145 regressions passed**, zero failures/errors/skips, including process
publisher ownership, numeric-grid transport and old/duplicate snapshot rejection.
The generated tour has **326 goals and 568.999 m nominal travel** within the
buffered convex footprint of the 29 configured simulated palms (4.5 m margin).
Tests check all goals remain within that polygon, <=2 m legs, and each tree
lies within 6 m of the nominal route. Tree coordinates guide mission geometry
only; they do not seed live obstacles/free space or prove visibility/coverage.
Global map bounds remain unchanged so measured returns outside the survey ROI
are still retained. Historical 586-goal results below refer to the old route.

Live check-flight evidence is recorded separately in
`reports/process_isolation_check_setup.json` and
`reports/process_isolation_check_flight.json` when the run finishes. Do not
infer flight success from the unit tests or a setup-only PASS.

## Control/runtime isolation (2026-10-09/10)

Mapping planning now uses a spawned process, not a Python thread sharing the
controller's GIL. Visual-only voxel/cloud/path messages are constructed and
serialized in a separate spawned process; the original navigation node publishes
their serialized bytes. No ROS node or control publisher is created in workers.
Queues permit only one pending local and one global visual job. Callback wall-time
maxima and timing-gap context are included in status/logs. Planner-worker timing
is also reported separately. Safety checks and the 0.2 **simulation-second**
control-gap guard remain unchanged. Default mapping pacing is restored to 0.2.

**119 regressions passed**, zero errors/failures/skips, after rebuilding all
three packages. Tests include spawned planning, live-map route invalidation,
visual serialization/frame/stamp preservation, bounded visual work, timing
measurement and explicit rejection of a 0.204 s control gap.

Actual initial `tour` flight, run `231817_96194`:

- Startup checks passed: `reports/isolated_tour_start_setup.json`.
- The persisted **130.0 simulation-second** snapshot records **1/586 goals**
  completed, state EXPLORE toward the next leg, empty failure and no recorded
  timing gap. Maximum sampled horizontal displacement is **3.195451 m**,
  maximum sampled altitude **2.301196 m**. Executed history has 609 samples.
- Local visual handoff maximum **11.692 ms wall time**; control tick maximum
  **518.909 ms wall time**. These are not simulation-time deadline values;
  worker timings are not time spent executing control. The live snapshot
  records no `clock_or_scheduler_gap`.
- The session was interrupted before the flight observer wrote its final report.
  Final ROS log contains network-unreachable warnings; the termination cause
  is not established. Landing/disarm and full mission completion were **not**
  confirmed. The old publisher/launch processes were absent on continuation.
- Summary of persisted evidence: `reports/isolated_tour_start_summary.json`;
  map: `reports/ros_ground_truth_palm_farm_20261009_231817_96194_mapping/map_3d/map.html`.
  This is initial progress past the earlier abort, not a whole-farm validation.

Fresh disarmed startup integration on 2026-10-10 received map-frame voxel
messages (11,423 displayed cells), global cloud (5,815 points) and all 586 survey
poses from the isolated visual worker. Mission start remained false and motors
disarmed. Evidence: `reports/runtime_visual_startup.json`. No GUI visual QA or
new complete flight was performed in this check.

Launcher now owns separate process groups, including ROS/spawned workers, and
rejects an existing navigation process before starting. A second-launch attempt
during the first run was correctly rejected without creating another controller.
The fresh disarmed run was stopped via Ctrl+C; subsequent checks found no
SITL/navigation processes and ports 5762/9002/14550 were free. Map exports remain
available. Logs are retained. See [runtime details](RUNTIME_ISOLATION.md).

## Faster mapping simulation pacing (2026-10-09)

At this historical trial, `start_mapping.sh` requested factor **0.3**, previously 0.2 (target 50%
faster wall-clock progress). Drone speed limits, sensor sample rates, 20 cm
voxels, 1 ms physics steps and safety thresholds are unchanged. An explicit
`POLINASI_SIM_RTF=0.2` restores conservative pacing. CPU/GPU load can reduce
the achieved factor, and opening GUI viewers adds load.

- A 0.5 trial reached takeoff, then BRAKE/HOLD_ABORT at 24.260 simulation
  seconds with `clock_or_scheduler_gap`; a 0.250 s callback interval exceeded
  the unchanged 0.2 s guard. Regression tests were also running during this
  trial. It is not a completed mission; operator LAND was used for cleanup.
  Evidence: `reports/fast_smoke_retry_flight.json`. Factor 0.5 was rejected
  as the default rather than loosening the guard.
- Factor **0.3 smoke flight passed**: all four goals, three continuous passes,
  return, automatic LAND and disarm, COMPLETE at **67.093 simulation seconds**.
  Maximum altitude **2.013813 m**, maximum sampled tracking error **0.011796 m**.
  Evidence: `reports/factor03_smoke_setup.json` (PASS) and
  `reports/factor03_smoke_flight.json` (completed).
- Gazebo reported instantaneous factor **0.299270**, physics step **0.001 s**;
  evidence: `reports/factor03_pacing.json`. This is an instantaneous observation,
  not the full-run average or a whole-farm timing guarantee.
- Regression coverage checks unchanged world geometry/physics/source files
  at factors 0.2, 0.3, 0.5 and 1.0. Only 0.3's short new flight passed here;
  factor 1.0 and the complete farm tour have not been flight-validated.

All three packages rebuilt; **112 regressions passed**, zero failures/errors/skips.

Startup also rejects an occupied SITL TCP port 5762 before starting new
processes. A previously stopped user run left a disarmed SITL process on this
port; it was explicitly identified and stopped with approval. The first 0.5
attempt never acquired flight telemetry (`reports/fast_smoke_flight.json`).
Launcher cleanup now also handles SIGHUP (terminal closure); logs are retained.
All flights above use ground-truth localisation, not validated LiDAR localisation.

Commands: [SURVEI_KEBUN.md](SURVEI_KEBUN.md).

## Continuous mapping, PVA feedforward and adaptive speed (2026-10-09)

Implemented bounded C2 quintic chunks joining up to four currently safe survey
goals. Interior goals require a new integrated scan after entering their pass
radius; endpoints still settle and stop before replanning. Curve hulls are
checked against inflated occupied/unknown cells. Smooth time dilation uses
the chain rule and reserves acceleration/jerk headroom. Mapping alone switches
to the sole `/mavros/setpoint_raw/local` position/velocity/acceleration publisher;
inspection retains its position interface. Numerical alignment rotates vectors
without translating derivatives; MAVROS performs the ENU/NED conversion once.

All three packages rebuilt; **109 tests passed**, zero failures/errors/skips.
New regressions cover C2 continuity, analytic derivatives, adaptive chain-rule
limits, occupied/unknown contacts between knots, stale LiDAR, missing new scans,
repeated out-and-back goals, async live-map invalidation, no batching into unknown
space, and nonidentity raw-command transformation. Known-map point-mass tests
are explicitly separate from the actual flight below.

Actual GPS-disabled Gazebo/ArduPilot ground-truth `smoke` flight:

- Setup: `reports/continuous_smoke_setup.json`, all checks PASS.
- Flight: `reports/continuous_smoke_flight.json`, **COMPLETE** at 66.099 simulation
  seconds; four goals, return, automatic LAND and disarm, no operator LAND.
- Three interior points passed without endpoint dwell, at command speeds
  approximately 0.137, 0.027 and 0.027 m/s. Pass radius was 0.2 m on this tiny
  0.25 m loop; this is not an exact stop-at-centre inspection test.
- Maximum altitude 2.014061 m; maximum sampled tracking error **0.010107 m**.
  Command speed peaked at 0.138198 m/s and acceleration at 0.155124 m/s².
- Adaptive scale reached 0.25 due to proximity to unknown-space boundaries.
  Increased caps do not imply increased speed everywhere.
- Minimum sampled occupied-map clearance during survey/return was 2.231253 m.
  This is a map estimate, **not** an independent physical collision-distance
  measurement; unknown space is checked separately.
- Exactly one raw-local publisher and zero position-local publishers.
  Over 721 regular command intervals, finite-difference position velocity
  differed from average feedforward velocity by at most 0.000115 m/s
  (RMS 0.0000121 m/s). This checks consistency, not full dynamic flight proof.
- Saved map: `reports/ros_ground_truth_palm_farm_20261009_214919_77507_mapping/map_3d/map.html`.
  Its JSON contains 380 timestamped localisation-derived executed-path samples.
  Ground-truth localisation is **not validated LiDAR localisation**.

Current navigation window is 10 × 10 × 8 m (100,000 cells), while archive bounds
and 20 cm voxels are unchanged. `check`/`tour` command caps are now 0.3 m/s;
`smoke` remains 0.2 m/s. The new higher-cap `check` and full tour have **not** been
flight-validated; historical 4 m flight results below used the old position-only
interface at 0.2 m/s. Full-flight dropout/blocked-route and sensor-based
localisation remain unvalidated. No hardware was operated.

See [Bahasa implementation notes](LINTASAN_KONTINU.md).

## Toggleable flight paths in map logs (2026-10-09)

New offline logs save nominal survey routes, the current planned trajectory
and sampled localisation-derived executed paths with independent HTML checkboxes.
This separates intended goals from observed motion without changing control.
Measurement-time map/odom transforms are numerical; missing transform/data
intervals leave segment breaks. Executed history is bounded and adaptively
decimated, preserving its first/latest stored sample and reporting timestamps.

All three packages rebuilt and **94 tests passed**, zero failures/errors/skips.
Tests exercise checkbox drawing/hiding in a mocked JavaScript canvas, old-log
fallback, gap handling, bounded independent snapshots, both occupancy export
formats and a nonidentity timestamped ROS transform. There was no new flight
or visual browser QA for this logging-only change.
The old successful flight's original JSON/HTML are preserved; its separate
`map_paths.html` includes only the nominal route from that run's configuration.
Old actual paths were never stored and are not invented or inferred from targets.

## Whole-farm archive / rolling navigation update (2026-10-09)

The new presets archive 92 × 82 × 10.8 m at 20 cm: **10,184,400** possible
cells. A rolling 8 m cube bounds navigation computation to **64,000** cells.
Sparse history keeps observations outside that cube; unlisted cells stay unknown.
Tests cover recentering, expiry, occupied precedence, bounded fixed-pad priors,
sparse export accounting, nominal route coverage, sensor freshness, unchanged
physics/FOV and run-local sensor-range configuration. Build/regression results
are recorded in `log_ros`: **89 tests passed**, with zero failures/errors/skips.

At this historical stage, `mapping_tour.json` had 586 short goals across zigzag lanes, approximately
1,085 m nominal travel. Geometry tests are not evidence of observed coverage or
collision-free full flight. Trunk geometry guides goals only, not map state.
Mapping free-history retention is 10,800 s in short tests and 21,600 s in the
long tour for the explicitly static simulator;
it is not validated for moving obstacles or hardware. Fresh cloud, localisation,
integrated-map age, tracking and trajectory safety checks remain active.

**Passed live small-loop mapping flight:**
`reports/coverage_smoke_slow_setup.json` passed setup; the flight in
`reports/coverage_smoke_slow_flight.json` completed takeoff, four survey points,
return, descent, normal automatic LAND and disarm at 84.065 simulation seconds.
Maximum altitude **2.004608 m**, maximum sampled tracking error **0.076590 m**;
one final setpoint publisher. This used **ground-truth localisation**, the old
12 m sensor range and real-time factor 0.2. Physics step remained 1 ms.
Its final map contains **89,269** points, **21,261** occupied, **96,251** free
and **10,066,888** unknown cells over the whole farm bounds. Only about **5.77%**
of horizontal columns contain observations: a small flight is not a full survey.
The initial callback-performance trials failed and remain in
`reports/coverage_smoke_flight.json` and `reports/coverage_smoke_flight_v2.json`.

The later 20 m `check` attempt passed setup but did not take off: inconsistent
synthetic IMUs preceded takeoff rejection (result 4). It was cleaned up with
normal simulator LAND, not counted as success. Evidence:
`reports/coverage_check_setup.json`, `reports/coverage_check_flight.json`.
The SITL manifest now also restricts backend startup with `INS_ENABLE_MASK=1`
to match the one supplied autopilot IMU, rather than only disabling use of two
redundant instances. Normal arming checks remain enabled.
An intermediate one-backend trial then correctly refused arming because the
upstream `copter.parm` still contained calibration offsets/scales for absent
IMU2 (`reports/coverage_check_single_imu_flight.json`). The simulation override
now clears that unused calibration only; the primary calibration is preserved.
Ten coverage/configuration tests passed again after that adjustment.
The next trial armed normally but still rejected takeoff. Read-only MAVLink
diagnostics showed no live EKF origin, global latitude/longitude zero and
non-finite home XYZ. ROS logs showed the origin ACK **before** EKF core
initialisation, which clears it. The helper now repeatedly confirms both
origin and finite home across three FCU/simulation seconds before notifying ROS.
It does not inject GPS measurements or set references during flight.
Failed evidence is retained in `reports/coverage_check_calibrated_flight.json`.

With stable references, the 20 m check took off once, used one observation move
and resumed SURVEY; the rolling window shifted from Y=-4 to Y=-5.8 m and
global known cells grew from about 96,000 to **363,671**. It then held on
the unchanged tracking guard at **0.250675 m**. It did not complete a waypoint;
operator-requested normal LAND/disarm was cleanup only. Evidence:
`reports/coverage_check_stable_reference_flight.json`. The command-speed preset
was subsequently reduced from 0.4 to 0.2 m/s, not the tracking guard increased.
Fifteen coverage/export tests passed after that change.

**Passed current 20 m / conservative-speed flight:**
`reports/coverage_check_conservative_setup.json` passed all setup checks;
`reports/coverage_check_conservative_flight.json` records one accepted takeoff,
one additional observation viewpoint, all four survey goals (out to 4 m and
back), automatic RETURN → DESCEND → LAND, then COMPLETE/disarm at **112.100 s**.
No operator LAND was used in this successful run. Maximum altitude was
**2.304780 m** (the extra observation view is above the 2 m survey plane);
maximum sampled tracking error **0.097942 m**. Exactly one final MAVROS
setpoint publisher remained active. The local-window Y origin moved from
**-4 m to -8.2 m** and returned towards home. This is a ground-truth flight,
not LiDAR localisation validation or complete dynamics certification.

The graceful final archive at 126.727 s contains **268,169** points,
**63,226** occupied cells, **361,154** free cells and **9,760,020** unknown cells;
all **10,184,400** possible cells are accounted for. **407,451** known cells
are outside the former 6 × 6 m box. About **17.94%** of horizontal grid columns
contain some observation; this is not 17.94% of tree surfaces or whole-farm
completion. Snapshot status is COMPLETE, armed=false, sensor range=20 m.
Open `reports/ros_ground_truth_palm_farm_20261009_204135_64865_mapping/map_3d/map.html`.

Full farm survey completion (now 326 goals), dynamic-world free-history safety, moving
sensor-localisation flight and Jetson timing remain **unvalidated**.
The tour budget was adjusted to 18,000 s with static-history TTL 21,600 s;
a regression checks that nominal stop-to-stop quintic timing fits the budget.
This long-tour-only adjustment was not the configuration of the successful
short flight above (which retained 10,800 s history).

## Mapping-only program update (2026-10-09)

All three selected packages rebuilt; **79 regression tests passed**, including
survey lifecycle, newly integrated scans, unknown-route refusal, asynchronous
route invalidation, MAVROS parameter acknowledgements, numerical FCU frame
conversion, and duplicate-versus-gapped simulation timestamps. Survey lifecycle
unit tests are point-mass tests, not ArduPilot flight evidence.

The mapping launch selects a separate controller and retains one final MAVROS
publisher. Its plugin configurator waits for explicit parameter acknowledgements;
the source-built MAVROS plugins disable global launch arguments, so YAML alone
did not apply the requested frame/clock settings. The fixed map/FCU transform
is now calibrated only after the helper confirms simulator origin and home.
The helper also requires a disarmed FCU. A repeated mission-start helper waits
for controller acknowledgement instead of assuming a one-shot message arrived.

An initial actual Gazebo/SITL attempt armed normally and rose to approximately
**1.80 m**, but did not visit any survey waypoint: it held on
`flight_management_timeout`. Its takeoff request incorrectly subtracted initial
body-centre height from a target referenced to home at world Z=0. That mapping
conversion was corrected and regression-tested. Normal LAND was requested for
cleanup; touchdown and automatic disarm were observed. This attempt is **not a
successful mapping mission**. A 0.214 s callback gap was also observed, so the
timing guard remains necessary.

At that historical revision, the ten-waypoint farm tour, moving LiDAR localisation, complete plantation
coverage, and timing performance on a Jetson remain unvalidated. The smoke
preset had a small launch-pad occupancy volume, not a full-farm voxel map.

At the user's request the original repository was inspected read-only. The
takeoff service request in its `flight_manager.py` was already reused; the
difference was the new navigation node's startup policy. The mapping startup
now ports `pratesting_works.py` / `missions/basic_orbit.py`: confirmed GUIDED and
arm, 0.35 s settling, one takeoff request, flight-manager hover confirmation,
then navigation. Five new tests cover accepted-once behaviour, bounded temporary
retries, hard rejection, no-climb failure and unsafe hover. ACK/progress bounds
come from the original's `simple_single_tree_mission.py`. No original file was
modified and its extra setpoint publishers are not launched.

**Actual ported-sequence flight:** the GPS-disabled ground-truth Gazebo/SITL
test accepted **one** takeoff request (`success=True`, result 0), confirmed hover,
and entered SURVEY. All four launch-pad loop viewpoints completed with newly
integrated scans; altitude reached approximately 2 m. On return the occupied/
free/unknown map lost stopping clearance as old free cells expired, and the
controller braked (`emergency_unavoidable:insufficient_stopping_clearance`).
This validates the takeoff handoff and small waypoint loop, **not** automatic
return/landing, the farm tour, or LiDAR localisation. Evidence:
`reports/original_takeoff_setup.json`, `reports/original_takeoff_smoke.json`,
and the run's saved point-cloud/voxel map. Normal operator-requested LAND was
used for cleanup, not counted as mission completion.
Measured maximum altitude was **2.004 m**, maximum sampled tracking error
**0.081 m**, with exactly one final setpoint publisher. The observer timed out
before cleanup, so its `land_seen: false` is retained; the subsequent ROS log
confirms operator-requested LAND and automatic motor disarm. A later 0.211 s
callback gap was also observed while already holding. The failure is not hidden.

Run date: 2026-10-07, Ubuntu 22.04 development host. Reports are local artifacts,
not benchmark claims for a Jetson or real plantation.

* Python safety/geometry regression suite: **19 passed**, plus **11 exploration
  tests** and **9 ROS adapter tests** (39 total). ROS tests cover visualisation
  startup, finite cloud decode, sparse-cloud freshness, IMU prediction stopping
  at scan time and visible no-pose waiting status.
* Static executable audit, Python compilation, git whitespace checks.
* Colcon package discovery and **polinasi_nav-only packaging build passed**.
* After installing Humble and the Harmonic bridge, all three selected ROS
  packages **built successfully**. Native colcon testing: **39 passed**, zero
  skipped. Earlier missing-ament failure is resolved.
* Integrated **ground-truth ROS startup check passed**: cloud, IMU, range,
  camera joint feedback, simulation clock, healthy localisation, ExternalNav
  and FCU position received; timestamps fresh and expected frames matched.
  One publisher owns each control interface. FCU vs ExternalNav difference:
  0.00013 m position and 0.00011 rad yaw while stationary. Fresh restart passed
  without manual telemetry requests. Evidence: `reports/ros_setup.json`.
  This check sent no arm/flight commands and did not validate a ROS mission.
* **Sensor-mode data flow works, but the longer stationary check failed**.
  IMU/scan ordering was corrected: newer IMU packets are buffered until matching
  scan time. A brief check initially passed; later FCU/estimator position
  difference reached **0.527 m**, exceeding the 0.25 m guard. Navigation stayed
  in `WAIT_EXTERNALNAV` with `fcu_externalnav_alignment`; no flight commands
  were sent. LiDAR/IMU-only subscriptions were verified, with no simulator truth
  input. Evidence: `reports/ros_sensor_setup.json`. This is a measured limitation,
  not validated LiDAR-based flight; do not relax the guard to hide it.
  Follow-up telemetry showed a nearly stationary scan position (z≈0.194 m)
  but estimated vertical velocity ≈−0.447 m/s. The estimator corrects position
  with ICP without correcting accumulated IMU velocity; this inconsistency
  drives FCU drift. Consistent velocity/bias estimation needs implementation
  and moving-trajectory validation before sensor-mode flight.
* Pinned `ardupilot_gazebo` CMake build and ArduCopter 4.6.3 SITL build passed.
* Actual headless Gazebo cloud/IMU/odometry messages inspected.
* Direct MAVLink SITL baseline verified GPS disabled, ExternalNav ready and
  accepted normal GUIDED/arm/takeoff/LAND commands. Maximum altitude: 2.203 m.
  Touchdown and automatic disarm passed. See `reports/sitl_takeoff_land.json`.
  This used **GROUND_TRUTH** localisation and did not test the ROS control graph.
* Five planner scenarios and one continuous-orbit case ran against a simple
  acceleration-limited point-mass plant and fully observed map fixture. Results
  are in `reports/validation.json` and `reports/continuous_orbit/validation.json`.

| Planner case | Result | Minimum surface clearance | Tracking RMSE | Mock buzzer events |
| --- | --- | --- | --- | --- |
| Clear hover viewpoints | Landing requested | 1.105 m | 0.040 m | 2 |
| Leaf blocked | 3D views/detours, landing requested | 0.954 m | 0.042 m | 2 |
| Vertical detour | Climbed over barrier, landing requested | 0.934 m | 0.042 m | 2 |
| Fully blocked | Held; `blocked_route` | 0.837 m | 0.036 m | 0 |
| Sensor dropout | Braked/held; `stale_lidar` | 1.250 m | 0.045 m | 0 |
| Continuous clear orbit | Landing requested | 0.724 m | 0.023 m | 2 |

Across these cases: commanded speed ≤0.450 m/s, acceleration ≤0.318 m/s²,
jerk ≤0.956 m/s³; executed point-mass speed ≤0.527 m/s. Configured limits are
0.600 m/s, 0.500 m/s² and 1.000 m/s³, with 0.200 m required obstacle clearance.
Maximum tracking error was 0.115 m, within the 0.200 m reserved envelope.
Clearance is measured against exact scene boxes using the full conservative
vehicle envelope, not inferred from 'no collision' or from the waypoint alone.

Plots: [vertical detour](../reports/vertical_detour.png),
[leaf blocked](../reports/leaf_blocked.png),
[continuous orbit](../reports/continuous_orbit/clear_orbit.png).

The fixture makes the whole bounded map known for isolating planning. It is
not a LiDAR-generated map and is never loaded by the ROS runtime. A completed
fixture mission means takeoff from 1.5 m airborne, inspect both trees, two mock
buzzer events, return and **request landing**. It does not imply touchdown.
The actual SITL touchdown check is a separate simple takeoff/hover/land flight.

# Sensor experiment

`benchmark.run_localisation` generates noisy LiDAR first returns and accelerometer/
gyro data. Only those sensor values enter the estimator. A known launch pose
sets the coordinate origin; truth is used afterwards for error measurement.
For the final configuration, all 80 scans were accepted; position RMSE was
0.106 m and maximum error 0.169 m over an 8-second slow traverse.

This limited result is not validated general LiDAR-inertial localisation.
The estimator lacks IMU bias estimation, complete observability testing,
motion deskew, loop closure, calibrated covariance and translational IMU
lever-arm compensation. It has only a simple planar-degeneracy/overlap gate.
Sparse leaves, repeated trees, moving foliage and fast yaw can produce incorrect
alignment. A maintained LIO implementation and dedicated recorded-data/SITL
evaluation are required before treating sensor localisation as established.

# Remaining work

1. Record a ROS bag/RViz inspection and validate moving transforms and
   commanded trajectory tracking. Stationary ROS startup and ownership passed.
2. Demonstrate the whole mission with the **live ray map**, then sensor-mode
   localisation. The MID-360 downward blind region, sparse/no-return data and
   expiring free space can intentionally stop progress. Unknown must stay blocked.
   The explicit surveyed launch-pad exception does not solve general exploration.
   Bounded airborne gap observation is now implemented and tested separately;
   see [EXPLORATION.md](EXPLORATION.md). The first live attempt took off, selected
   a view, then held; it is not successful full-flight exploration validation.
3. Profile worst-case map/planner latency on Jetson and validate actual ArduPilot
   tracking, tilted vehicle geometry, attitude/thrust limits, yaw acceleration
   and actuator response. The current envelope explicitly handles arbitrary yaw;
   the point-mass tests assume level flight and do not validate roll/pitch sweeps.
   The point-mass plant and analytic translation limits do not establish these.
4. Test sensor/noise delays, real Livox message/per-point time conventions and
   calibrated extrinsics; integrate robust LIO. ROS sensor mode exists but its
   complete flight has not been validated.
5. Replace rigid box foliage and ideal camera views with richer sensor scenes.
   ZED SDK/stereo and flower recognition are intentionally not prerequisites.
   Physical RGB replacement/pitch hardware remain optional future work.
6. Add operator-reviewed recovery policies and fuller service-ACK tests. Current
   blocked/lost-sensor policy holds and reports the cause; an unavoidable obstacle
   or autopilot estimator failsafe cannot be made safe by a position setpoint alone.

No real hardware was operated; no data was published remotely. Upstream lacks a
clear project-wide licence declaration; its retained code is not relicensed.
The new package's code is AI-assisted and should be reviewed before further use.
