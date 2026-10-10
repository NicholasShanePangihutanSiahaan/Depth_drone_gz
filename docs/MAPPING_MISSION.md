# Separate mapping flight

This program attempts **takeoff → survey waypoints → return home → descend →
land** in the palm farm, while accumulating real-time simulated LiDAR points
and a 20 cm occupancy grid. It does not inspect trees or trigger the buzzer.
It is a separate executable/controller, not a second publisher running beside
the inspection controller. Ground-truth and sensor-only localisation remain
separate modes; start with ground truth. Sensor localisation is not flight-ready.

## Run

Stop any previous simulation before launching this one. This computer's package
is built; if changing code, run `bash tools/build_ros.sh` first.

Terminal 1 — full configured farm tour:

```bash
cd /home/abin/polinasi_lidar
bash tools/start_mapping.sh tour ground_truth
```

For an initial **small** launch-pad loop, use `smoke` instead of `tour`.
That tests the flight sequence, not full farm coverage.
All presets now archive the same 92 × 82 × 10.8 m farm bounds, with 20 cm
voxels. Navigation uses a rolling 10 × 10 × 8 m window; historical observations
remain in the global archive after the window moves. `check` visits four points
out to 4 m from home and back; `smoke` remains a tiny flight-sequence check.
Neither short route surveys the whole farm.

Mapping runs use a run-local 20 m LiDAR range, unchanged 360°/−7°..52° coverage,
and a requested real-time factor 0.2 through `start_mapping.sh`. A 0.3 smoke
flight passed but subsequent tour flights held on timing gaps, so the default
was restored to 0.2. One simulation second targets five wall seconds; CPU/GPU load may reduce the
achieved factor. `POLINASI_SIM_RTF=0.2 bash tools/start_mapping.sh tour ground_truth`
restores conservative pacing without weakening safety checks. The 1 ms physics
step, sensor sample rates and drone speed limits are unchanged.
Mapping planning and RViz message construction now use isolated spawned
processes, with callback timing diagnostics: [runtime isolation](RUNTIME_ISOLATION.md).
The physics step stays 1 ms. At 2.2 m sensor height, the shallow downward angle
needs approximately 18 m range to see level ground. This is still a simplified
ray sensor, not an exact Livox acquisition model. Original model files stay intact.

Terminal 2 — wait for the fixed reference to be confirmed and check data flow:

```bash
cd /home/abin/polinasi_lidar
source tools/ros_environment.sh
.venv/bin/python tools/set_origin.py --timeout 120
python3 tools/check_ros_simulation.py --output reports/mapping_setup.json
```

The reference helper confirms both the estimator origin and ArduPilot home,
using fixed simulator metadata while disarmed. Neither is a GPS measurement.
It requires repeated live confirmations across three FCU/simulation seconds;
an early origin ACK alone can be erased by EKF core initialisation.
The mapping takeoff altitude is relative to that home at world Z=0, not the
initial height of the drone's body centre.

Only after the check passes, start the simulated mission:

```bash
python3 tools/start_mission.py
```

The ordinary GUIDED/arm/takeoff services and normal ArduPilot arming checks
are retained. No force-arm, GPS navigation, or real hardware endpoints are used.
Takeoff now follows the original repository's `pratesting_works.py` and
`missions/basic_orbit.py`: confirm GUIDED and armed status, settle for 0.35 s,
send the requested height once, and wait for flight-manager hover telemetry.
Only after observed clearance and low speed are also confirmed for 2 s does
navigation start sending position targets. The original `flight_manager`
service path is retained, with result-code feedback added. Accepted takeoffs
are not repeatedly sent; temporary rejection/timeouts permit at most three
attempts. Hard rejection or accepted-but-no-climb is reported explicitly.
The helper retries the start request until the running mission acknowledges it;
acknowledgement is not proof of takeoff or completion.

Terminal 3 — visualise the scene or sensor data:

```bash
cd /home/abin/polinasi_lidar
bash tools/view_simulation.sh
```

RViz alternative, in a sourced terminal:

```bash
source tools/ros_environment.sh
rviz2 -d polinasi_nav/config/navigation.rviz --ros-args -p use_sim_time:=true
```

Yellow survey waypoints show the intended tour, **not a certified flight
trajectory**. Cyan planned paths are collision-checked; green paths show motion.
For read-only flight evidence, start this before the mission in another terminal:

```bash
source tools/ros_environment.sh
python3 tools/watch_mapping.py --seconds 600 --output reports/mapping_live.json
```

The tour can take substantially longer than 600 wall-clock seconds when the
simulator runs slowly. The observer timing limit does not stop the mission.

Wait for `mission: COMPLETE`, with `armed: false`, before stopping the simulator.
Ctrl+C in Terminal 1 then stops its own processes. The launcher prints the new
`reports/..._mapping/map_3d/` folder. Open **map.html** to rotate/zoom the accumulated
point cloud and voxel map. **mission_status.json** records the latest mission
state with the map; **map.json** includes both geometry and that status. This
is not a rosbag recording. Unknown voxels remain unknown, not empty.

## Behaviour and limits

Verified on this computer before the continuous/PVA change: ground-truth `check` completed takeoff,
all four goals out to 4 m and back, return, automatic landing and disarm.
Maximum altitude was 2.305 m including an extra observation view; maximum
sampled tracking error was 0.098 m. This successful run used the new 20 m range
and 0.2 m/s command speed. The local window moved while retaining global history.
The full farm tour and sensor localisation remain unvalidated.
See [validation](VALIDATION.md) and [Bahasa Indonesia instructions](SURVEI_KEBUN.md).

- `mapping_tour.json` defines 326 short goals on zigzag lanes at 2 m height,
  spaced at most 2 m apart, with a nominal route length about 569 m. The
  buffered convex tree footprint (4.5 m edge margin) excludes empty outer
  scene borders. Tree layout guides goals, never free-space evidence.
  Trunk positions guide nominal goals only; they never populate the live map.
  Current command-speed cap is 0.3 m/s (`smoke` stays 0.2). All return and land; actual full-tour tracking remains
  unvalidated. A 0.4 m/s check trial hit the existing 25 cm tracking-error guard.
- Free observations are retained for 10,800 simulation seconds in short tests
  and 21,600 seconds in the long tour, only under the
  explicitly static simulated-farm assumption. This is not a safe policy for
  moving obstacles or real hardware. Occupied cells stay obstacles, unobserved
  cells stay unknown, and stale LiDAR/localisation still stop flight. Navigation
  also rejects integrated maps older than 1 s after takeoff handoff.
- The long-tour duration budget is 18,000 simulation seconds. A stop-to-stop
  trajectory takes longer than distance divided by peak speed; the nominal
  budget is regression-checked. This does not prove the complete tour finishes.
- Continuous mapping joins up to four currently safe goals using bounded C2
  quintic curves, publishing position/velocity/acceleration on the sole raw-local
  MAVROS interface. Interior points count after entering their pass radius and
  receiving a newly **integrated** LiDAR observation; chunk endpoints still
  settle/dwell. Adaptive phase scaling slows turns, tracking lag and proximity
  to occupied/unknown space. It is not NMPC. See [details](LINTASAN_KONTINU.md).
- Routes use the actual live occupancy map. Blocked/unknown routes can trigger
  bounded, safe observation moves; no whole-map ground-truth free-space fixture
  is injected. Local exploration does not guarantee complete farm discovery.
- Planning and trajectory certification run on an isolated worker snapshot
  while holding at rest. Results are checked against the current map, and each
  commanded point still passes collision/braking checks. The inspection mission
  keeps its original synchronous planner.
- Mapping and HTML/JSON export run in separate worker processes so their
  computation does not occupy the flight-control callback. This is not a
  guarantee of real-time performance on every computer.
- The mapping launch explicitly configures MAVROS plugin parameters and waits
  for acknowledgements before arming. This MAVROS build ignores global launch
  parameters for plugin nodes. A fixed, stationary-calibrated numerical
  transform connects `map` to `fcu_local`; commands and comparisons are actually
  transformed, not merely relabelled. The transform cannot follow or hide drift.
- Duplicate callbacks with an unchanged simulation timestamp are skipped;
  backwards time or gaps above 0.2 simulation seconds still trigger a safety
  hold. Status now reports the exact last gap. This does **not** prove all timing
  problems are solved.
- LiDAR/localisation loss, tracking error, or a blocked route can stop the tour
  before landing. It never blindly returns through unknown space or descends
  at an unsurveyed location just to label a run complete. Do not disable guards.

Edit `polinasi_nav/config/mapping_tour.json` to change the route. Changing targets
does not certify them as safe. `tools/generate_mapping_configs.py` recreates the
three presets and intentionally replaces edits to those generated files.
