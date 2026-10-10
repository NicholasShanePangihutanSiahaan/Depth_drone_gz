# polinasi_lidar

Pada branch `lidar360-PID`, misi mapping memakai PID feedback PVA, bukan MPC.
Gunakan `bash tools/start_pid.sh tour` (326 target) atau `tour_check` (4 target).
Lihat [panduan PID](docs/PID_CONTROL.md). Snapshot MPC ada di `lidar360-mpc`;
hasil uji MPC tidak otomatis berlaku untuk PID.
Hasil sementara: enam target PID dilewati tanpa HOLD_ABORT; lihat
[hasil dan batas pengujian PID](docs/PID_RESULTS.md). Seluruh kebun belum tervalidasi.

Separate simulation prototype, based on upstream `151a6b3b8b916dada056db9f0f212036192c348b`.
The original `/home/abin/p0l1n4s1` is untouched. Branches are published to
`NicholasShanePangihutanSiahaan/Depth_drone_gz`; its default branch is untouched.

Installed: ROS 2 Humble, RViz, Gazebo Harmonic bridge and source-built MAVROS.
Latest selected tests: 177 unique navigation/ROS-adapter cases passed; legacy
beehive helper tests have an unresolved import mismatch (see PID results).
Historical tests include six planner cases, four partial-map exploration cases,
and GPS-disabled Gazebo/SITL
takeoff–survey out to 4 m–return–automatic landing using ground truth.
Full sensor-based flight remains unvalidated.
Integrated ground-truth ROS startup/data-flow check passed. Sensor-only data flow
works, but a longer stationary check failed FCU/estimator alignment (0.53 m);
its safety guard blocks flight. A later ROS exploration attempt took off and
selected a view, then held; full mission completion is still unvalidated.

Start locally (no automatic arming):

```bash
cd /home/abin/polinasi_lidar
bash tools/start_simulation.sh ground_truth clear_orbit
```

See [local setup](docs/LOCAL_SETUP.md) for the second-terminal checks and RViz.

For the separate mapping-only mission (no inspection or buzzer), run
`bash tools/start_mapping.sh smoke ground_truth` for a small launch-pad loop,
or replace `smoke` with `tour` for the configured farm route.
Follow [petunjuk survei dalam Bahasa Indonesia](docs/SURVEI_KEBUN.md) or
[mapping mission instructions](docs/MAPPING_MISSION.md) for readiness,
explicit mission start, and saved point-cloud/voxel maps. Full farm-tour
completion and sensor-based flight are not validated.

Mapping also implements safe continuous quintic chunks, position/velocity/
acceleration MAVROS feedforward and adaptive speed. A new GPS-disabled
ground-truth smoke flight completed all four goals and automatic landing,
with 1.01 cm maximum sampled tracking error. Higher-speed `check`/`tour` flight
is not yet validated. See [lintasan kontinu](docs/LINTASAN_KONTINU.md).

Mapping planning and RViz message construction now use isolated processes,
with callback timing diagnostics and process-group cleanup. An interrupted
initial tour snapshot reached 130 simulation seconds and completed the first
goal without a recorded timing abort; full-tour completion is still unvalidated.
Default pacing is restored to 0.2. See [runtime isolation](docs/RUNTIME_ISOLATION.md).

Separate simulation-only system-identification missions and a guarded online
RLS model observer are available. The adaptive model is **shadow-only**, not
NMPC or automatic PID tuning. See [identifikasi dan model adaptif](docs/IDENTIFICATION_ADAPTIVE_MODEL.md)
for train/validation runs, parameter snapshots and candidate-model checks.
Two empty-arena ground-truth identification flights completed; a calibrated
simulation observer seed passed the configured 5 cm prediction gate. Vertical
validation still has approximately 3.8 cm command/response offset, so this is
not approval for active NMPC/hardware. See [measured identification results](docs/IDENTIFICATION_RESULTS.md).

Opt-in reduced-order PVA MPC now actively selects acceleration feedforward.
An empty-arena ground-truth flight completed with 858 MPC applications and
7.34 mm maximum sampled tracking error. This is not full rotor-model NMPC,
an A/B improvement result, adaptive-flight validation, or orchard validation.
See [pengendali prediktif aktif](docs/PREDICTIVE_CONTROL.md).

Implemented: 3D occupied/free/unknown map, drone inflation, exact swept-segment
checks, 3D detours, speed/acceleration/jerk-limited trajectories, checked braking,
replanning, tree-facing orbit or hover views, mock flower confirmations/buzzer,
and exclusive truth/sensor localisation modes. The new ROS launch reuses the
original flight manager and bypasses vortex and the old setpoint adapter.
Active gap observation now tries safe nearby viewpoints before giving up:
move in known-free space → observe → retry the mission. It never flies into
unknown space. See [exploration behaviour and tests](docs/EXPLORATION.md).

Run the tested standalone prototype:

```bash
cd /home/abin/polinasi_lidar
OPENBLAS_NUM_THREADS=1 PYTHONPATH=polinasi_nav /usr/bin/python3 \
  -m unittest discover -s polinasi_nav/test -v
OPENBLAS_NUM_THREADS=1 PYTHONPATH=polinasi_nav /usr/bin/python3 \
  -m polinasi_nav.benchmark --scenario all
OPENBLAS_NUM_THREADS=1 PYTHONPATH=polinasi_nav /usr/bin/python3 \
  -m polinasi_nav.benchmark --scenario clear_orbit --inspection-mode orbit \
  --output reports/continuous_orbit
```

Requires NumPy, SciPy and Matplotlib (`sudo apt install python3-numpy python3-scipy
python3-matplotlib`). Reports contain plots, CSV paths and measured clearance,
tracking errors, commanded derivatives and failures. Planner tests use an
explicit fully observed ground-truth map fixture and a point-mass plant.
They do not prove live LiDAR mapping, ArduPilot tracking, or LiDAR localisation.

The separate sensor experiment uses laser returns and IMU only. Its experimental
ICP estimator aligns scans to earlier scans; it is not production FAST-LIO.
The ROS sensor mode never subscribes to simulator ground-truth odometry.

Read [installation and launch commands](docs/SIMULATION.md),
[audit and architecture](docs/AUDIT_AND_ARCHITECTURE.md), and
[implemented work, tests and limitations](docs/VALIDATION.md).
Tune [navigation.json](polinasi_nav/config/navigation.json); regenerate worlds
with `python3 tools/generate_worlds.py` after changing tree geometry.

Terms: a *voxel* is a small 3D map cell; *inflation* enlarges obstacles to allow
for the drone's size; *ExternalNav* supplies an outside position estimate to
ArduPilot; *SITL* runs autopilot software against simulated physics.

No real hardware operation or remote publication is included.
The inherited deployment instructions remain in [ReadME.md](ReadME.md).
# Saved 3D map

Simulation runs now save an offline, rotatable point-cloud and voxel-grid map.
New runs use [20 cm navigation/log voxels](docs/VOXEL_RESOLUTION.md).
The launcher prints the `map.html` location. See [3D map log](docs/3D_MAP_LOG.md).

For detailed palm trees and coloured farm ground, use the separate
[`palm_farm` scene](docs/PALM_FARM.md):
`bash tools/start_simulation.sh ground_truth palm_farm`, then
`bash tools/view_simulation.sh` in another terminal.

# Kontrol default pada branch lidar360-PID

Misi mapping sekarang memakai PID feedback PVA, bukan MPC. Jalankan
`bash tools/start_pid.sh tour` atau `bash tools/start_mapping.sh tour`.
Untuk uji singkat, gunakan `bash tools/start_pid.sh tour_check`.
Lihat [panduan PID](docs/PID_CONTROL.md). Snapshot MPC tersedia pada branch
`lidar360-mpc`; hasil validasinya tidak otomatis berlaku untuk PID.

