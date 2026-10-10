# Active gap observation

Implemented in the existing simulation mission, enabled by default. No new
MAVROS control publisher is added. The original project is unchanged.

When inspection/return planning is blocked:

1. Select a nearby **observed-free** drone position that could reveal unknown
   cells. Score LiDAR coverage, sampled obstacle occlusion, travel and progress
   toward the original mission goal. Use the actual configured LiDAR mounting
   transform and −7°..52° / 360° coverage.
2. Enter `EXPLORE/MOVE`, using a checked, slow trajectory with braking reserve.
3. At rest, enter `EXPLORE/OBSERVE`. Wait at least 0.8 seconds and for a newly
   integrated measurement after arrival, then retry the original mission leg.
   Tree/view indices do not advance and mock flower events cannot fire here.
4. If no usable view exists, budgets expire, localisation fails, or clouds stop,
   brake/hold. Unknown is **never** converted to free by viewpoint selection.

Movement is limited to directly reachable local free-space hops. This is not
whole-world exploration or a guarantee of escaping every blind spot. It starts
only when already airborne; takeoff still needs a confirmed safe launch area.

Configuration in `polinasi_nav/config/navigation.json`:

| Setting | Default | Meaning |
| --- | --- | --- |
| `exploration_enabled` | true | Set false to restore passive blocked-route behaviour. |
| `exploration_speed` | 0.20 m/s | Bounded exploratory speed, not an added slew limiter. |
| `exploration_radius` / `exploration_look_range` | 3 / 6 m | Local move radius / information-scoring range. |
| `exploration_min_altitude` / `exploration_max_altitude` | 1.0 / 4.2 m | Allowed ENU viewpoint heights; collision checks still apply. |
| `exploration_max_views` / `exploration_timeout` | 8 / 120 s | Visit budget per mission leg / stalled exploration episode timeout. |
| `exploration_observe_seconds` | 0.8 s | Settled dwell, also requiring a new integrated scan. |
| `exploration_revisit_distance` | 0.45 m | Exclude already attempted view neighbourhoods. |

The prediction uses a level-body, yaw-aware sensor model and coarse rays, so
gain is an estimate, not proof of visibility. It never replaces exact vehicle
envelope/trajectory/braking checks. Invalid infinite returns remain ignored;
they are not invented free-space evidence. Fresh sparse packets now count as
sensor activity, separately from map processing age. Even one valid finite ray
can update its actual observed cells; the rest remain unknown and free cells
still expire. LiDAR-localisation's minimum-return quality gate is unchanged.

# Run and inspect

```bash
cd /home/abin/polinasi_lidar
bash tools/build_ros.sh
bash tools/start_simulation.sh ground_truth clear_orbit
```

Another terminal:

```bash
cd /home/abin/polinasi_lidar
source tools/ros_environment.sh
.venv/bin/python tools/set_origin.py
python3 tools/watch_exploration.py --seconds 90
```

The origin helper now retries until SITL confirms the reference, instead of
silently succeeding when an early message is ignored. No GPS measurements are
sent. Start the mission in a third terminal with the same environment:

```bash
ros2 topic pub --once /mission/start std_msgs/msg/Bool '{data: true}'
```

Watch `/navigation/status` for mission/phase, exploration view count, predicted
gain, unknown-cell count, packet age and map-measurement age. RViz adds purple
exploration viewpoint markers to the existing cloud/map/planned/executed paths.
They show selection order, **not a collision-certified connecting route**;
only `/navigation/planned_path` shows the active flight trajectory.

# Evidence and limits

Run the standalone analytic-LiDAR experiment:

```bash
OPENBLAS_NUM_THREADS=1 PYTHONPATH=polinasi_nav .venv/bin/python \
  -m polinasi_nav.exploration_benchmark
```

Its initial map is a surveyed fixture **except a withheld unknown gap**. The
gap changes only through simulated first-return ray measurements once an
observation move is selected. It starts airborne, uses ground-truth position
and simplified motion, and retains surveyed cells for 120 seconds. It is not
an initially unmapped Gazebo environment or LiDAR-localisation validation.
Results: `reports/exploration/validation.json`, per-case JSON/CSV/PNG.

* Observable gap: 240 unknown cells → 0; one observation move; original first
  inspection goal reached. Minimum clearance 1.076 m; tracking RMSE 0.025 m.
* Sealed gap: no useful reachable view, held with `blocked_route`.
* Downward blind gap: 320 unknown cells → 37; one view, then held. The remaining
  unknown cells were not waived to manufacture a successful flight.
* Dropout during the move: braked/held with `stale_lidar`.

The first live ground-truth Gazebo/SITL attempt took off and selected one view,
but held on a LiDAR-health fault before useful exploration was completed.
Logs: `.dependencies/ros_ground_truth_clear_orbit_20261007_092922` and
`reports/exploration/live_after_origin.json`. It was commanded to LAND after
the test. The sparse-packet/map-age fix is regression-tested; a complete live
gap-recovery flight after that fix is still pending. The initial live recorder
missed the moving phase, so it must not be treated as tracking evidence for a
successful exploration move. Sensor-mode velocity inconsistency remains
unresolved; do not disable the alignment guard or use real hardware.
