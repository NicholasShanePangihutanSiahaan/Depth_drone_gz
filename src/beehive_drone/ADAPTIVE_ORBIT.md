# Adaptive orbit using observed trunk geometry

The PCL mapper publishes /map/trunk_estimates (Float64MultiArray):
[tree_id, median_observed_base_world_z, median_observed_trunk_height].
At least three accepted observations are required, with a seven-sample median.
Existing radius, height, confidence and track gates still apply.
This is visible cylinder extent, not full tree height or a verified ground base.
No new LiDAR sensor or classifier is introduced.

At START_ORBIT, the FSM locks a recent estimate (60 seconds by default) for the selected tree:
clearance = clamp(fraction * observed_height, min_height, max_height)
orbit_z = observed_base_z + clearance.
It publishes /control/orbit_local_z and waits within 0.2 m for one second,
with a fresh measured AGL >= 0.8 m, before starting orbit. Missing geometry
keeps the drone hovering until the existing state timeout aborts the mission.
The target remains fixed during orbit; the normal navigation altitude resumes
afterward. Stale orbit altitude commands freeze the last commanded setpoint.
The world and FC local coordinate origins must agree, as in the existing
simulation stack. This assumption must be resolved before real flight.

sim.yaml enables adaptive orbit; real.yaml disables it pending validation.
Parameters under mission_state_machine:
adaptive_orbit_enabled: true
adaptive_orbit_fraction: 0.5
adaptive_orbit_min_height: 1.0
adaptive_orbit_max_height: 4.0

Use simulation_mission.launch.py for full detection/approach/orbit missions.
terrain_mapping_test.launch.py intentionally flies straight and never orbits.
On hills, use the corrected optical transform and a terrain-aware base gate;
the straight mapping launch already overrides these settings. Ordinary
simulation_mission retains its original PCL configuration.

Validate visible trunks of 4 m and 7 m: expected relative orbit heights 2 m
and 3.5 m when the entire intended trunk segment is observed. Occluded/truncated
trunks yield shorter estimates; no full-height claim is made. Review collision
clearance and observation geometry before executing an orbit.

## Gazebo validation
Use adaptive_orbit_validation.launch.py instead of simulation_mission.launch.py
to land immediately after one completed orbit (land_after_orbit=True). This
launch isolates adaptive orbit from the normal return-to-home phase.
The first flight completed a full 360 degree orbit with target Z=2.03 m
and observed mean Z=2.028 m; normal return subsequently became unstable.
Do not treat that return phase as validated. See reports/adaptive_orbit_validation.
