# Local simulation setup

This computer now has ROS 2 Humble, RViz and the Gazebo Harmonic ROS bridge.
MAVROS is built in `.dependencies/mavros_ws`; the project overlay uses
`build_ros`, `install_ros` and `log_ros`, leaving earlier build artifacts intact.
The existing ROS 1 project and shell startup files are preserved.
All three prototype packages build; 124 regression tests pass. Ground-truth
startup passed. Sensor data flow works, but its estimate drifts from the FCU
estimate; the alignment guard prevents flight. A small mapping loop completed
automatic return/landing; full inspection and full-farm missions remain unvalidated.

Start the simulation in a terminal:

```bash
cd /home/abin/polinasi_lidar
bash tools/start_simulation.sh ground_truth clear_orbit
```

In another terminal:

```bash
cd /home/abin/polinasi_lidar
source tools/ros_environment.sh
.venv/bin/python tools/set_origin.py
python3 tools/check_ros_simulation.py
rviz2 -d polinasi_nav/config/navigation.rviz --ros-args -p use_sim_time:=true
```

The launcher waits for an explicit mission start:

```bash
python3 tools/start_mission.py
```

Ctrl-C in the first terminal stops its Gazebo, SITL and ROS child processes.
Their logs and fresh SITL parameters are stored in the printed `.dependencies`
run directory. Ports 9002 and 14550 must be free before starting.

For sensor localisation, replace `ground_truth` with `sensor`. It uses the
experimental LiDAR/IMU estimator. A longer stationary check failed alignment
at 0.53 m difference, so this mode is not flight-ready. See
`reports/ros_sensor_setup.json`; do not disable the alignment guard.
Choose `leaf_blocked`, `vertical_detour`, `fully_blocked`, or `sensor_dropout`
as the second argument to change the scene. The live map begins unknown; it
now tries bounded safe observation moves when an airborne mission route is
unknown, but still holds when no usable view exists. See [EXPLORATION.md](EXPLORATION.md).

Rebuild:

```bash
cd /home/abin/polinasi_lidar
bash tools/build_mavros.sh
bash tools/build_ros.sh
```

MAVROS 2.15.1 is pinned for the installed MAVLink headers. The Humble apt
repository currently lacks the MAVROS executable packages, so the source
overlay is required. The system installer refuses package removals; Ubuntu
authentication is requested through `pkexec`, not a password in chat.
All ROS/DDS traffic in these wrappers is restricted to localhost and ROS domain
86, separate from the default domain. To use a different domain, set
`POLINASI_ROS_DOMAIN_ID` identically in both terminals before sourcing. No serial
device or real autopilot connection is configured.
