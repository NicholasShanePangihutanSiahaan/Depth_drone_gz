# Installation and exact commands

Target: Ubuntu 22.04, ROS 2 Humble, Gazebo Harmonic, ArduCopter 4.6.3.
Use a development computer/VM. These commands use only loopback FCU endpoints.
ROS 2 and the Harmonic bridge are now installed on this host. For the quickest
local startup use [LOCAL_SETUP.md](LOCAL_SETUP.md).

If ROS/Gazebo apt repositories are absent, configure them:

```bash
sudo apt update
sudo apt install curl ca-certificates gnupg software-properties-common
sudo add-apt-repository -y universe
curl -fL https://github.com/ros-infrastructure/ros-apt-source/releases/download/1.3.0/ros2-apt-source_1.3.0.jammy_all.deb \
  -o /tmp/polinasi-ros2-apt-source.deb
sudo dpkg -i /tmp/polinasi-ros2-apt-source.deb
curl -fL https://packages.osrfoundation.org/gazebo.gpg -o /tmp/polinasi-gazebo.gpg
sudo install -m 0644 /tmp/polinasi-gazebo.gpg /usr/share/keyrings/polinasi-gazebo.gpg
printf 'deb [arch=%s signed-by=/usr/share/keyrings/polinasi-gazebo.gpg] https://packages.osrfoundation.org/gazebo/ubuntu-stable jammy main\n' \
  "$(dpkg --print-architecture)" | sudo tee /etc/apt/sources.list.d/polinasi-gazebo.list
sudo apt update
sudo apt install ros-humble-ros-base ros-humble-rviz2 \
  ros-humble-sensor-msgs-py ros-humble-tf2-ros \
  ros-humble-ros-gzharmonic gz-harmonic python3-colcon-common-extensions \
  python3-rosdep python3-venv python3-numpy python3-scipy python3-matplotlib \
  build-essential cmake libgz-sim8-dev libgz-sensors8-dev rapidjson-dev \
  libopencv-dev libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev \
  python3-empy python3-lxml python3-future
```

The current Humble repository lacks MAVROS executables. Build pinned source
instead (skip the clone if already present):

```bash
cd /home/abin/polinasi_lidar
mkdir -p .dependencies/mavros_ws/src
git clone --branch 2.15.1 https://github.com/mavlink/mavros.git .dependencies/mavros_ws/src/mavros
git -C .dependencies/mavros_ws/src/mavros checkout 22ae5b7cc7cdb4cb9c2070a8213c72dae445a23e
curl -fL https://github.com/ros-infrastructure/ros-apt-source/releases/download/1.3.0/ros2-apt-source_1.3.0.jammy_all.deb \
  -o .dependencies/ros2-apt-source.deb
pkexec /usr/bin/bash /home/abin/polinasi_lidar/tools/install_ros_system.sh
bash tools/build_mavros.sh
```

The installer expects this host's existing `gazebo-stable.list` OSRF source and
refuses package removals. If a fresh host has no GeographicLib dataset, install
it with `sudo bash .dependencies/mavros_ws/src/mavros/mavros/scripts/install_geographiclib_datasets.sh`
then rerun the installer. Do not send administrator passwords in chat.

Humble's default `ros-humble-ros-gz` targets Fortress; the Harmonic package above
is the OSRF alternative and conflicts with that default. Do not mix the two.
Review apt's transaction if this machine already has a ROS/Gazebo stack.
[Gazebo documents this pairing](https://gazebosim.org/docs/harmonic/ros_installation/).
The apt bootstrap package is pinned; verify availability when using a mirror.

Pinned simulator source dependencies (already downloaded locally):

```bash
cd /home/abin/polinasi_lidar
mkdir -p .dependencies
git clone https://github.com/ArduPilot/ardupilot_gazebo.git .dependencies/ardupilot_gazebo
git -C .dependencies/ardupilot_gazebo checkout 082a0fe231f6e63bc8d1598f1cba461d9e2ea7f5
cmake -S .dependencies/ardupilot_gazebo -B .dependencies/ardupilot_gazebo/build -DCMAKE_BUILD_TYPE=RelWithDebInfo
cmake --build .dependencies/ardupilot_gazebo/build -j6
git clone https://github.com/ArduPilot/ardupilot.git .dependencies/ardupilot
git -C .dependencies/ardupilot checkout 92b0cd788ec29406f26c6f9c31d5ceedbd1cc538
git -C .dependencies/ardupilot submodule update --init --recursive
cd .dependencies/ardupilot
./waf configure --board sitl
./waf copter -j6
cd /home/abin/polinasi_lidar
/usr/bin/python3 -m venv --system-site-packages .venv
.venv/bin/pip install pymavlink==2.4.49 pytest==8.4.2
```

Skip clone commands when those directories already exist; check each revision
with `git -C DIRECTORY rev-parse HEAD`. Never run `waf --upload`.
The main repository has no publishing remote. `.dependencies` is ignored by
both git and colcon; simulator source trees are independent of existing projects.

Minimal ROS build avoids detection dependencies:

```bash
cd /home/abin/polinasi_lidar
source tools/ros_environment.sh
colcon list --base-paths polinasi_nav beehive_drone uav_interfaces
bash tools/build_ros.sh
source tools/ros_environment.sh
```

The optional ZED/PCL packages remain ignored. To revisit perception later,
resolve their dependencies and deliberately remove those particular
`COLCON_IGNORE` files in this new repository. Detection is unnecessary now.

# Run simulation

Terminal 1:

```bash
cd /home/abin/polinasi_lidar
bash tools/run_gazebo.sh clear_orbit
```

Terminal 2:

```bash
cd /home/abin/polinasi_lidar
bash tools/run_sitl.sh
```

Terminal 3 (choose ONE localisation mode):

```bash
cd /home/abin/polinasi_lidar
source tools/ros_environment.sh
export GZ_PARTITION=polinasi_lidar
ros2 launch polinasi_nav sitl_navigation.launch.py localisation:=ground_truth
```

For the sensor demonstration replace `ground_truth` with `sensor`. Ground-truth
mode isolates planner behaviour. Sensor mode uses only LiDAR/IMU estimation.
Neither mode automatically uses the fully observed benchmark fixture: the live
ROS map begins unknown and is ray-mapped. Insufficient observed space can hold
or block the mission; this is intentional and must not be hidden by a free map.

Set the fixed EKF origin, inspect the graph, then start:

```bash
cd /home/abin/polinasi_lidar
.venv/bin/python tools/set_origin.py
ros2 topic echo --once /mavros/state
ros2 topic info /mavros/odometry/out -v
ros2 topic info /mavros/setpoint_position/local -v
ros2 topic echo --once /localisation/healthy
ros2 topic echo --once /navigation/status
ros2 topic pub --once /mission/start std_msgs/msg/Bool '{data: true}'
rviz2 -d polinasi_nav/config/navigation.rviz --ros-args -p use_sim_time:=true
```

`odometry/out` must have the MAVROS subscription and one ExternalNav publisher.
Final position setpoints must have exactly one producer. The origin is static
reference metadata; no GPS measurements are sent or used. GPS is disabled and
all EKF navigation sources are ExternalNav in `config/sitl.parm`. That file is
specific to pinned 4.6.3 (simulation GPS disable parameter names differ in
newer ArduPilot). Never combine old mission/vision/controller launches.

Change Terminal 1's scenario to `leaf_blocked`, `vertical_detour`,
`fully_blocked`, or `sensor_dropout`. For repeatable dropout use Terminal 3:

```bash
ros2 launch polinasi_nav sitl_navigation.launch.py localisation:=ground_truth drop_after:=25.0
```

Dropout starts 25 simulation seconds after `/mission/start`. Use an explicit
start message for this scenario; `autostart` does not trigger the sensor gate.
Stop all three terminals between scenarios. SITL persistence lives in
`.dependencies/sitl-run`; use `bash tools/run_sitl.sh sitl-run-new` when changing
startup parameters, because saved ArduPilot parameters override defaults.

Collect ROS evidence for the still-required full integration test:

```bash
ros2 bag record -o reports/ros_ground_truth /clock /livox/lidar /livox/imu \
  /localisation/odometry /localisation/healthy /mavros/local_position/pose \
  /mavros/setpoint_position/local /navigation/status /navigation/occupancy \
  /navigation/planned_path /navigation/executed_path /navigation/nominal_orbit \
  /simulation/rangefinder /actuators/mock_buzzer /camera/joint_state /tf /tf_static
```

# Independent SITL baseline check

Instead of Terminal 3, the direct diagnostic can test Gazebo/SITL/ExternalNav
without ROS. It owns UDP port 14550, so stop MAVROS first:

```bash
cd /home/abin/polinasi_lidar
OPENBLAS_NUM_THREADS=1 .venv/bin/python tools/sitl_smoke.py --seconds 35
OPENBLAS_NUM_THREADS=1 .venv/bin/python tools/sitl_smoke.py --fly --seconds 60
```

It sends **ground truth** odometry, verifies GPS-disable/EKF-source parameters,
and optionally requests simulated takeoff/hover/land with normal arming checks.
Its report labels direct MAVLink transport. This cannot validate the ROS launch
or the sensor estimator. Do not run this diagnostic concurrently with MAVROS.

# Sensor simplifications

The MID-360 specification gives 360° azimuth and −7°..52° elevation.
The model uses 360×32 simultaneous rays at 10 Hz, first returns, 12 m capped
range and simple Gaussian noise. It does not reproduce Livox's nonrepeating
pattern, 200k points/s, reflectivity-dependent range, multiple returns, UDP
packet timing, per-point offsets or motion distortion. IMU is 200 Hz, colocated
with LiDAR, without calibrated bias/drift. Small negative elevation leaves a
substantial downward blind region; the narrow rangefinder does not observe an
entire flight corridor. No-return NaNs do not clear unknown cells.

The 0.65×0.65×0.50 m default envelope covers the Iris plus the LiDAR mount,
standoffs and payload; replace it with measured dimensions for any later model.
The initial 2.8 m square launch pad is an explicit flat, surveyed-clear prior,
updated only with a fresh downward range in startup/takeoff. Ground remains an
obstacle outside the contact/takeoff exception. The standalone benchmark starts
at 1.5 m airborne and ends at a landing request; it does not simulate touchdown.
Foliage is rigid opaque boxes; the RGB placeholder models neither real flower
classification nor ZED depth. Pitch is optional and actual joint feedback drives
the camera TF. These simplifications must stay attached to any demonstration.

Source: [Livox MID-360 specification](https://www.livoxtech.com/mid-360/specs),
[ArduPilot Gazebo setup](https://ardupilot.org/dev/docs/sitl-with-gazebo.html),
[ArduPilot non-GPS estimation](https://ardupilot.org/dev/docs/mavlink-nongps-position-estimation.html).
