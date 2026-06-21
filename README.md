# gazebo_sim / uav_plantation_sim

ROS 2 Humble + Gazebo Sim package for UAV perception and navigation tests in an oil palm plantation environment.

The GitHub repository can be named `gazebo_sim`; the ROS 2 package inside it is named `uav_plantation_sim`.

## Contents

- `worlds/plantation.sdf`: SDF 1.9 world with Gazebo Harmonic system plugins, a 60 x 60 m textured ground area, oil palm rows, static obstacles, and directional sunlight with shadows.
- `models/plantation_quadrotor/model.sdf`: quadcopter model with `MulticopterMotorModel`, `MulticopterVelocityControl`, IMU, ZED 2i-style stereo cameras, RGB-D depth camera, and downward 1D GPU lidar.
- `config/bridge_config.yaml`: `ros_gz_bridge` mappings for IMU, camera images, camera info, depth, lidar/range, and `/cmd_vel`.
- `launch/plantation_sim.launch.py`: starts Gazebo, starts the bridge, and spawns the drone at `(0, 0, 1)`.

## Install From GitHub

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/NicholasShanePangihutanSiahaan/Depth_drone_gz.git gazebo_sim
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select uav_plantation_sim
source install/setup.bash
```

Required runtime stack:

- ROS 2 Humble
- Gazebo Sim 8.11.0 or compatible Gazebo Sim / Harmonic installation
- `ros_gz_sim`
- `ros_gz_bridge`

## Run

```bash
ros2 launch uav_plantation_sim plantation_sim.launch.py
```

Test velocity control:

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.4}, angular: {z: 0.0}}" -r 10
```

Useful ROS 2 topics:

- `/imu`
- `/zed2i/left/image_rect_color`
- `/zed2i/left/camera_info`
- `/zed2i/right/image_rect_color`
- `/zed2i/right/camera_info`
- `/zed2i/depth/depth_registered`
- `/zed2i/depth/camera_info`
- `/range`
- `/cmd_vel`

## ArduPilot `sim_vehicle.py` Status

The current drone does not connect to `sim_vehicle.py` yet. It uses Gazebo's `MulticopterVelocityControl` system and accepts ROS 2 velocity commands on `/cmd_vel`.

This is useful for early camera, depth, detection, SLAM, and navigation algorithm validation. To fly the same model from ArduPilot SITL, add an ArduPilot Gazebo/MAVLink integration path that maps SITL motor outputs to the four rotor joints and feeds IMU/state data back to ArduPilot. After that, the launch flow can be extended to start Gazebo and `sim_vehicle.py` together.
