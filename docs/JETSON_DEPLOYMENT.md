# Deployment Jetson — Beehive Drone

Repository ini harus diletakkan sebagai workspace ROS 2, dengan package
`beehive_drone`, `point-cloud-test`, `pcl_cstm_msg`, dan `uav_interfaces` di
root repository.

## Build pertama

```bash
cd ~/polinasi
source /opt/ros/humble/setup.bash
rosdep install --from-paths . --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

## Urutan menjalankan drone nyata

1. Jalankan MAVROS yang terhubung ke Pixhawk.
2. Jalankan ZED wrapper sampai `/zed/zed_node/pose` dan
   `/zed/zed_node/point_cloud/cloud_registered` aktif.
3. Jalankan mission:

```bash
source /opt/ros/humble/setup.bash
source ~/polinasi/install/setup.bash
ros2 launch beehive_drone real_mission.launch.py
```

`real_mission.launch.py` otomatis menjalankan `vision_to_mavros`, PCL,
tree mapper, controller, safety monitor, mission analyzer, dan FSM. MAVROS dan
ZED wrapper tetap harus dijalankan lebih dahulu.

Periksa sebelum start:

```bash
ros2 topic echo --once /mavros/state
ros2 topic hz /mavros/vision_pose/pose
ros2 topic hz /mavros/local_position/pose
ros2 topic echo --once /mission/safety_ok
ros2 topic echo --once /mission/safety_reason
```

Mulai misi hanya jika seluruh pemeriksaan sehat:

```bash
ros2 topic pub --once /mission/start std_msgs/msg/Bool "{data: true}"
```

Manual lengkap tersedia di `docs/manual_operasional_drone_real.pdf`.

Program uji `odom_tester.py`, `pratesting_works.py`, dan
`tes_kiri_kanan.py` dipertahankan, tetapi tidak dijalankan oleh launch misi.
