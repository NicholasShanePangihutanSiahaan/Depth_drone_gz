# Cara Menjalankan Gazebo Revisi — Misi Satu Pohon

Versi ini memakai inti program revisi real, tetapi dikonfigurasi untuk simulasi lama yang sudah pernah berhasil:

- frame misi: `map`
- frame hasil PCL: `plantation`
- point cloud: `/zed2i/depth/points`
- odometry: `/mavros/odometry/out`
- approach dan orbit sama-sama 3 meter
- alur: arm → takeoff → hover → cari satu pohon → approach → hover → orbit 360° → hover → kembali ke titik sebelum orbit → home → hover → land
- pergantian mode dari `GUIDED` mengunci `PILOT_OVERRIDE`

## 1. Salin ke workspace

Ekstrak seluruh isi ZIP ke workspace, misalnya:

```bash
mkdir -p ~/gazebo_revised_ws/src
cd ~/gazebo_revised_ws/src
unzip ~/Downloads/gazebo_revised_one_tree.zip
```

## 2. Build bersih

```bash
cd ~/gazebo_revised_ws
source /opt/ros/humble/setup.bash
rm -rf build install log
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

Jika paket diekstrak langsung ke root workspace dan bukan `src`, jalankan `colcon list` terlebih dahulu. Package harus terlihat.

## 3. Jalankan Gazebo

Terminal 1:

```bash
source /opt/ros/humble/setup.bash
source ~/gazebo_revised_ws/install/setup.bash
ros2 launch uav_plantation_sim plantation_sim.launch.py
```

Tunggu world dan drone selesai dimuat.

## 4. Jalankan ArduPilot SITL

Terminal 2, sesuaikan lokasi source ArduPilot:

```bash
cd ~/ardupilot/ArduCopter
sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON --console --out=udp:127.0.0.1:14551
```

Gunakan command SITL lama Anda bila konfigurasi yang pernah berhasil berbeda.

## 5. Jalankan MAVROS

Terminal 3:

```bash
source /opt/ros/humble/setup.bash
source ~/gazebo_revised_ws/install/setup.bash
ros2 run mavros mavros_node --ros-args -p fcu_url:=udp://127.0.0.1:14551@
```

Periksa:

```bash
ros2 topic echo /mavros/state --once
```

Harus `connected: true`.

## 6. Periksa data simulasi sebelum misi

```bash
ros2 topic hz /zed2i/depth/points
ros2 topic hz /mavros/odometry/out
ros2 topic hz /mavros/local_position/pose
```

Semua harus aktif.

## 7. Uji pertama: takeoff dan hover saja

Terminal 4:

```bash
source /opt/ros/humble/setup.bash
source ~/gazebo_revised_ws/install/setup.bash

ros2 launch beehive_drone mission_gazebo_revised.launch.py \
  hold_after_takeoff:=true \
  use_pcl:=true \
  use_analyzer:=true \
  point_cloud_topic:=/zed2i/depth/points \
  odom_topic:=/mavros/odometry/out
```

Target log:

```text
WAIT_CONNECTION -> SET_MODE
SET_MODE -> ARM
ARM -> TAKEOFF
Perintah takeoff diterima autopilot; MAV_RESULT=0
TAKEOFF -> HOLD
```

Hentikan dengan `Ctrl+C` setelah hover terbukti stabil.

## 8. Periksa PCL

Saat launch masih aktif:

```bash
ros2 node list | grep -E 'pcl_proc_node|pcl_tree_mapper'
ros2 topic hz /perception/pcl/tracked_cylinders
ros2 topic echo /map/tree_count
ros2 topic echo /map/trees_ready
```

Kondisi siap:

```text
/map/tree_count >= 1
/map/trees_ready = true
```

## 9. Jalankan misi lengkap

Setelah takeoff/hover dan PCL berhasil:

```bash
ros2 launch beehive_drone mission_gazebo_revised.launch.py \
  hold_after_takeoff:=false \
  use_pcl:=true \
  use_analyzer:=true \
  point_cloud_topic:=/zed2i/depth/points \
  odom_topic:=/mavros/odometry/out
```

Pantau:

```bash
ros2 topic echo /mission/fsm_state
ros2 topic echo /map/trees_ready
ros2 topic echo /mission/pilot_override
```

Urutan state yang diharapkan:

```text
WAIT_CONNECTION
SET_MODE
ARM
TAKEOFF
WAIT_TAKEOFF_HOVER
SEARCH_TREE
APPROACH_TREE
HOVER_BEFORE_ORBIT
PREPARE_ORBIT
WAIT_ORBIT
HOVER_AFTER_ORBIT
RETURN_PRE_ORBIT
HOVER_RETURN_POINT
RETURN_HOME
HOME_HOVER
LAND
WAIT_LANDED
DONE
```

## 10. Uji takeover dalam SITL

Ketika drone sedang menjalankan misi, ubah flight mode dari `GUIDED` ke `LOITER` atau `ALT_HOLD` melalui Mission Planner/console. Target:

```text
/mission/fsm_state = PILOT_OVERRIDE
/mission/pilot_override = true
/mission/autonomy_enabled = false
```

Misi tidak otomatis aktif kembali walau mode dikembalikan ke `GUIDED`; restart launch untuk misi baru.

## Catatan

- Tidak perlu `vision_to_mavros` di Gazebo karena SITL menyediakan local position langsung.
- Jangan memakai topik ZED real `/zed/zed_node/...`; versi simulasi memakai `/zed2i/depth/points`.
- `approach_distance` dan `orbit_radius` sudah sama-sama 3 meter agar FSM tidak bolak-balik sebelum orbit.
- Hasil analyzer disimpan di `~/beehive_mission_results_gazebo`.
