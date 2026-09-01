# Pengujian stack dunia nyata di Gazebo

Launch `real_stack_sim.launch.py` menjalankan program ROS yang sama dengan
penerbangan nyata setelah mengganti perangkat keras ZED2i dan rangefinder dengan
adapter Gazebo:

```text
Gazebo ground truth -> raw ZED pose -> zed_frame_alignment -> vision_to_mavros
Gazebo tree truth   -> ZED ObjectsStamped -> bb_pcl_proc_node
Gazebo LaserScan    -> /mavros/rangefinder/rangefinder
                                      |
                                      v
                            real_mission.launch.py
```

Jalur ini sengaja **tidak** menjalankan `pcl_proc_node`,
`simulation_mission.launch.py`, atau `sim_external_odometry`. Hanya satu
publisher boleh ada pada `/global_cylinders` dan hanya satu sumber ExternalNav
boleh dikirim ke MAVROS.

Adapter object detection membuktikan kontrak pesan, sinkronisasi, transformasi
kamera, mapping, pemilihan pohon, approach, orbit, dan landing. Adapter tidak
membuktikan akurasi model neural-network ZED terhadap citra dunia nyata; bagian
itu tetap harus diuji dengan ZED2i dan dataset/rosbag yang merekam
`/zed/zed_node/obj_det/objects`.

## Build

Plugin `vision_pose` berada di `mavros_extras`, bukan paket inti MAVROS. Tanpa
plugin ini topic vision tetap terlihat aktif, tetapi tidak ada pesan yang
dikirim ke flight controller. Pasang dependency berikut pada companion
computer dan komputer simulasi:

```bash
sudo apt update
sudo apt install ros-humble-mavros ros-humble-mavros-extras
```

```bash
cd /home/shane/ProjekAtaka/gazebo_sim
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select \
  uav_interfaces pcl_cstm_msg point-cloud-test beehive_drone \
  uav_plantation_sim
source install/setup.bash
```

## Baseline flat-world

Gunakan empat terminal. Jangan jalankan stack simulasi lama bersamaan.

Terminal 1 — Gazebo:

```bash
cd /home/shane/ProjekAtaka/gazebo_sim
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch uav_plantation_sim plantation_sim.launch.py \
  world:=plantation.sdf
```

Terminal 2 — ArduPilot SITL. Profil ini memakai AHRS simulasi untuk memisahkan
bug program ROS dari tuning EKF:

```bash
cd /home/shane/ProjekAtaka/gazebo_sim
sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON -w \
  --add-param-file=$PWD/src/beehive_drone/config/sitl_sim.parm \
  --console --out=udp:127.0.0.1:14551
```

Terminal 3 — MAVROS:

```bash
source /opt/ros/humble/setup.bash
source /home/shane/ProjekAtaka/gazebo_sim/install/setup.bash
ros2 launch mavros apm.launch fcu_url:=udp://127.0.0.1:14551@
```

Pastikan plugin vision benar-benar termuat:

```bash
ros2 node list | rg '^/mavros/vision_pose$'
ros2 topic info /mavros/vision_pose/pose -v
```

Terminal 4 — adapter perangkat keras dan stack real:

```bash
source /opt/ros/humble/setup.bash
source /home/shane/ProjekAtaka/gazebo_sim/install/setup.bash
ros2 launch beehive_drone real_stack_sim.launch.py auto_start:=false
```

Alignment mengambil 50 pasangan pose ketika kendaraan diam dan disarmed,
menghitung `yaw_FC - yaw_ZED`, lalu mengunci rotasi dan translasi untuk satu
sesi. Pastikan ini bernilai `true` sebelum start:

```bash
ros2 topic echo --once /alignment/ready
ros2 topic echo --once /alignment/yaw_offset_deg
```

Pose yang dikirim ke MAVROS dan pose yang dipakai mapper sama-sama berasal dari
`/zed/aligned_pose`; jangan campur landmark raw-ZED dengan local pose FC.

Khusus launch simulasi, yaw offset dikunci `0 deg`. ZED sintetis berasal dari
ground truth Gazebo dan sudah memakai sumbu world yang benar; offset `+/-90 deg`
dari AHRS_SIM adalah konversi ENU/NED SITL, bukan kesalahan pemasangan kamera.
Auto-calibration tetap digunakan oleh `vision_to_mavros.launch.py` pada drone
nyata.

Tunggu validator menyatakan siap:

```bash
ros2 topic echo --once /simulation/real_stack/ready
ros2 topic echo --once /simulation/real_stack/status
```

Nilai `ready.data` harus `true`, `failures` harus kosong,
`mavros_vision_plugin_present` harus `true`, dan publisher
`/global_cylinders` harus tepat satu. Validator akan melaporkan
`mavros_vision_plugin_missing` jika `mavros_extras` tidak terpasang. Setelah
itu mulai misi:

```bash
ros2 topic pub --once /mission/start std_msgs/msg/Bool "{data: true}"
```

Pantau state sampai `DONE`:

```bash
ros2 topic echo /mission/fsm_state
```

Laporan ditulis ke:

```text
~/beehive_mission_reports/sim_real_stack/mission_YYYYMMDD_HHMMSS/
```

## World berbukit

Pohon pada world berbukit berada di ground Z 0.35 m. Jalankan:

```bash
ros2 launch uav_plantation_sim plantation_sim.launch.py
ros2 launch beehive_drone real_stack_sim.launch.py \
  tree_ground_z:=0.35 auto_start:=false
```

Periksa `tree_xy_error_m`, AGL, approach, dan orbit pada status serta laporan.

## Uji transformasi orientasi

Pengujian ini sengaja memberi mount yaw 30 derajat pada kamera sintetis. Posisi
pohon global tetap harus berada di sekitar `(7, 0)` walaupun bounding box berada
dalam frame kamera yang diputar:

```bash
ros2 launch beehive_drone real_stack_sim.launch.py \
  camera_yaw:=0.5235987756 auto_start:=false
```

Gerakkan/yaw drone atau jalankan misi, lalu pantau:

```bash
ros2 topic echo /simulation/real_stack/status
```

`tree_xy_error_m` harus tetap di bawah 0.35 m. Ini merupakan uji langsung untuk
kesalahan orientasi kamera -> `base_link` -> `map` yang sebelumnya membuat
koordinat target bergerak bersama drone.

## Uji noise dan dropout

Setelah baseline lulus, ulangi dengan noise posisi 5 cm dan satu frame hilang
setiap sepuluh deteksi:

```bash
ros2 launch beehive_drone real_stack_sim.launch.py \
  position_noise_stddev:=0.05 dropout_every_n:=10 auto_start:=false
```

Mapping harus tetap stabil, satu pohon tidak boleh berubah menjadi banyak ID,
dan mission tidak boleh masuk `ABORT` karena satu frame hilang.

## Uji EKF3 ExternalNav

Profil berikut menguji apakah `/mavros/vision_pose/pose` benar-benar menjadi
sumber posisi horizontal dan yaw EKF3. Profil menggunakan barometer untuk POSZ,
sesuai arsitektur kendaraan nyata. Profil ini terpisah agar kegagalan tuning EKF
tidak tertukar dengan kegagalan BB/misi.

Perhatian: profil ExternalNav murni dapat membentuk bootstrap loop—EKF menunggu
vision, sedangkan auto-alignment menunggu local pose FC. Karena itu validasi
alignment pertama kali harus memakai profil baseline `sitl_sim.parm`. Profil
berikut baru dipakai setelah jalur alignment terbukti dan FC menyediakan local
pose/yaw awal yang valid. Ganti command SITL Terminal 2 dengan:

```bash
sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON -w \
  --add-param-file=$PWD/src/beehive_drone/config/sitl_external_nav.parm \
  --console --out=udp:127.0.0.1:14551
```

Kemudian jalankan MAVROS dan `real_stack_sim.launch.py` seperti baseline.
Periksa pesan FC dan local pose:

```bash
ros2 topic echo /mavros/statustext/recv
ros2 topic hz /mavros/vision_pose/pose
ros2 topic hz /mavros/local_position/pose
ros2 topic echo /simulation/real_stack/status
```

Validator memeriksa local pose MAVROS terhadap ground truth. Jangan mulai misi
sebelum `ready: true` dan pesan pre-arm EKF bersih. File
`sitl_external_nav.parm` adalah khusus SITL dan tidak boleh diunggah utuh ke
Pixhawk.

## Kriteria lulus sebelum uji nyata

1. Validator `ready: true` pada baseline datar.
2. `tree_xy_error_m <= 0.35` saat drone translasi dan yaw.
3. Tepat satu publisher `/global_cylinders`.
4. State mencapai `DONE`, drone kembali ke home, landing, dan disarm.
5. Error approach dan radius orbit pada `mission_summary.json` berada dalam
   toleransi konfigurasi.
6. Baseline, mount-yaw, noise/dropout, terrain berbukit, dan ExternalNav EKF
   diuji sebagai run terpisah.
7. Uji ZED2i tanpa propeller tetap wajib karena simulasi tidak memodelkan kualitas
   neural detection, exposure, motion blur, maupun kegagalan USB/kamera.
