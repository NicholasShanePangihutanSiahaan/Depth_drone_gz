# Beehive Drone — Total Revision (PCL + ROS 2 + Gazebo)

Paket ini menggantikan pipeline lama menjadi satu pipeline dengan **PCL sebagai sumber utama posisi dan ID pohon**. YOLO/depth tetap tersedia sebagai fallback dan dapat dipakai kemudian untuk deteksi bunga.

## Arsitektur

```text
/zed2i/depth/points + /mavros/odometry/out
                    |
                    v
             pcl_proc_node
                    |
/perception/pcl/tracked_cylinders
                    |
                    v
             pcl_tree_mapper
                    |
                /map/trees
                    |
                    v
 mission_state_machine -> dynamic_orbit_controller
                    |                 |
                    +------ goals ----+
                              |
                 vortex_avoidance_controller
                              |
                 velocity_controller
                              |
          /mavros/setpoint_velocity/cmd_vel
```

Fallback opsional:

```text
RGB + registered depth -> YOLO -> PointStamped(camera optical)
                              -> tree_localizer(TF2)
                              -> PointStamped(odom)
                              -> pcl_tree_mapper
```

## Perubahan utama

1. Hanya `pcl_tree_mapper` yang boleh mempublikasikan `/map/trees`.
2. ID, posisi, radius, tinggi, confidence, `seen_count`, dan `missed_count` dari PCL difilter sebelum masuk peta misi.
3. Status `inspected` disimpan di mapper dan tidak hilang saat PCL memperbarui posisi.
4. FSM memiliki pemeriksaan koneksi, freshness pose/map, verifikasi target, retry, timeout orbit, landing confirmation, dan home point aktual.
5. Titik pendekatan menggunakan radius orbit, sehingga drone tidak berpindah mendadak dari jarak pendekatan 2 m ke orbit 3 m.
6. Orbit menghitung progres bertanda sesuai arah putaran; jitter atau gerakan mundur tidak dihitung sebagai progres.
7. Avoidance tidak menganggap pohon target aktif sebagai obstacle, tetapi tetap menghindari pohon lain.
8. Velocity controller mengirim nol ketika target/pose stale dan memiliki acceleration limiting.
9. Hover dinyatakan stabil berdasarkan ketinggian **dan kecepatan**, bukan ketinggian saja.
10. YOLO tidak lagi memakai path model dan intrinsic kamera yang ditulis langsung di source; semua melalui parameter dan `CameraInfo`.
11. Tree localization menggunakan TF2, bukan transformasi yaw manual.
12. Mission analyzer tidak membanjiri log setiap 0,5 detik dan menyimpan hasil secara otomatis.

## Penempatan package

Ekstrak folder ini sebagai:

```text
~/Gazebo_sim/src/beehive_drone/
```

Repository PCL harus berada di workspace yang sama:

```bash
cd ~/Gazebo_sim/src
git clone https://github.com/Wafi-Afdi/PCL_Segmentation.git
```

Pastikan tidak ada package `beehive_drone` lama yang masih ikut dibangun.

## Instalasi dependensi

```bash
cd ~/Gazebo_sim
rosdep install --from-paths src --ignore-src -r -y
python3 -m pip install ultralytics
colcon build --symlink-install
source install/setup.bash
```

## Menjalankan mode PCL utama

```bash
ros2 launch beehive_drone complete_mission.launch.py \
  use_pcl:=true \
  use_yolo_fallback:=false \
  point_cloud_topic:=/zed2i/depth/points \
  odom_topic:=/mavros/odometry/out
```

## Menjalankan fallback YOLO

```bash
ros2 launch beehive_drone complete_mission.launch.py \
  use_pcl:=false \
  use_yolo_fallback:=true \
  yolo_model_path:=/absolute/path/best_detection_palm_oil.onnx \
  yolo_device:=cpu
```

Gunakan `yolo_device:=0` jika Ultralytics/CUDA pada Jetson sudah dikonfigurasi dan model mendukungnya.

## Pemeriksaan sebelum terbang

```bash
ros2 topic hz /zed2i/depth/points
ros2 topic hz /mavros/odometry/out
ros2 topic echo /perception/pcl/tracked_cylinders --once
ros2 topic echo /map/trees --once
ros2 topic echo /mission/fsm_state
ros2 topic echo /control/orbit_status
ros2 topic echo /mavros/setpoint_velocity/cmd_vel
```

Periksa TF:

```bash
ros2 run tf2_ros tf2_echo odom zed2i_left_camera_optical_frame
```

Nama frame kamera dapat berbeda sesuai URDF/plugin Gazebo.

## Catatan penting tentang PCL_Segmentation

Source PCL saat ini menulis frame output sebagai `plantation` dan memakai matriks optical-to-robot tetap. Launch ini menambahkan TF identitas `odom -> plantation` karena nilai globalnya dihitung dari odometri. Namun, orientasi dan translasi pemasangan kamera pada model Gazebo tetap harus sesuai dengan asumsi source PCL. Untuk pemasangan kamera yang tidak tepat di origin `base_link`, source C++ PCL perlu ditambah extrinsic translation atau diubah memakai TF2.

Jangan menjalankan `tree_mapper.py` lama bersamaan dengan `pcl_tree_mapper`, karena dua publisher `/map/trees` akan menghasilkan peta yang saling menimpa.

## Parameter tuning

Ubah `config/mission.yaml`. Parameter paling penting:

- `flight_altitude`
- `orbit_radius`
- `tree_min_confidence`
- `min_seen_count`
- `max_missed_count`
- `influence_radius`
- `hard_radius`
- `max_velocity_xy`
- `max_acceleration_xy`

Mulai pengujian dengan kecepatan maksimal rendah dan tanpa propeller pada hardware nyata.

## Validasi yang sudah dilakukan

Semua file Python dan launch file telah melewati `python3 -m compileall`. Pengujian runtime ROS 2, Gazebo, MAVROS, model YOLO, dan flight controller tetap harus dilakukan di workspace Anda karena environment pembuatan paket ini tidak memiliki graph ROS/Gazebo aktif.
