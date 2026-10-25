# Livox tetap di atas drone — main

## Konfigurasi

Kedua model `plantation_quadrotor` dan `plantation_quadrotor_ardupilot`
memiliki `livox_top_link`, fixed joint ke `base_link`, pose numerik
`x=0, y=0, z=0.18 m, roll=pitch=yaw=0`. Posisi adalah pusat housing
simulasi, bukan pengukuran extrinsic hardware. Sumbu X depan, Y kiri, Z atas.
Housing 65×65×60 mm, massa .265 kg dan inertial box ditambahkan sebagai link
terpisah. Total massa drone meningkat .265 kg; controller belum diuji ulang
atau dituning untuk massa ini.

| Data | Gazebo | ROS 2 | Frame |
|---|---|---|---|
| Point cloud | `/livox/top/scan/points` | `/livox/top/points` (`sensor_msgs/msg/PointCloud2`) | `livox_top_link` |
| IMU sensor atas | `/livox/top/imu` | `/livox/top/imu` (`sensor_msgs/msg/Imu`) | `livox_top_link` |

`/livox/top/scan` adalah keluaran LaserScan Gazebo; gunakan topic `/points`
untuk cloud 3D lengkap. Bridge GZ_TO_ROS, QoS SENSOR_DATA. Timestamp sensor
tidak diganti. Launch `plantation_sim.launch.py` menambahkan satu static TF
`base_link → livox_top_link` dengan pose yang sama. Topic ZED, `/imu` lama,
rangefinder, flight controller, dan routing navigasi tidak dialihkan.
LiDAR belum menjadi input obstacle avoidance atau lokalisasi otomatis.

## Build dan penggunaan normal

Jalankan dari root checkout/worktree **main**, bukan rotating-lidar/fixed-lidar:

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select uav_plantation_sim
source install/setup.bash
ros2 launch uav_plantation_sim plantation_sim.launch.py \
  world:=plantation_simple.sdf gz_args:='-s -r --headless-rendering -v 3'
```

Launch normal tetap memiliki dependensi stack lama (antara lain
`uav_localization`, `robot_state_publisher`, `depth_image_proc` dan plugin
ArduPilot); build/start sesuai README utama jika belum terpasang. Perubahan
ini tidak memasang atau memperbaiki seluruh stack tersebut. Untuk hanya
memeriksa Livox tanpa flight stack, gunakan bench terisolasi di bawah.

Terminal lain, dengan environment/domain ROS yang sama:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 topic info /livox/top/points -v
ros2 topic hz /livox/top/points
ros2 topic echo --once /livox/top/imu --qos-reliability best_effort
ros2 run tf2_ros tf2_echo base_link livox_top_link
```

RViz opsional: tambah PointCloud2 `/livox/top/points`, Reliability Best Effort.
Frame `base_link` cukup untuk inspeksi relatif drone; gunakan `odom` hanya
jika transform odometry memang tersedia. Jangan mengganti label frame untuk
menyamarkan transform yang hilang. Hentikan launch dengan Ctrl+C.

## Bench sensor tanpa hardware, arm, SITL atau MAVROS

```bash
source /opt/ros/humble/setup.bash
ROS_DOMAIN_ID=182 /usr/bin/python3 tools/check_top_livox.py \
  --output reports/livox_top_check/result.json --timeout 90
```

Gunakan domain ROS yang belum dipakai sesi lain. Script membuat GZ_PARTITION
unik dan menguji world **plantation_simple.sdf**. Salinan sementara drone
dibuat stationary pada `[0,-3,.12]`; actuator plugins dan sensor selain
Livox dihapus HANYA pada salinan bench. Model sumber, world, dan sensor
lain tidak berubah. Ini mengisolasi pengujian LiDAR/IMU tanpa membuka port
SITL atau mengirim perintah terbang. Tidak ada GUI. Script menghentikan
process group miliknya saat selesai/gagal, tidak menghentikan simulasi lain.

Pada 25 Oktober 2026:

- Build `uav_plantation_sim` berhasil.
- `colcon test --packages-select uav_plantation_sim`: 4 tes konfigurasi lulus.
- Bench di plantation_simple lulus: 10 cloud, 183 pesan IMU, maksimum
  2092 titik finite dalam satu cloud 360×32, frame sesuai, timestamp meningkat,
  clock diterima, dan static mount TF sesuai. Semua proses uji berhenti.
- Hasil lokal: `reports/livox_top_20261025/result.json` beserta log bridge/Gazebo.
- Belum diuji terbang dengan massa tambahan, belum validasi MID-360 hardware,
  bukan validasi lokalisasi atau navigasi.

Tes konfigurasi tanpa simulator:

```bash
python3 -m pytest src/uav_plantation_sim/test/test_livox_top.py -q
```

## Penyederhanaan

Horizontal 360°, vertikal −7°…+52°, cloud 10 Hz dengan grid 360×32 rays,
range simulasi .1–20 m dan Gaussian range noise σ=.02 m. Ini raster GPU
LiDAR, **bukan pola non-repetitif MID-360 asli**, bukan model akurasi terhadap
reflectivity/cuaca, dan tidak menyediakan deskew per-point Livox CustomMsg.
IMU 200 Hz menggunakan noise sederhana, bukan kalibrasi IMU hardware.
Housing dan body visual dapat menutup sebagian sinar; cloud yang diterbitkan
adalah raw, belum self-filtered. Infinity/NaN bukan otomatis ruang kosong.

Spesifikasi FOV/dimensi/massa merujuk
[Livox MID-360](https://www.livoxtech.com/cn/mid-360/specs).
Konfigurasi sensor dan bridge mengikuti dokumentasi
[Gazebo Harmonic sensors](https://gazebosim.org/docs/harmonic/sensors/) dan
[ROS 2 integration](https://gazebosim.org/docs/harmonic/ros2_integration/).
