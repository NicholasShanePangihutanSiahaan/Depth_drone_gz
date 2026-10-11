# Depth kamera untuk ruang depan-bawah

Opt-in pada `tilt_inspection.json`: LiDAR + RGB-D simulasi menjadi sumber
occupancy yang sama. Sensor RGB-D Gazebo **bukan emulasi stereo/depth SDK ZED
2i**. Kamera RGB lama di model asli tetap utuh; launcher menambahkan kemampuan
depth hanya pada salinan model setiap run.

## Perubahan dan alasan

- Kamera 320×240, 5 Hz, menghasilkan RGB, depth dan CameraInfo. RGB tetap
  `/zed/image`, sehingga viewer lama tidak berubah.
- Mapper memproyeksikan depth memakai intrinsics CameraInfo. Koordinat optical
  adalah kanan/bawah/depan; TF optical dan yaw/pitch terukur diterapkan pada
  timestamp gambar. Gerak mount tidak dianggap sebagai gerak drone.
- Maksimum 800 sinar per gambar, stride 8 dengan fase bergeser; batas kedalaman
  optical 0,3–20 m, dengan clip sensor dan batas proyeksi yang sama. Padding
  baris, endian dan format 32FC1/16UC1 ditangani. Batas 8 m sebelumnya membuang
  pantulan tanah jauh yang dibutuhkan untuk membuktikan ruang bawah jalur.
- NaN, infinity, nol dan depth di luar rentang **tidak membuat ruang bebas**.
  Semua titik valid adalah hit permukaan. Ruang sebelum hit diintegrasikan
  bebas, permukaan occupied, belakangnya unknown. Occupied LiDAR maupun kamera
  tidak dihapus oleh sinar sensor lain. Filter badan tetap aktif.
- Satu worker mengintegrasikan dua origin sensor ke history yang sama, memakai
  ROI masing-masing. Kamera tidak mengganti health/timestamp LiDAR; dropout
  LiDAR tetap memicu keselamatan. Saat APPROACH, depth yang telah terintegrasi
  harus segar (<1 detik simulasi), atau drone tidak boleh meneruskan approach.
- Saat APPROACH (termasuk EXPLORE), kamera mengarah ke jalur dengan depresi
  nominal 25°. Saat fase pengamatan atas/SCAN, kamera kembali ke perkiraan
  atas batang. Perintah memakai body rotation dan offset mount, bukan mengganti
  frame label. Pembatas joint dan laju yaw/pitch tetap aktif.

## Visualisasi dan audit

RViz mempunyai checkbox `Camera depth cloud (simulated RGB-D, not ZED stereo)`
untuk `/zed/depth/points`. Cloud akumulasi dan voxel/log HTML mengandung hasil
LiDAR dan depth. Viewer kamera tetap RGB; ini bukan preview depth palsu.

`/navigation/status` dan `navigation_debug.jsonl` memuat `depth_mapping`:
jumlah frame diterima/disubmit, titik valid/ditolak, self-filter, origin kamera,
timestamp valid dan **timestamp terintegrasi**, alasan penolakan, serta statistik
ROI. `mapping_io_callback_wall_max_ms` mencatat biaya penerimaan/proyeksi depth.
`check_ros_simulation.py` memeriksa tiga stream, optical frame, dan integrasi
depth yang segar jika fitur ini aktif.

## Menjalankan

```bash
cd /home/abin/polinasi_lidar
source tools/ros_environment.sh
/usr/bin/python3 tools/generate_sensor_gimbal_config.py
colcon --log-base log_ros build --symlink-install --build-base build_ros --install-base install_ros --base-paths polinasi_nav beehive_drone uav_interfaces --packages-select polinasi_nav
bash tools/start_tilt_inspection.sh
```

Terminal kedua:

```bash
cd /home/abin/polinasi_lidar
source tools/ros_environment.sh
.venv/bin/python tools/set_origin.py --timeout 120
/usr/bin/python3 tools/check_ros_simulation.py --output reports/depth_inspection_setup.json
# Hanya bila setup passed=true:
/usr/bin/python3 tools/start_mission.py
```

RViz dan kamera dibuka; server Gazebo tetap menjalankan fisika tanpa GUI Gazebo.
Radius 2 m, tilt tunggal 68°, kamera pitch saja dan 6 viewpoint berlaku pada
konfigurasi terbaru. Saat transit kamera melihat bawah jalur; saat hover pada
viewpoint badan menghadap pohon dan kamera melihat ke atas. Lihat
[alur terbaru](TILT_ASCENT.md). Jangan menjalankan dua simulator.

## Pengujian dan batas

31 tes terarah geometri/depth/pemetaan/mount/FSM dan 27 tes adapter ROS lulus
saat implementasi. Tes meliputi invalid depth, endian/padding, anggaran sampling,
rotasi optical, dua origin, unknown di belakang hit, obstacle sticky, model asli
tidak berubah, serta konfigurasi radius/tilt.

RGB-D ideal belum mensimulasikan kegagalan stereo karena tekstur, cahaya atau
daun bergerak. Untuk ZED asli perlu stream calibrated registered depth dengan
filter confidence, verifikasi rentang/occlusion, dan audit TF mount. Tidak ada
klaim lokalisasi LiDAR–IMU, sensor hardware, atau full mission tervalidasi.
Integrasi ini juga belum memperbaiki semua kebijakan eksplorasi/target
perantara yang memilih ketinggian rendah; hasil run harus diaudit terpisah.

Dasar antarmuka: [Gazebo RGB-D Sensor](https://gazebosim.org/api/sensors/8/classgz_1_1sensors_1_1RgbdCameraSensor.html)
dan [ZED depth ROS 2](https://docs.stereolabs.com/docs/integrations/ros-2/depth-sensing).

[Spesifikasi ZED 2i wide](https://docs.stereolabs.com/docs/products/cameras/zed/specifications)
menyebut maksimum 20 m, rentang ideal sampai 12 m. Simulasi memakai depth ideal
sampai 20 m; ini BUKAN bukti depth hardware pada 20 m cukup akurat untuk terbang.
Deployment hardware memerlukan confidence/error model dan margin terpisah.

Pada kontrol, umur hasil depth terintegrasi maksimum 1 detik (batas inklusif
dengan toleransi floating point). Jika hasil terintegrasi terlambat tetapi
depth valid masih segar, drone yang sudah diam di posisi hold aman boleh
menunggu maksimum 2 detik tanpa menerima trajectory baru. Selama menunggu,
LiDAR, localisation dan pemeriksaan akhir PID tetap aktif. Kamera hilang,
hold tidak aman, timeout berlanjut, atau depth stale saat bergerak tetap gagal.
`depth_safety` membedakan umur data valid/terintegrasi dan aksi fresh/hold/fail;
`depth_mapping.last_received_stamp` membantu audit antrean. Ini bukan izin
terbang memakai depth basi atau ke ruang unknown.
