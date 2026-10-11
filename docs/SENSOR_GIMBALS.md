# LiDAR tilt dan kamera depan (simulasi opt-in)

Konfigurasi yang dihasilkan generator sekarang memakai **kamera pitch saja**
(`camera_yaw_enabled=false`). Yaw joint fixed, tidak menerima perintah dan
tidak ditunggu feedback-nya. Saat transit badan mengikuti jalur dan kamera
melihat ke bawah; saat viewpoint badan menghadap pohon dan kamera ke atas.
Lihat [6 viewpoint dan tilt tunggal 68°](TILT_ASCENT.md).
Bagian dua sumbu di bawah adalah mode legacy opt-in
`camera_yaw_enabled=true`, bukan konfigurasi misi terbaru.

Perubahan:

- LiDAR memiliki joint pitch; perintah positif menghadap atas. IMU internal
  ikut bergerak bersama LiDAR, sesuai model fisiknya.
- Kamera berada pada posisi depan `[0.20, 0, 0.02]` m terhadap badan FLU
  (X depan, Y kiri, Z atas), dengan joint yaw lalu pitch.
- Pada konfigurasi gimbal, badan mengikuti arah kecepatan lintasan saat
  scanning dan mempertahankan yaw saat hover. Kamera mengarah ke kandidat
  pohon secara independen memakai orientasi badan yang terukur.
- Node mount adalah satu-satunya publisher perintah joint final dan TF mount.
  Transformasi mengikuti sudut joint terukur dan timestamp, bukan target servo.
- Self-filter cloud menggunakan transformasi LiDAR–badan pada timestamp scan.
- Referensi model Gazebo dikunci ke `iris_with_standoffs::base_link`. Tanpa
  ini link LiDAR yang bergerak dapat menjadi referensi pose model dan
  menyebabkan gerakan sensor salah terbaca sebagai gerakan drone.

Model asli tetap utuh. Joint hanya ditambahkan ke salinan model milik run.
Aktifkan dengan `sensor_gimbals_enabled=true` dan `scan_body_faces_path=true`.
Dasar referensi pose: [SDFormat canonical link](https://sdformat.org/tutorials/specification/pose_frame_semantics/1.7/).
Aktuator memakai [Gazebo JointPositionController](https://gazebosim.org/api/sim/8/classgz_1_1sim_1_1systems_1_1JointPositionController.html).

## Build dan uji tanpa takeoff

```bash
cd /home/abin/polinasi_lidar
source tools/ros_environment.sh
/usr/bin/python3 tools/generate_sensor_gimbal_config.py
colcon --log-base log_ros build --symlink-install --build-base build_ros --install-base install_ros --base-paths polinasi_nav beehive_drone uav_interfaces --packages-select polinasi_nav
bash tools/start_sensor_gimbal_test.sh
```

Gazebo dan RViz dibuka. Jangan mengirim `/mission/start` untuk uji mount.
Terminal kedua:

```bash
cd /home/abin/polinasi_lidar
source tools/ros_environment.sh
/usr/bin/python3 tools/check_sensor_gimbals.py --output reports/sensor_gimbals_check.json
```

Program menguji tiga sumbu, feedback, TF, aliran gambar/cloud, dan memastikan
pose badan tidak ikut berubah signifikan. Tidak menerbitkan arm/takeoff.
Perintah reset dikirim saat selesai. Semua sudut dalam radian.

## Kontrol manual

```bash
# LiDAR naik sekitar 26 derajat; hanya saat drone tidak di-arm.
ros2 topic pub --once /mounts/lidar/pitch_target std_msgs/msg/Float64 '{data: 0.45}'
# Kamera menoleh kiri sekitar 69 derajat dan melihat atas sekitar 37 derajat.
ros2 topic pub --once /mounts/camera/yaw_target std_msgs/msg/Float64 '{data: 1.2}'
ros2 topic pub --once /mounts/camera/pitch_target std_msgs/msg/Float64 '{data: 0.65}'
# Kembalikan LiDAR ke posisi normal sebelum memulai misi.
ros2 topic pub --once /mounts/lidar/pitch_target std_msgs/msg/Float64 '{data: 0.0}'
ros2 topic echo /mounts/status
```

Tidak mengirim langsung ke `/lidar/pitch_cmd` atau `/camera/*_cmd`: itu keluaran
internal node mount. Jangan membuat publisher kontrol tambahan.

Limit joint dan adapter LiDAR simulasi: −0,2 rad sampai +70° (1,22173 rad);
kamera yaw ±170°, pitch −0,5 sampai +1,2 rad. Ini bukan limit hardware terkalibrasi.
Target dibatasi; slew perintah LiDAR 0,35 rad/s dan kamera 0,5 rad/s. Ini belum
model servo perangkat keras yang diidentifikasi.

## Batas yang penting

- Hanya ground-truth simulation. Estimator LiDAR–IMU sekarang mengasumsikan
  mount tetap; konfigurasi gimbal ditolak pada mode sensor. Kalibrasi/dinamika
  mount bergerak diperlukan sebelum lokalisasi sensor dapat digunakan.
- Pada konfigurasi dasar `tree_inspection_gimbal.json`, tilt airborne tetap
  tidak diaktifkan. Pada konfigurasi baru `tilt_inspection.json`, tilt airborne
  diizinkan khusus oleh FSM observasi/ascent yang segar; perubahan sudut hanya
  saat diam. Lihat [TILT_ASCENT.md](TILT_ASCENT.md). Permintaan manual nonzero
  membutuhkan status disarmed yang segar; jika armed atau status hilang,
  target dikembalikan nol. Navigasi menunggu feedback normal/segar sebelum
  start dan berhenti aman jika mount tidak siap saat penerbangan.
- FSM otomatis kini tersedia pada konfigurasi tilt terpisah. Uji joint terdahulu
  tetap bukan validasi alur penerbangan tersebut.
- Kamera masih placeholder RGB tunggal, bukan emulasi stereo/depth ZED 2i.
  Penilaian mutu gambar, deteksi bunga, dan manuver untuk target di luar limit
  kamera belum tersedia. Arah kamera ditujukan ke perkiraan ketinggian batang.
- Gazebo GPU LiDAR memakai snapshot scan; kompensasi gerakan mount per titik
  untuk sensor nyata belum tersedia. Robot tetap diam pada uji ini.
- Log `/navigation/status` memuat `sensor_mounts` dan kebijakan arah badan.
  `/mounts/status` memuat target, perintah, sudut aktual, freshness, dan penolakan.

Untuk berhenti beserta kedua GUI, gunakan direktori RUN yang dicetak launcher:

```bash
/usr/bin/python3 tools/stop_simulation.py --run RUN
```

## Hasil uji 10 Oktober 2026

Run pertama dihentikan: saat tilt 0,45 rad, odometri ikut berputar sekitar
26° dan posisi berubah 9,1 cm. Ini bukan gerakan badan yang benar: referensi
model mengikuti link sensor. Perbaikan dilakukan pada `canonical_link`,
bukan dengan mengubah nama frame.

Run ulang `ros_ground_truth_palm_farm_20261010_214653_56195_mapping` lulus empat
pose joint (normal, tilt/kiri/atas, tilt/kanan, kembali normal). Gambar dan
point cloud diterima pada seluruh langkah. Perubahan posisi badan maksimum
5,2 mm; perubahan matriks rotasi badan sekitar 1,05e-7. Galat sudut feedback
dibatasi 0,03 rad dalam tes; itu toleransi uji, bukan akurasi servo nyata.
Hasil: `reports/sensor_gimbals_run2.json`. Tidak ada arm, takeoff, atau validasi
lokalisasi LiDAR pada pengujian ini.
