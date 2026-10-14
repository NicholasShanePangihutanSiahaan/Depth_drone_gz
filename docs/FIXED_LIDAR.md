# Uji MID-360 fixed

Branch `fixed-lidar`; konfigurasi rotating tetap di branch `rotating-lidar`.
MID-360 horizontal, fixed pada `[0, 0, 0.28]` meter terhadap badan.
ZED di depan `[0.20, 0, 0.06]`, yaw nol, hanya pitch (termasuk ke atas).
Model visual 50 cm dipertahankan; dinamika tetap Iris, bukan model hardware
50 cm teridentifikasi. Pantulan badan tidak disembunyikan dari sensor.

Lokalisasi sensor memakai LiDAR dan IMU badan yang rigid. Ground truth hanya
dipakai auditor, tidak dikirim ke kontrol. Estimator eksperimental yang sama
dengan branch rotating dipakai untuk perbandingan, bukan FAST-LIO produksi.
Depth fusion tidak aktif pada fixture hover untuk mengisolasi baseline LiDAR.
Fixture inspection memakai RGBD sintetis untuk mengamati bagian bawah jalur;
ini bukan emulasi kesalahan stereo ZED atau validasi hardware. Explore scan
tetap pada ketinggian sama. Jika unknown menghalangi, kamera mengamati jalur
ke bawah saat hold, lalu kembali tree-up sebelum melanjutkan scanning.
Unknown tetap ditolak; tidak ada pelonggaran clearance atau bypass takeoff.

Jalankan tanpa GUI, dari root repository:

```bash
source tools/ros_environment.sh
bash tools/run_fixed_lidar_validation.sh hover
# Hanya setelah hover COMPLETE dan disarmed, lanjutkan:
bash tools/run_fixed_lidar_validation.sh inspection
```

Runner melakukan build, menyalakan Gazebo/SITL/ROS, menetapkan origin,
memeriksa setup, memulai misi, mengaudit, dan menghentikan run miliknya.
Run gagal tidak diulang buta: periksa log, perbaiki, tes, lalu jalankan lagi.
Jangan menjalankan dua simulasi bersamaan. Semua perintah hanya untuk SITL.

Log tahap/setup/akurasi: `reports/fixed_lidar_validation_<tanggal>/`.
`run.txt` menunjuk log mentah Gazebo/ROS/SITL; map HTML berada di
`reports/<nama-run>/map_3d/map.html`. Akurasi, alasan berhenti dan status
terakhir tersimpan dalam `accuracy.summary.json`.

Uji kode: 30 tes fixed-geometry, sensor-localisation, continuous-scan/ascent,
1 tes stop idempotent dan 10 tes adapter mount ROS lulus.
Baseline `fixed_lidar_validation_20261014_082953`: takeoff-hover-return-land,
COMPLETE/disarmed, RMSE posisi 1,60 cm; maksimum 5,43 cm (802 pasangan).
Inspeksi dua pohon belum dinyatakan lulus oleh hasil baseline ini.
