# Validasi software tes SLAM — 6 November 2026

Paket terpisah `mid360_slam_test` pada branch `lidar_test`. Tidak mengubah kontrol
simulasi atau pekerjaan lokal lain. Tidak ada commit, push, koneksi SSH atau
pengoperasian perangkat hardware oleh agen.

Implementasi mencakup preprocessing, estimator eksperimen ICP dengan prediksi
rotasi IMU, pose graph SE(3), loop event accepted/rejected, menu manual dengan
ACK, evaluator bertahap, recorder, log dan konfigurasi RViz. Tidak mengklaim
FAST-LIO, covariance yang terkalibrasi, deskew atau SLAM produksi.

## Hasil yang sudah dijalankan

| Pemeriksaan | Status | Cakupan |
| --- | --- | --- |
| Suite gabungan sensor lama dan SLAM baru | PASS | 50 tes, 0 gagal, 0 dilewati; domain localhost 195 |
| Core estimator dan evaluator baru | PASS | 23 kasus, termasuk rigid registration, rotasi penuh, loop positif/negatif, graph correction dan referensi tahap |
| Integrasi ROS baru | PASS | 2 kasus, sensor → estimator → hasil tahap → loop event → bag/peta dan exclusion box |
| Build colcon paket terpisah | PASS | mid360_test dan mid360_slam_test, build/install pada /tmp |
| Launch antarp­roses paket terpasang | PASS | 6 proses, tahap diam melalui command/ACK, 2 loop accepted dari fixture, 492 pesan bag |
| Python/XML/YAML dan konfigurasi RViz | PASS | Pemeriksaan sintaks, bukan inspeksi visual GUI |
| Hardware MID-360/Jetson | NOT TESTED | Tidak ada perangkat yang diakses agen |
| Error posisi/sudut dan drift terhadap pengukuran fisik | NOT TESTED | Tidak ada acuan independen hardware |
| Keliling pohon dan loop closure lapangan | NOT TESTED | Tidak ada pengambilan data lapangan |
| RViz melalui NoMachine | NOT TESTED | GUI Jetson belum dibuka/diperiksa |

Laporan JUnit dan JSON: `reports/slam_test/software_validation_20261106/`.
Smoke test lokal: `/tmp/mid360_slam_launch_jc_2e8eb/session`.
Direktori /tmp sementara; pengujian bisa diulang menggunakan `test/check_launch.py`.

Masalah yang ditemukan saat pengembangan:

- Pengujian waktu floating point mengungkap selisih numerik sub-nanodetik pada
  perbandingan now vs receipt; toleransi 1e-9 ditambahkan, bukan melonggarkan timeout.
- Putaran sintetis 360° mengungkap akumulasi roundoff matriks rotasi karena
  invers transpose memerlukan orthonormality. Produk rotasi sekarang diproyeksi
  kembali ke SO(3); regression test rotasi penuh kemudian lulus.
- Titik acuan map tahap perlu direvisi dengan graph yang sama setelah loop,
  agar error endpoint tidak membandingkan dua revisi peta berbeda. Ditambahkan
  graph solution pada event dan regression test revisi anchor.
- Smoke test antarp­roses pertama terlalu cepat memeriksa readiness dari jendela
  fixture 0.2 detik. Fixture kini menunggu can_start dan memakai 0.4 detik dengan
  minimum tiga sampel; batas hardware default 1 detik tidak diubah.
- Penggabungan suite menemukan bentrok nama test_ros.py pada plugin pytest ROS.
  File baru dinamai test_slam_ros.py; suite gabungan kemudian lulus seluruhnya.

Keterbatasan: titik ukur adalah pusat LiDAR, IMU hanya inisialisasi/prediksi
rotasi, tidak ada tightly coupled fusion/deskew/global place recognition.
Kandidat loop bergantung kedekatan pose dan dapat gagal pada drift besar atau
geometri sawit berulang. Loop accepted tetap harus diverifikasi dengan referensi
fisik dan peta; endpoint nol tidak dianggap sebagai loop closure.
