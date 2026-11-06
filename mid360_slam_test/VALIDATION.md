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

## Penambahan filter bola untuk mounting pitch-down

Filter mendukung pusat di frame badan (mode lama `body_offset`) dan koordinat
langsung dalam frame LiDAR (`lidar_coordinates`). Konfigurasi hardware terbaru
menggunakan pitch-down 20 derajat sebagai informasi mounting, pusat (0,0,-16) cm
di frame LiDAR, dan radius 44 cm berdasarkan informasi operator. Pusat tidak
dirotasi lagi. Filter hardware aktif; default node dan replay tetap nonaktif.
Raw bag tidak diubah. Preview titik terhapus dan marker bola tersedia di RViz.

Tes geometri mencakup konversi satuan, rotasi mode lama, pusat native yang tidak
berubah terhadap pitch, konfigurasi hardware, batas bola, titik di luar bola,
serta parameter invalid. Kesesuaian fisik dan preview RViz pada Jetson tetap
NOT TESTED. Laporan lokal: `reports/slam_test/body_sphere_validation/`.

Verifikasi perubahan pusat native/radius: 28 tes geometri dan 3 tes integrasi
ROS sintetis lulus. Termasuk uji pusat tetap (0,0,-0.16) m walau pitch berubah;
mode native tidak menerapkan rotasi mounting kedua kali.

Tampilan berorientasi drone ditambahkan: TF statis `slam_test_drone_view` ke
frame LiDAR dengan pitch mounting +20 derajat dan translasi nol (asal tetap di
LiDAR). Hardware default memakai tampilan tersebut; `rviz_view:=slam` dan
`rviz_view:=lidar` tetap tersedia. Uji ROS sintetis lantai horizontal memverifikasi
arah rotasi TF dan koreksi kemiringan, serta transform pusat bola yang konsisten.
28 tes geometri dan 4 tes ROS lulus. Ini bukan kompensasi kemiringan badan drone
terhadap gravitasi dan bukan perubahan orientasi peta SLAM. GUI/hardware tetap
NOT TESTED.

## 2026-11-06 — Kamera mengikuti posisi tanpa mengikuti rotasi

Ditambahkan node pasif `follow_view`, pilihan `rviz_view:=follow` (default
hardware), target kamera dengan rotasi identitas di `slam_odom`, dan sumbu
pose badan dengan mounting pitch 20 derajat. Pusat badan memakai pengukuran
(0,0,-16) cm pada frame LiDAR; radius bola 44 cm. Scan/pose harus cocok timestamp
sebelum digunakan. Tracking invalid/basi menghapus scan live dan menampilkan
NOT READY; tidak menghasilkan pose pengganti atau perintah gerak.

11 tes tampilan/integrasi ROS sintetis lulus: lima kombinasi orientasi, uji
ROS pairing/center/dropout/malformed input, registrasi ICP aktual untuk gerak
roll/pitch/yaw bertahap, dan empat tes integrasi ROS sebelumnya (termasuk
rekaman output follow dari pipeline). Build kedua package berhasil.
Smoke test launch terinstal antarp­roses juga PASS dengan data sintetis;
recorder menyimpan cloud/status follow, 631 pesan total, dan dua loop diterima.
Laporan JSON/JUnit: `reports/slam_test/follow_view_validation/`.

Uji dropout menemukan alasan input malformed tertimpa timer stale. Timer
sekarang hanya menandai stale saat tampilan masih aktif, sehingga diagnosis
invalid yang terakhir tetap terbaca sampai ada data sehat berikutnya.

Acuan gravitasi hanya diinisialisasi saat diam oleh estimator yang sudah ada;
tidak ditambahkan continuous gravity fusion atau deskew. Drift/degenerasi
geometri tetap membatasi ketepatan tampilan. GUI NoMachine, hardware, kalibrasi
mounting/IMU, dan akurasi lantai saat gerak asli masih NOT TESTED.

## 2026-11-06 — Perbaikan crash telemetri suhu Jetson

Operator melaporkan startup hardware berhenti pada `resource_tick` dengan
`TypeError: can't concat NoneType to bytes` dari pembacaan sysfs temperatur.
Pembacaan sebelumnya hanya menangani OSError/ValueError. Kini pembaca suhu
juga menangani TypeError dan UnicodeError per sensor, menolak nilai nonfinite,
serta menyimpan `thermal_read_errors` dan status OK/PARTIAL/UNAVAILABLE dalam
`resources.jsonl`. Sensor yang gagal dibaca tidak diberi suhu nol. Kegagalan
telemetri suhu tidak menghentikan pengujian/recorder.

Sembilan tes pembaca suhu lulus, termasuk reproduksi exception dari log operator,
serta regression test callback monitor untuk memastikan logging tetap berlanjut.
Pengujian ulang pada Jetson masih NOT TESTED. Warning ketidakcocokan NumPy/SciPy
dan penolakan publisher perlu didiagnosis terpisah tanpa melonggarkan allowlist.
