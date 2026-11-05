# Hasil implementasi — 30 Oktober 2026

Branch kerja `lidar_test`, basis Git `6779baf`. Paket baru belum di-commit/push.
Perubahan lokal yang sudah ada di luar `mid360_test` tidak diubah oleh pekerjaan ini.

## Hasil yang benar-benar dijalankan

| Pemeriksaan | Status | Bukti dan cakupan |
| --- | --- | --- |
| Perhitungan sektor, invalid/no-return, integrasi IMU, health, log file | PASS | 16 kasus tes core sintetis |
| Integrasi ROS, pembukaan bag, adapter CustomMsg, guard hardware, overflow recorder | PASS | 9 kasus ROS, domain lokal 192 |
| Build paket terpisah dengan colcon dan symlink-install | PASS | Build/install di `/tmp/mid360_build`, `/tmp/mid360_install` |
| Launch paket terpasang tanpa sensor | PASS | Startup enam node, hasil data NOT TESTED, stop normal dan metadata bag tersedia |
| Launch antarp­roses dengan data sintetis | PASS | 40 cloud + 200 IMU diterima dan ditulis, recorder drop 0; empat jarak sesuai fixture; estimasi setelah kalibrasi |
| Koneksi/driver MID-360 asli di Jetson | NOT TESTED | Tidak ada koneksi perangkat/SSH yang diasumsikan atau dilakukan |
| Error jarak terhadap papan terukur | NOT TESTED | Belum ada pengukuran independen |
| Drift yaw/kecepatan pada hardware | NOT TESTED | Integrasi diuji numerik saja |
| Tampilan RViz secara visual | NOT TESTED | Config tersedia; GUI belum dibuka/diverifikasi pada desktop Jetson |
| Kinerja CPU/RAM/latency Jetson | NOT TESTED | Pengujian berjalan pada host pengembangan, bukan benchmark Jetson |

## Masalah yang ditemukan selama implementasi

- Build pertama dimulai ketika README belum selesai ditulis, sehingga data file
  README tidak ditemukan. Setelah file tersedia, build ulang berhasil.
- Tes pertama pada orientasi miring menggunakan pengecekan boolean objek
  scipy Rotation yang tidak didukung. Helper tes diperbaiki; perhitungan diuji ulang.
- Perbandingan byte antara serialisasi Python sebelum publish dan CDR yang
  diterima middleware berbeda pada padding alignment. Verifikasi bag diperbaiki
  menjadi kesetaraan **seluruh field pesan ROS** setelah deserialisasi. Recorder
  sendiri menyimpan bytes CDR asli yang diterimanya tanpa rekonstruksi.
- Sandbox menolak socket DDS serta direktori log ROS default. Tes ROS dijalankan
  dengan izin di luar sandbox pada domain localhost khusus dan log `/tmp`.

## Batas hasil

PASS software bukan PASS hardware. `summary.json` runtime hanya menilai aliran
data yang diterima dan status recorder; akurasi hardware tetap NOT TESTED sampai
operator mengukur dan meninjau buktinya. Tidak ada pengoperasian flight controller,
motor, mount, atau misi. Tidak ada publisher kontrol atau ExternalNav.

Integrasi kecepatan mengasumsikan nol saat awal, penghilangan gravitasi bergantung
pada orientasi gyro yang juga drift, dan offset titik IMU ke pusat LiDAR tidak
dikompensasi. Ini eksperimen, bukan lokalisasi tervalidasi. Saat data terputus,
integrasi invalid sampai sesi dimulai ulang. Tidak ada titik hilang yang dianggap
ruang bebas. Topic sumber raw direkam sebelum filter analisis; hilangnya paket
sebelum callback recorder tidak dapat dijamin terdeteksi.

Log smoke test lokal selama implementasi:

- `/tmp/mid360_launch_o8_g20ml/session`: startup/shutdown tanpa sensor.
- `/tmp/mid360_multiprocess_41v7l7z_/session`: replay sintetis 240 pesan.
- `reports/lidar_test/software_validation_20261030/`: hasil pytest JUnit/JSON.

Direktori `/tmp` bersifat sementara. Tes dapat diulang dengan perintah pada
README dan `test/check_launch.py`; data sintetis tidak boleh dilaporkan sebagai
pengukuran perangkat asli.
