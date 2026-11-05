# Tes mandiri Livox MID-360 di Jetson

Paket ROS 2 Humble ini hanya membaca sensor dan menerbitkan topic
`/lidar_test/*`. Tidak membutuhkan FC, MAVROS, GPS, motor, atau komponen drone.
Driver Livox dijalankan terpisah oleh operator. Paket tidak mengubah jaringan,
firmware, parameter LiDAR, atau file program simulasi.

Yang disediakan:

- Rekaman rosbag2 **pesan asli** point cloud dan IMU, sebelum filter analisis.
- Jarak minimum front/right/left/back dari point cloud terbaru.
- Estimasi orientasi dan kecepatan dengan integrasi IMU, tanpa magnetometer.
- RViz realtime dalam frame sensor; tanpa TF palsu atau peta lokalisasi.
- Log teks, CSV, events JSONL, konfigurasi, dan ringkasan JSON.

## 1. Instalasi dan build

Di Jetson dengan Ubuntu/JetPack yang mendukung ROS 2 Humble, jalankan dari
workspace ini. Perintah berikut adalah instruksi untuk operator; pemasangan
driver atau tes hardware **belum dilakukan** oleh agen.

```bash
sudo apt update
sudo apt install python3-colcon-common-extensions python3-numpy python3-scipy \
  python3-yaml python3-pytest ros-humble-rclpy ros-humble-sensor-msgs-py \
  ros-humble-diagnostic-msgs ros-humble-rosidl-runtime-py \
  ros-humble-rosbag2-py ros-humble-rosbag2-storage-default-plugins \
  ros-humble-ros2bag ros-humble-rviz2
source /opt/ros/humble/setup.bash
colcon --log-base log_lidar_test build --base-paths mid360_test \
  --packages-select mid360_test --build-base build_lidar_test \
  --install-base install_lidar_test --symlink-install
source install_lidar_test/setup.bash
```

Jika driver belum terpasang, ikuti langkah resmi pemasangan Livox-SDK2 lalu
livox_ros_driver2, khusus ROS 2 Humble:

- https://github.com/Livox-SDK/Livox-SDK2
- https://github.com/Livox-SDK/livox_ros_driver2

ROS 2 driver dibangun dengan `./build.sh humble` dari direktori driver di
workspace terpisah. Source `install/setup.bash` workspace driver sebelum tes.
Alamat IP LiDAR/Jetson harus diambil dari perangkat dan konfigurasi nyata;
tidak ada IP atau akses SSH yang diasumsikan di paket ini.

## 2. Pemeriksaan awal tanpa merekam

Hubungkan daya sesuai spesifikasi MID-360 dan Ethernet ke Jetson. Letakkan sensor
tegak pada dudukan stabil. Jaga diam selama kalibrasi awal. Gunakan domain ROS
khusus yang tidak dipakai sesi lain (contoh di bawah: 102), **pada semua terminal
driver, tes, dan RViz**. Jangan source `tools/ros_environment.sh`, karena itu
mengatur domain dan lingkungan simulasi.

```bash
source /opt/ros/humble/setup.bash
# Source workspace driver sesuai lokasi nyata:
source /PATH/WORKSPACE_LIVOX/install/setup.bash
source install_lidar_test/setup.bash
export ROS_DOMAIN_ID=102
export ROS_LOCALHOST_ONLY=1
ip -br address
ros2 pkg prefix livox_ros_driver2
ros2 launch livox_ros_driver2 rviz_MID360_launch.py
```

Launch resmi tersebut mengeluarkan PointCloud2 dan membuka RViz. Sesuaikan
`MID360_config.json` pada driver dengan jaringan nyata terlebih dahulu. Jangan
jalankan dua driver untuk sensor yang sama. Driver varian CustomMsg juga
didukung, dengan pengaturan `cloud_type` yang sesuai.

Di terminal lain dengan environment sama:

```bash
ros2 topic list -t
ros2 topic info /livox/lidar -v
ros2 topic info /livox/imu -v
ros2 topic hz /livox/lidar
ros2 topic hz /livox/imu
ros2 topic echo --once /livox/imu
```

Pastikan nama node sumber, tipe, frame, dan satuan sesuai. Hardware default
menerima tepat satu publisher per topic, bernama `livox_lidar_publisher`.
Jika driver nyata memakai nama berbeda, ubah `allowed_publishers` setelah
memeriksanya. Jangan memasukkan node simulator/replay ke allowlist hardware.

**Penting tentang satuan IMU:** upstream driver Livox yang diperiksa meneruskan
percepatan dalam **g**, walaupun menggunakan pesan `sensor_msgs/Imu`. Karena
itu default `acceleration_unit: g`. Versi driver yang mengonversinya menjadi
m/s² harus memakai `acceleration_unit: m_s2`. Saat diam normanya sekitar 1 g
atau 9,81 m/s². Salah satuan akan menghalangi kalibrasi; jangan mengubah angka
kalibrasi untuk menutupi masalah satuan.

Referensi kode yang diperiksa:

- https://github.com/Livox-SDK/livox_ros_driver2/blob/master/src/comm/pub_handler.cpp
- https://github.com/Livox-SDK/livox_ros_driver2/blob/master/src/lddc.cpp
- https://livox-wiki-en.readthedocs.io/en/latest/tutorials/new_product/mid360/livox_eth_protocol_mid360.html

## 3. Konfigurasi tes

Salin dan edit konfigurasi sesuai hasil pemeriksaan, jangan mengganti label
frame agar data yang salah seolah-olah benar.

```bash
cp mid360_test/config/hardware.yaml /tmp/mid360_hardware.yaml
```

Pengaturan utama:

| Parameter | Arti dan dasar default |
| --- | --- |
| `cloud_topic`, `imu_topic` | Topic driver nyata; default `/livox/lidar`, `/livox/imu` |
| `cloud_type` | `sensor_msgs/msg/PointCloud2` atau `livox_ros_driver2/msg/CustomMsg` |
| `expected_frame`, `expected_imu_frame` | Default `livox_frame`; verifikasi dari pesan dan mounting |
| `sector_half_width_deg: 5.0` | Sektor +/-5° untuk memperoleh sampel di sekitar setiap arah; bukan sinar persis 0° |
| `elevation_min_deg: -5.0`, `elevation_max_deg: 5.0` | Pita mendekati horizontal untuk mengurangi pengaruh lantai/atap; ubah sesuai tes |
| `minimum_range_m: 0.2` | Hindari zona dekat yang presisinya tidak dijamin; bukan self-filter badan drone |
| `maximum_range_m: 70.0` | Batas analisis berdasarkan jangkauan nominal maksimum; bukan jaminan semua permukaan terdeteksi sejauh itu |
| `reject_tag_mask: 0` | Tidak menafsirkan tag vendor sebelum arti bit driver diverifikasi. Raw tag tetap direkam |
| `calibration_seconds: 2.0`, `calibration_min_samples: 100` | Jendela awal bias gyro saat operator memegang sensor diam; nilai awal tes meja |
| `calibration_gyro_limit_rad_s: 0.03` | Gerak rotasi besar menunda kalibrasi; bukan ukuran akurasi heading |
| `calibration_accel_std_m_s2: 0.15` | Menolak jendela kalibrasi yang bergetar; perlu disesuaikan berdasar pengukuran |
| `imu_max_dt_s: 0.1` | Integrasi dibatalkan jika jarak timestamp melampaui ini; tidak mengisi celah dengan gerak buatan |
| `cloud_timeout_s: 0.5`, `imu_timeout_s: 0.1` | Watchdog awal tes meja, bukan batas aman penerbangan |
| `timestamps_synchronized: false` | Default tidak mengklaim umur akuisisi/latency dari dua jam yang belum selaras |
| `imu_cloud_same_clock: true` | Hanya untuk cloud dan IMU dari driver/perangkat yang sama; matikan bila belum terkonfirmasi |
| `sync_tolerance_s: 0.05` | Selisih maksimum timestamp estimasi IMU terdekat dengan cloud; bukan sinkronisasi fisik yang dijamin |
| `expected_cloud_hz: 10.0` | Referensi estimasi pesan terlewat; samakan dengan publish_freq driver |
| `expected_imu_hz: 0.0` | Tidak menebak frekuensi IMU. Masukkan hasil yang benar bila diketahui |
| `logger_hz: 5.0` | Log ringkas disampel 5 Hz; bag tetap merekam setiap pesan yang diterima recorder |
| `device_serial`, `mounting_notes`, `operator_notes` | Isi identitas perangkat, posisi pemasangan, kondisi pengujian |

Konfigurasi disimpan dalam setiap sesi. Batas di atas adalah parameter awal
engineering, bukan klaim akurasi alat atau izin menerbangkan drone.
Spesifikasi: https://www.livoxtech.com/mid-360/specs

## 4. Menjalankan tes hardware

Setelah driver sehat, pada terminal dengan environment yang sama:

```bash
ros2 launch mid360_test hardware_test.launch.py \
  config:=/tmp/mid360_hardware.yaml rviz:=false
```

Gunakan `rviz:=true` bila driver belum membuka RViz dan Jetson memiliki desktop.
Launch mencetak lokasi sesi, default `reports/lidar_test/hardware_<tanggal>_<id>`.
Direktori baru wajib digunakan; sesi lama tidak ditimpa. Jangan membuka dua
launch tes sekaligus pada domain yang sama.

Jika menjalankan node satu per satu untuk diagnosis, parameternya sama:

```bash
ros2 run mid360_test directional_distance --ros-args \
  --params-file /tmp/mid360_hardware.yaml -p output_dir:=/tmp/mid360_individual
```

Node ini memerlukan `cloud_adapter` berjalan untuk `/lidar_test/points`.
Nama executable lain: `cloud_adapter`, `imu_integration`, `health_monitor`,
`test_logger`, `raw_recorder`. Logger dan recorder menolak file/sesi yang telah
dipakai; gunakan direktori baru. Launch lengkap adalah cara yang dianjurkan.

Biarkan sensor diam sampai `Yaw_est` dan `Speed_est` tersedia. Setelah itu
boleh digeser/diputar secara manual di meja atau dibawa dengan aman. Tidak ada
gerak otomatis, reset kecepatan otomatis, atau penggunaan magnetometer.
Hentikan dengan **Ctrl+C**, tunggu recorder menyelesaikan antrean dan metadata.
Mematikan daya/membunuh proses paksa dapat menyisakan rekaman belum final.

### RViz

```bash
ros2 run rviz2 rviz2 -d mid360_test/config/mid360.rviz
```

Fixed Frame default `livox_frame`; ubah ke frame sumber yang benar jika berbeda.
Topic `/lidar_test/points`, QoS **Best Effort**. Sumbu X merah adalah front,
Y hijau left, Z biru up. Right = -Y, back = -X. Periksa dengan objek nyata di
masing-masing sisi; arah konektor tidak diasumsikan. Tidak menerbitkan TF
orientasi hasil integrasi sebagai pose drone.

Decay Time default nol: tampilan scan terbaru, tidak menumpuk titik lama
seolah menjadi peta statis saat sensor bergerak. Hasilnya belum di-deskew.

### Arti log

```text
Time = 12.350 s | Smallest distance: front = 153.2 cm, right = 180.5 cm, left = 220.1 cm, back = N/A | Yaw_est = 12.40 deg | Speed_est = 0.080 m/s | Missing = back | Status = MISSING_RETURNS
```

- `Time`: detik **waktu nyata monotonic** sejak logger dimulai. Timestamp sumber
  tersimpan terpisah pada CSV dan pesan bag.
- Jarak: minimum Euclidean dari pusat sensor, dalam sektor/pita elevasi yang
  dikonfigurasi. Bukan jarak dari bodi drone atau sertifikasi ruang bebas.
- `Missing`: sektor tanpa pantulan valid di scan terbaru. `N/A` bukan nol,
  bukan tak hingga, dan bukan bukti ruang kosong. Semua sumbu ikut sensor.
- `Yaw_est`: sudut relatif hasil integrasi seluruh sumbu gyro; yaw awal nol,
  bukan utara. Tampilan yaw dibungkus pada rentang -180 sampai 180 derajat.
  Pitch/roll awal diestimasi dari gravitasi.
- `Speed_est`: norma vx/vy/vz setelah orientasi digunakan untuk menghilangkan
  gravitasi dan percepatan diintegrasikan. Ini kecepatan titik IMU; tidak ada
  kompensasi lever arm ke pusat optik LiDAR.
- Ketika kalibrasi, time mismatch, dropout atau invalid: estimasi `N/A`.
  Restart sesi saat sensor diam untuk kalibrasi ulang setelah invalid. Jangan
  menyambung lintasan integrasi melewati celah data.
- `STALE_CLOUD`, `INVALID_CLOUD`, `SOURCE_REJECTED`: jarak numerik tidak dipakai.
  `RECORDER_ERROR`: periksa ruang disk, antrean dan `recorder_status.json`.
- `Status = OK` hanya menjelaskan pembacaan sektor saat itu, bukan akurasi
  integrasi. Kolom `imu_state` dan alasan di events menjelaskan kondisi IMU.

CSV menyimpan jumlah titik front/right/left/back (bukan jumlah objek), yaw,
pitch, roll, vx/vy/vz, lama integrasi, dan selisih timestamp pencocokan IMU-cloud.
Logger memilih estimasi IMU terdekat dalam toleransi, tidak sekadar mengambil
nilai terakhir. Timestamp ini mengacu header scan, bukan deskew setiap titik.

## 5. File hasil dan pemeriksaan kelengkapan

```text
<sesi>/
  raw_bag/metadata.yaml + raw_bag_0.db3
  readings.log / readings.csv
  events.jsonl
  session.json / launch_session.json
  health.json / recorder_status.json / summary.json
```

`raw_bag` menyimpan serialized message asli (CustomMsg atau PointCloud2 serta
Imu), termasuk field invalid, tag, intensity, waktu titik jika ada pada driver.
Adapter CustomMsg hanya membuat salinan untuk RViz; timebase/lidar_id asli tetap
di bag. Tidak merekonstruksi sinar kosong atau menghapus titik raw.

Yang tidak tersedia pada pesan ROS—misalnya paket UDP yang hilang sebelum
driver, status firmware yang tidak dipublikasikan, dan informasi sinar yang
tidak dikeluarkan driver—tidak dibuat-buat atau diklaim telah direkam.

Recorder memakai antrean terbatas dan worker disk. QoS sensor Best Effort
mendukung publisher Best Effort maupun Reliable, tetapi **bukan jaminan tanpa
kehilangan pesan**. Overflow/disk error dicatat sebagai FAIL, bukan disembunyikan.
Periksa `closed: true`, `dropped_by_recorder: 0`, dan hitungan kedua topic pada
`recorder_status.json` setelah berhenti. File ini merupakan status recorder
terakhir; `summary.json` adalah snapshot periodik logger yang mungkin mendahului
penutupan recorder. Pastikan metadata bag tersedia:

```bash
ros2 bag info /PATH/SESI/raw_bag
```

Ringkasan PASS berlaku hanya untuk integritas aliran yang diterima dan recorder,
bukan kelulusan hardware menyeluruh. Sensor tanpa data = NOT TESTED, input invalid,
timestamp tidak urut, timeout, source ditolak, disk error/overflow = FAIL.
Frekuensi diukur; expected_hz hanya referensi perkiraan pesan terlewat, belum
gate frekuensi. Hilangnya paket Ethernet tidak dapat diukur dari header ROS
tanpa sequence counter yang sesuai. Missing return bukan otomatis dropout.

P50/p95 gap/umur/waktu proses memakai maksimal 4096 sampel terakhir; maksimum
gap seluruh sesi dicatat terpisah. Receive Hz berdasarkan awal-akhir penerimaan.
Source age/latency tetap kosong jika jam belum dikonfirmasi sinkron.
Waktu callback perhitungan jarak dicatat pada health cloud di topic distances;
pipeline planner/kontrol/FCU tidak ada dan berstatus belum diuji.

## 6. Checklist tes meja

1. **Koneksi/data:** satu driver, cloud+IMU ada, tipe/frame/satuan benar, tidak
   ada `/clock` pada domain hardware. Tidak ada data berarti belum lulus.
2. **Empat arah:** papan dipindah ke depan/kanan/kiri/belakang; sektor yang
   bersangkutan harus berubah. Jumlah titik dan elevasi diperiksa di CSV/RViz.
3. **Jarak:** ukur dari pusat optik sesuai diagram perangkat, lalu bandingkan
   dengan jarak papan di log. Hindari sisi papan/objek lain yang lebih dekat
   pada sektor. Catat referensi manual dan ketidakpastian pada operator_notes.
   Tidak ada ambang error otomatis yang mengklaim akurasi tanpa referensi.
4. **Diam:** kalibrasi selesai, catat yaw/speed semu selama interval yang
   diketahui. Tidak menganggap nilai nol sebagai jaminan IMU bebas bias.
5. **Putar/geser terukur:** bandingkan rotasi dengan busur derajat/acuan mandiri;
   estimasi kecepatan hanya eksperimen integrasi, bukan pengukuran acuan.
6. **Perekaman:** stop normal, periksa recorder, bag info, lalu replay.

Setiap sesi mencatat waktu UTC, konfigurasi, kode Git jika tersedia, hash Python,
versi paket driver jika tersedia, domain, identitas perangkat yang diisi operator,
dan masalah. Perangkat asli belum diuji pada implementasi ini.

## 7. Replay, tanpa perangkat

Gunakan domain **berbeda** dari hardware. Jalankan analisis dahulu, baru player,
agar kalibrasi awal dalam rekaman tidak terlewat. Jangan menjalankan driver
hardware di domain replay.

```bash
export ROS_DOMAIN_ID=103
export ROS_LOCALHOST_ONLY=1
source /opt/ros/humble/setup.bash
source /PATH/WORKSPACE_LIVOX/install/setup.bash  # diperlukan untuk CustomMsg
source install_lidar_test/setup.bash
ros2 launch mid360_test replay_test.launch.py rviz:=true
```

Terminal kedua dengan environment/domain sama:

```bash
ros2 bag play /PATH/SESI/raw_bag --clock --rate 1.0
```

Untuk CustomMsg, salin replay.yaml dan atur cloud_type sesuai bag. Replay juga
merekan ulang pesan yang diterima ke sesi baru untuk verifikasi; tidak menimpa
bag sumber. Pause, seek, loop, playback lambat, atau awal rekaman saat sensor
sudah bergerak dapat membuat integrasi invalid. Mulai ulang analisis dan bag
dari awal yang diam; jangan menganggap replay sebagai bukti tes hardware baru.
Hardware menolak use_sim_time=true, keberadaan topic `/clock`, nama input
simulasi/mock, publisher tidak diizinkan, dan publisher ganda. Ini guard terhadap
kesalahan konfigurasi, bukan autentikasi asal data: node yang sengaja menyamar
sebagai driver tetap memerlukan pemeriksaan operator.

## 8. Pengujian software

```bash
PYTHONDONTWRITEBYTECODE=1 PYTHONPATH=mid360_test python3 -m pytest -q \
  -p no:cacheprovider mid360_test/test/test_core.py
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=192 ROS_LOCALHOST_ONLY=1 ROS_LOG_DIR=/tmp/mid360_test_ros_logs
PYTHONDONTWRITEBYTECODE=1 PYTHONPATH=mid360_test:$PYTHONPATH python3 -m pytest -q \
  -p no:cacheprovider mid360_test/test/test_ros.py
```

Tes ROS menggunakan publisher sintetis pada mode replay, file sementara dan
domain terpisah. Menguji data mengalir hingga logger, pembukaan kembali rosbag,
pelestarian semua field pesan, penolakan waktu simulasi pada mode hardware,
serta adapter CustomMsg. Bukan tes perangkat asli atau benchmark Jetson.

Untuk mengulangi tes launch antarp­roses setelah paket dibangun/source overlay:

```bash
python3 mid360_test/test/check_launch.py
```

Skrip memakai domain lokal 194 dan direktori `/tmp` baru, menerbitkan data
sintetis selama sekitar empat detik, lalu memeriksa log dan penutupan recorder.
Pastikan domain tersebut kosong sebelum menjalankannya. Tidak membuka RViz.

Integrasi gyro+accelerometer tanpa koreksi eksternal akan drift. Kalibrasi awal
tidak mampu membedakan gerak berkecepatan konstan dari diam, atau mengidentifikasi
seluruh bias accelerometer dari satu orientasi. Tidak ada klaim estimasi posisi,
ketepatan yaw absolut, deteksi seluruh rintangan, maupun kesiapan terbang.
