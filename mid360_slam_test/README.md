# Pengujian manual SLAM MID-360 di Jetson

Paket baru pada branch `lidar_test`, khusus tes darat. Drone tidak diterbangkan,
tidak di-arm, propeller dilepas, dan motor harus tidak dapat aktif. Program
tidak menggunakan MAVROS, FCU, GPS atau ground truth simulator. Gunakan Jetson
melalui NoMachine untuk melihat RViz, dengan dua terminal terpisah untuk log dan
menu pengujian. Driver Livox berjalan di terminal ketiga.

**Backend yang disertakan adalah eksperimen**: ICP point-to-point dengan prediksi
rotasi IMU, keyframe, pencarian loop berdasarkan kedekatan pose, pencocokan dua
arah, dan optimasi pose graph SE(3). Ini bukan implementasi FAST-LIO/GLIM atau
LIO tightly coupled. Tidak ada deskew per titik, descriptor place recognition,
relocalisation global, atau covariance terkalibrasi. Posisinya adalah posisi
**pusat LiDAR**, belum pusat badan drone. Ini implementasi awal yang dapat
dijalankan/dievaluasi, bukan bukti akurasi hardware ataupun izin terbang.

## Alur dan node

```text
driver Livox /livox/lidar → cloud_adapter (mid360_test) → /lidar_test/points
                                                        ↓
                                              cloud_preprocessor
                                                        ↓ /slam_test/points
/livox/imu ───────────────────────────────────→ lidar_imu_estimator
                                                        ↓
                                     odometry lokal + pose map + loop events
                                                        ↓
test_controller (menu) ─→ commands/ACK ──────────→ test_evaluator
                                                        ↓
                                             test_monitor + RViz
```

`slam_recorder` merekam kedua sumber raw dan topic hasil; jangan jalankan launch
`mid360_test` bersamaan karena adapter/topic analisisnya akan ganda. Satu driver,
satu sesi tes, satu menu kontrol dalam satu domain ROS.

| Topic | Tipe | Arti |
| --- | --- | --- |
| `/livox/lidar` | CustomMsg atau PointCloud2 | Pesan asli perangkat, direkam tanpa filter |
| `/livox/imu` | Imu | Gyro/accelerometer asli, direkam tanpa konversi |
| `/lidar_test/points` | PointCloud2 | Salinan driver untuk analisis, semua field PC2 dipertahankan |
| `/slam_test/points` | PointCloud2 | XYZ finite, range/box filter dan downsample untuk ICP |
| `/slam_test/odometry_local` | Odometry | Odometry kontinu, tidak dikoreksi oleh loop |
| `/slam_test/pose_map` | PoseStamped | Pose saat ini setelah koreksi graph |
| `/slam_test/estimator_status` | String JSON | Validitas tracking, kualitas ICP, timing, batas backend |
| `/slam_test/loop_events` | String JSON | REJECTED/ACCEPTED beserta bukti registrasi dan optimasi |
| `/slam_test/path_local` | Path | Riwayat pose lokal terukur, bounded 20.000 pose |
| `/slam_test/path_map` | Path | Keyframe yang direvisi menggunakan graph terbaru |
| `/slam_test/registered_points` | PointCloud2 | Scan terbaru ditransform ke map setelah registrasi |
| `/slam_test/map_points` | PointCloud2 | Peta keyframe terkoreksi untuk RViz |
| `/slam_test/reference_markers` | MarkerArray | Penanda awal dan target yang dimasukkan operator |
| `/slam_test/commands`, `/slam_test/command_ack` | String JSON | Perintah menu dengan ID dan konfirmasi penerimaan |
| `/slam_test/test_state` | String JSON | Tahap, fase, hasil, dan kesiapan memulai |
| `/tf` | TFMessage | Hanya `slam_map → slam_odom → slam_lidar` |

`slam_odom` dan `slam_map` memakai sumbu awal yang dilevelkan terhadap gravitasi,
yaw awal lokal nol. `slam_lidar` memiliki sumbu LiDAR. Tidak ada TF palsu ke
`base_link`, tidak menimpa TF sistem lain. Pose yang hilang/basi tidak diterbitkan
sebagai pose baru. Covariance odometry diberi diagonal -1 sebagai penanda
**konvensi paket ini saja** untuk unknown; jangan diberikan ke FCU atau consumer
yang menganggapnya covariance fisik. Lihat `covariance_available: false` pada
status; belum tersedia estimasi ketidakpastian statistik.

## Instalasi dan build

Salin **kedua folder** `mid360_test` dan `mid360_slam_test` ke workspace Jetson.
Lokasi contoh berikut `~/mid360_ws`; sesuaikan dengan lokasi sebenarnya. Tidak
ada IP/SSH baru yang diasumsikan atau pemasangan remote yang dijalankan agen.
Dependensi Livox-SDK2/driver harus sudah terpasang seperti pada README mid360_test.

```bash
cd ~/mid360_ws
source /opt/ros/humble/setup.bash
source /home/palmbee1/palmbee_ws/src/ws_livox/install/setup.bash
sudo apt install python3-colcon-common-extensions python3-numpy python3-scipy \
  python3-yaml python3-pytest ros-humble-sensor-msgs-py ros-humble-nav-msgs \
  ros-humble-geometry-msgs ros-humble-diagnostic-msgs ros-humble-tf2-ros \
  ros-humble-visualization-msgs ros-humble-tf2-msgs \
  ros-humble-rosbag2-py ros-humble-rosbag2-storage-default-plugins \
  ros-humble-ros2bag ros-humble-rviz2
colcon --log-base log_slam_test build --base-paths mid360_test mid360_slam_test \
  --packages-select mid360_test mid360_slam_test \
  --build-base build_slam_test --install-base install_slam_test --symlink-install
source install_slam_test/setup.bash
```

Gunakan versi NumPy/SciPy apt Jetson/ROS Humble; tidak perlu library optimasi C++
atau GPU tambahan untuk backend awal ini. Kinerja real-time Jetson belum diuji.

## Konfigurasi sebelum perangkat digerakkan

```bash
cp mid360_slam_test/config/hardware.yaml ~/mid360_slam_hardware.yaml
nano ~/mid360_slam_hardware.yaml
```

Periksa pengaturan berikut berdasarkan data/perangkat nyata:

- `cloud_type`: default CustomMsg sesuai `msg_MID360_launch.py`. Untuk driver
  PointCloud2 gunakan `sensor_msgs/msg/PointCloud2`.
- Nama topic, nama publisher driver tunggal, dan frame cloud/IMU harus sesuai
  hasil `ros2 topic info -v` dan pesan asli. Default `livox_frame`.
- `acceleration_unit: g` sesuai kode upstream driver yang diperiksa. Jika driver
  nyata mengonversi ke SI, gunakan `m_s2`. Jangan mengubah ambang kalibrasi untuk
  menyembunyikan salah satuan.
- `imu_lidar_rotation_rpy_deg`: rotasi terukur IMU ke frame LiDAR. Nol hanya jika
  sumbu keluaran benar-benar sejajar. IMU diputar ke frame LiDAR secara numerik,
  bukan sekadar diganti labelnya. Tidak ada kompensasi lever arm translasi IMU.
- `mounting_confirmed: true` **setelah pemeriksaan** unit, frame, dan sumbu. Launch
  hardware sengaja menolak mulai jika konfirmasi konfigurasi belum diisi.
  Sensor harus kaku terhadap dudukan; mekanisme pitch/roll, jika ada, dipertahankan
  tetap selama tes. Program tidak mengasumsikan mount bergerak atau menggerakkannya.
- `exclusion_boxes_json`: default `[]` (tanpa mengarang dimensi drone). Contoh
  sintaks saja: `'[{"min":[-0.2,-0.2,-0.3],"max":[0.2,0.2,-0.1]}]'`.
  Isi hanya volume rangka/pegangan yang diukur dalam frame sensor. Jangan pakai
  radius besar untuk menghapus badan operator dan menghilangkan objek nyata.
- `imu_cloud_same_clock` hanya true jika cloud/IMU benar-benar dari clock driver
  yang sama. `timestamps_synchronized` default false: tidak mengklaim latency
  akuisisi dengan mengurangkan jam perangkat dan host yang berbeda.
- Isi serial, mounting_notes, operator_notes, dan ketidakpastian pengukuran.

Gates algoritma pada YAML adalah **heuristik awal tes meja**, bukan angka aman
atau spesifikasi akurasi. Contohnya overlap .55, residual ICP .12 m, kandidat loop
radius 1 m, minimum 12 keyframe/4 m perjalanan/10 detik. Loop perlu overlap dua
arah .75 dan residual .08 m; koreksi besar di luar gate ditolak. Default keyframe
maksimal 160 untuk membatasi memori/optimasi. Batas tersebut mungkin menghalangi
closure yang benar atau menerima kecocokan salah; wajib ditinjau pada log/peta.
Loop candidate dicari berdasarkan posisi estimasi, sehingga drift > radius
pencarian tidak akan dipulihkan secara global.

**Batas kelulusan akurasi default -1 (belum ditentukan).** Ukuran error tetap
dihitung, tetapi hasil NOT TESTED jika tidak ada gate yang dikonfigurasi. Isi
`max_position_error_m`, `max_angle_error_deg`, `max_stationary_drift_m`, dan
`max_stationary_orientation_deg` sesuai kebutuhan dan ketelitian acuan ukur.
Isi `reference_uncertainty_m/deg` jika diketahui. PASS berarti hanya gates yang
ditampilkan dalam hasil terpenuhi; bukan sertifikasi seluruh kemampuan SLAM.

## Menjalankan melalui NoMachine

Pada **semua terminal** gunakan environment/domain sama, contoh domain 102:

```bash
source /opt/ros/humble/setup.bash
source /home/palmbee1/palmbee_ws/src/ws_livox/install/setup.bash
source ~/mid360_ws/install_slam_test/setup.bash
export ROS_DOMAIN_ID=102
export ROS_LOCALHOST_ONLY=1
```

1. Terminal driver: jalankan **satu** `ros2 launch livox_ros_driver2 msg_MID360_launch.py`.
   Pastikan cloud+IMU benar-benar terbit sebelum tahap berikutnya. Profil Ethernet
   `mid360-test` pada Jetson pengguna sebelumnya memakai host 192.168.1.5 dan
   LiDAR 192.168.1.163; verifikasi lagi pada perangkat saat digunakan. Jangan
   menjalankan MID360s/driver workspace lain bersamaan.
2. Letakkan drone diam di dudukan stabil, propeller dilepas. Gunakan operator
   kedua untuk menu saat drone dibawa; jangan mengetik sambil memegang drone.
3. Terminal log:

```bash
cd ~/mid360_ws
ros2 launch mid360_slam_test hardware_test.launch.py \
  config:=$HOME/mid360_slam_hardware.yaml rviz:=true
```

4. Tunggu kalibrasi IMU dan status TRACKING. Terminal kontrol:

```bash
ros2 run mid360_slam_test test_controller
```

Menu menyediakan Enter mulai/selesai, `e` edit referensi, `n` berikutnya, `r`
ulangi, `s` lewati, `p` perbarui, `q` akhiri sesi. Enter pada READY meminta
konfirmasi `ya` bahwa referensi fisik diukur dan operator siap. Pesan ACK
mengonfirmasi perintah diterima; jangan bergerak sebelum ACK berhasil.

Tidak ada peralihan ke gerak otomatis. Fase:
`READY → RUNNING → FINISHING/WAIT_LOOP → REVIEW → READY berikutnya`.
Tes diam selesai berdasarkan durasi. Perpindahan/rotasi selesai setelah operator
menekan Enter dan estimasi kembali stabil. Stabilisasi awal/akhir menggunakan
jendela pose lokal yang segar; bukan bukti operator berada tepat di penanda.
Recorder tetap merekam ketika persiapan dan antar tahap.

Loop menunggu `loop_wait_s` setelah operator menandai sudah kembali. Kejadian
loop yang diterima selama perjalanan tahap juga dicatat, karena pengenalan bisa
terjadi sedikit sebelum kaki operator tepat di penanda. Delay terhadap tanda
kembali bisa negatif. Untuk tahap ini harus ada accepted event sejak mulai,
tidak boleh memakai loop lama dari tahap sebelumnya. Tanpa accepted event tes
deteksi loop FAIL, meskipun posisi akhir mendekati nol.

## Tahap dan prosedur ukur

| Tahap | Referensi default dan prosedur |
| --- | --- |
| stationary | Diam di dudukan 30 detik; jangan dipegang |
| forward/backward | Perpindahan +1/-1 m pada X saat tahap dimulai; jalur diberi penanda |
| left/right | +1/-1 m pada Y sensor; Y positif kiri |
| up/down | +0.5/-0.5 m pada Z sensor; pertahankan sensor level |
| rotation | +360° berlawanan jarum jam dilihat dari atas; boleh edit -360 untuk searah |
| loop_closure | Keliling pohon/ruangan, kembali ke posisi, tinggi DAN orientasi awal |

Default jarak adalah contoh target yang **harus dikonfirmasi/diubah berdasarkan
penanda fisik**, bukan hasil yang diambil dari SLAM. Perpindahan dihitung relatif
orientasi sensor saat tahap dimulai; sensor sebaiknya level saat pengukuran arah
horizontal/vertikal. Acuan evaluasi tahap di-reset, estimator/peta **tidak**.
Tidak wajib kembali ke origin global antartahap, tetapi pengulangan menggunakan
penanda yang sama membuat hasil lebih dapat dibandingkan.

Rotasi memakai accumulated yaw yang di-unwrap (bukan hanya yaw akhir -180..180).
Pergerakan yang terlalu cepat atau celah pose dapat membuat akumulasi ambigu;
tracking/dropout/reset menggagalkan tahap. Untuk putaran terukur gunakan dudukan
putar; mengayunkan drone menghasilkan perpindahan pusat LiDAR yang nyata.

Loop keliling pohon: pilih lingkungan dengan beberapa objek tetap yang berbeda,
bukan hanya batang simetris/daun bergerak. Jaga tumpang tindih pengamatan dan
kecepatan rendah. Jangan menjanjikan closure di kebun dengan pohon mirip; matching
ICP lokal dapat salah mengasosiasikan lingkungan berulang. Periksa peta dan
referensi independen setelah koreksi.

## Tampilan realtime

```text
Time = 25.2 s | Test = forward | Phase = RUNNING | Tracking = TRACKING | Position = [0.620, 0.010, 0.000] m | RPY = [0.0, 0.1, 0.2] deg | Loop count = 0 | Reason = registered
Loop = ACCEPTED | Keyframes = 35->1 | Reason = registered_and_graph_optimised
```

Angka contoh bukan hasil hardware. CSV menambahkan kecepatan, perpindahan tahap,
akumulasi yaw, pose map dan timing. RPY adalah orientasi LiDAR; belum drone body.
Kecepatan diperoleh dari beda pose LiDAR, bukan integrasi percepatan tanpa koreksi.
IMU hanya dipakai untuk inisialisasi gravitasi/bias dan prediksi rotasi ICP.

RViz pada desktop Jetson: Fixed Frame `slam_map`; scan terbaru cyan, peta keyframe,
lintasan lokal oranye dan lintasan keyframe terkoreksi hijau, TF sensor serta
penanda operator. Peta/path keyframe diperbarui setiap 2 detik default, scan
terdaftar diterbitkan setiap hasil estimasi. Raw cloud tetap tersedia untuk
debug frame sensor. GUI belum diverifikasi visual pada Jetson.

Ketika LOST, estimasi dilatch; bukan relocalisation otomatis. Hentikan sesi,
periksa masalah, mulai sesi baru dari keadaan diam. Jangan meneruskan tahap
dengan pose lama. Pelanggaran timing/sumber/kalibrasi ditulis pada reason.

## Rekaman dan evaluasi hasil

Folder baru `reports/slam_test/hardware_<waktu>_<id>`:

```text
raw_bag/                    # pesan asli, hasil, TF, perintah dan ACK
configuration.yaml          # salinan config yang dipakai
code_manifest.json          # versi Git jika ada dan hash source
slam_session.json
slam_readings.log / slam_readings.csv
test_events.jsonl / test_results.json
loop_events.jsonl / recorder_status.json
resources.jsonl             # load, memory, thermal dan CPU ticks host
runtime_summary.json        # p50/p95/max timing dari jendela maksimal 4096 sampel
estimator_final.json
keyframe_poses.npz           # local + map, timestamp
keyframe_clouds.npz / map_points.npy
```

`q` pada menu menutup pengujian tahap; kemudian Ctrl+C pada terminal launch
untuk menutup recorder dan menyimpan peta. Jangan mematikan proses paksa jika
masih menulis. RawRecorder tidak menjamin hilangnya UDP sebelum driver atau DDS
sebelum callback dapat dideteksi. Overflow/error disk = FAIL, tidak disembunyikan.
Periksa `closed`, `dropped_by_recorder`, dan jumlah pesan sumber pada status
recorder. Bag asli menyimpan invalid return; filter hanya pada salinan analisis.

Evaluasi mencatat error vektor perpindahan terhadap referensi, perubahan posisi
dan orientasi maksimum/RMS saat diam, endpoint drift m/menit, accumulated yaw
dan error sudut. Loop mencatat local endpoint error **sebelum koreksi graph**,
map endpoint setelah koreksi, event/correction dan optimasi. Start anchor map
direvisi menggunakan graph solusi yang sama dengan endpoint supaya tidak
membandingkan dua revisi peta berbeda. Koreksi loop dapat memengaruhi pose lama.
ACCEPTED berarti lulus gate geometri dan optimasi, **bukan bukti independen**
bahwa tempatnya benar. Endpoint yang dekat nol sendiri tidak menjadi event loop.

Semua percobaan ulang disimpan; status keseluruhan menggunakan percobaan terakhir
per tahap, dengan riwayat gagal tetap tersedia. Lewati = NOT TESTED. Tanpa data
tidak boleh PASS. Akurasi seluruh lintasan/ATE/RPE tidak dihitung karena belum ada
ground truth kontinu. Referensi manual hanya menguji titik akhir dan kondisi diam.

Tidak ada planner/kontrol/FCU pada tes ini. Timing `processing_ms` adalah backend
ICP/graph, queue adalah antrean sejak cloud preprocessed diterima, preprocessing
terpisah. Ini bukan latency sensor-ke-gerak drone. Clock source vs wall tetap
dibedakan; CPU ticks/load adalah agregat host, bukan utilisasi per node.
CPU utilization dihitung dari selisih tick host antarsampel; mencakup seluruh
aplikasi termasuk RViz/NoMachine. Timing peta visual/serialisasi hasil tidak
termasuk pada timer backend; jangan menjumlahkan maksimum tahap yang tidak berkorelasi.

## Replay

Gunakan domain lain dan config replay yang cocok tipe pesan bag. Launch analisis
dahulu lalu **putar hanya topic sumber**, agar keluaran SLAM lama tidak bercampur
dengan estimator baru:

```bash
export ROS_DOMAIN_ID=104
export ROS_LOCALHOST_ONLY=1
ros2 launch mid360_slam_test replay_test.launch.py rviz:=true
```

Terminal kedua, environment sama:

```bash
ros2 bag play /PATH/SESI/raw_bag --clock --rate 1.0 \
  --topics /livox/lidar /livox/imu
```

Replay dapat dianalisis ulang melalui menu, tetapi input operator replay bukan
pengukuran fisik baru. Pause/seek/loop dapat menggagalkan integrasi dan tahap;
restart dari awal yang diam. Rekaman awal harus memuat kalibrasi diam.

## Tes software dan keterbatasan

```bash
PYTHONDONTWRITEBYTECODE=1 PYTHONPATH=mid360_test:mid360_slam_test \
  python3 -m pytest -q -p no:cacheprovider mid360_slam_test/test/test_estimator.py \
  mid360_slam_test/test/test_evaluation.py
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=195 ROS_LOCALHOST_ONLY=1 ROS_LOG_DIR=/tmp/mid360_slam_ros_logs
PYTHONDONTWRITEBYTECODE=1 PYTHONPATH=mid360_test:mid360_slam_test:$PYTHONPATH \
  python3 -m pytest -q -p no:cacheprovider mid360_slam_test/test/test_slam_ros.py
```

Smoke test **paket terpasang, antarproses** setelah source install overlay:

```bash
python3 mid360_slam_test/test/check_launch.py
```

Script tersebut hanya menggunakan fixture sintetis mode replay pada domain
localhost 196, menyimpan hasil di `/tmp`, menjalankan tahap diam dan menguji
loop dari lintasan sintetis. Pastikan domain 196 tidak dipakai sesi lain. Script
tidak membuka GUI atau mengakses LiDAR asli.

Tes data sintetis memeriksa ICP rigid motion, rotasi penuh, loop benar-benar
dioptimasi, penolakan loop palsu/geometri buruk, graph correction terpisah dari
odom, perpindahan referensi lokal, timestamp, reset, dropouts, repeat/skip,
ACK idempotent, dan sensor→hasil→bag. Bukan bukti akurasi perangkat asli.

No deskew berarti gerak di dalam scan dapat mengaburkan objek dan merusak matching.
Tidak ada rejection operator dinamis/daun bergerak secara semantik. Geometri
planar atau berulang dapat degenerat meski residual ICP kecil. ICP covariance
belum tersedia, reset tracking otomatis tidak didukung. Kondisi ini dicatat
sebagai batas, bukan diganti dengan ground truth atau pose sintetis pada hardware.

Dasar implementasi: hubungan odometry edge/loop constraint/pose graph pada
[Open3D multiway registration](https://www.open3d.org/docs/release/tutorial/pipelines/multiway_registration.html),
optimasi [SciPy least_squares](https://docs.scipy.org/doc/scipy/reference/generated/scipy.optimize.least_squares.html).
Implementasi lokal tidak mereproduksi klaim performa metode lain. Hasil software
dan tahap hardware yang belum diuji ada pada `VALIDATION.md`.
