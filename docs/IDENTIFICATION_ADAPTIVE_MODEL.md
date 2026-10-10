# Identifikasi respons dan model adaptif — simulasi saja

Misi terpisah: takeoff 2,4 m → gerakan kecil X/Y/Z dan yaw → kembali ke titik
hover → LAND → disarm. Tidak ada penyemprotan atau perubahan PID ArduPilot.
Arena kosong: LiDAR, laser rangefinder, kamera, dan proses mapping tidak
dijalankan. IMU, ground-truth ExternalNav, dan fisika/aktuasi ArduPilot tetap
aktif. Lantai dan batas area uji diketahui dari konfigurasi simulasi;
ini bukan peta yang diamati LiDAR dan bukan validasi obstacle avoidance.

Verifikasi 10 Oktober 2026: 135 tes lulus. Pemeriksaan ROS pada run
`ros_ground_truth_identification_20261010_052724_22332_identification/setup_ready.json`
di bawah `reports/` lulus: LiDAR dan mapping tidak aktif, satu publisher
setpoint akhir, IMU/lokalisasi segar, serta frame FCU selaras. Run ini tetap
unarmed dan dihentikan. Sesudahnya penerbangan train dan validasi tanpa
LiDAR selesai; lihat [hasil terukur dan keterbatasan](IDENTIFICATION_RESULTS.md).
Run sebelumnya memakai mapping dan berhasil takeoff, kemudian abort karena
`stale_integrated_map`; dataset parsialnya bukan data fit yang valid.
`train` mempunyai 100 detik gerakan uji; `validation` 43 detik dengan gerakan
berbeda. Waktu takeoff/landing tidak termasuk. Faktor simulasi 0,2 berarti
100 detik simulasi memerlukan sekitar 500 detik di laptop.

## Mengapa bukan AUTOTUNE?

[AUTOTUNE ArduPilot](https://ardupilot.org/copter/docs/autotune.html) menyetel
pengendali sikap. Di sini kita mengidentifikasi **respons tertutup**: bagaimana
drone dengan pengendali ArduPilot yang tetap merespons perintah posisi,
kecepatan, dan percepatan (PVA). Model kecil per sumbu adalah:

```
p_dot = v
v_dot = kp*(p_cmd_delayed-p) + kv*(v_cmd_delayed-v) + ka*a_cmd_delayed + bias
```

`kp/kv/ka` adalah koefisien model respons, **bukan PID yang ditulis ke FCU**.
`bias` mewakili sisa gangguan lokal. Model hanya sekitar hover dan gerakan
kecil; bukan model aerodinamika, massa atau inersia lengkap. Latensi yang
diukur mencakup jalur ROS/MAVROS/FCU, bukan hanya motor.

## Adaptasi yang sudah ada

RLS (*recursive least squares*, estimasi koefisien dari sampel terbaru)
berjalan terpisah dari kontrol, sekitar 20 Hz. Hasil dan alasan menerima/
menolak pembaruan diterbitkan di `/identification/model_candidate`, 2 Hz.
Parameter hanya diperbarui jika data sehat dan gerakan cukup informatif.
Hover tanpa variasi, input yang hampir identik, data terlambat, lonjakan
percepatan, dan hilangnya lokalisasi membekukan pembaruan. Batas parameter,
laju perubahan, kovarians, dan pemeriksaan kestabilan model linear diterapkan.

Mode saat ini **shadow_only**: model diperbarui real time dan dicatat, tetapi
tidak mengubah perintah penerbangan. Belum ada NMPC, adaptive PID, atau
pemuatan otomatis model hasil fit ke pengendali penerbangan. Observer dapat
memakai seed dari `adaptive_model_file` setelah validasi independen lulus dan
envelope (batas kecepatan/interface/payload) cocok. Jika file belum ada,
observer memakai koefisien nominal dan delay nol; hasil offline mencari delay
0–250 ms per sumbu. Delay seed diterapkan secara kausal pada pasangan data
perintah/respons; delay tidak diadaptasi online.
Kestabilan model kandidat bukan bukti kestabilan sistem adaptive control.

Alasan desain:

- [Hanover dkk., Adaptive Nonlinear MPC](https://arxiv.org/abs/2109.04210)
  mendukung gagasan mengompensasi ketidakpastian, angin, dan payload secara
  online. Implementasi ini **bukan L1-NMPC** dari paper tersebut.
- [Goel dkk., RLS dan hilangnya persistensi](https://arxiv.org/abs/2003.03523)
  membahas masalah pembaruan ketika data kurang informatif. Kita memakai
  forgetting tetap dengan penghentian pembaruan dan pembatas kovarians,
  **bukan variable-direction forgetting** paper tersebut.

Cairan berkurang memang alasan masuk akal untuk adaptasi. Namun perubahan
massa tidak bisa dibedakan secara unik dari angin dan kompensasi FCU hanya
dari PVA. Karena itu `estimated_mass_kg=null`. Untuk drone nyata diperlukan
model thrust/aktuasi atau informasi massa/debit tambahan, serta pengujian
stabilitas dan keselamatan. Simulasi sekarang tidak mengurangi massa cairan.

## Jalankan dua penerbangan terpisah

Hentikan simulasi sebelumnya. Build sekali setelah mengubah kode:

```bash
cd /home/abin/polinasi_lidar
bash tools/build_ros.sh
```

Terminal 1, tidak melakukan arming otomatis:

```bash
cd /home/abin/polinasi_lidar
bash tools/start_identification.sh train
```

Salin direktori `reports/..._identification` yang dicetak launcher ke `RUN_DIR`.
Terminal 2:

```bash
cd /home/abin/polinasi_lidar
source tools/ros_environment.sh
RUN_DIR=/home/abin/polinasi_lidar/reports/NAMA_RUN_YANG_DICETAK
.venv/bin/python tools/set_origin.py --timeout 120
.venv/bin/python tools/capture_sitl_parameters.py --output "$RUN_DIR/parameters_before.json"
python3 tools/check_ros_simulation.py --output "$RUN_DIR/setup.json"
```

Hanya jika `passed=true`, mulai dan pantau:

```bash
python3 tools/start_mission.py
ros2 topic echo /navigation/status
```

Status mencakup `identification_phase`, `protocol_completed`, `armed`,
`failure`, dan `adaptive_model_mode`. Tunggu `mission=COMPLETE` dan
`armed=false`; **jangan hentikan SITL sebelum menyimpan parameter sesudah**:

```bash
.venv/bin/python tools/capture_sitl_parameters.py --output "$RUN_DIR/parameters_after.json"
```

Ctrl+C Terminal 1. Jalankan lagi dengan `bash tools/start_identification.sh
validation`, lalu ulangi seluruh Terminal 2 dengan direktori run baru.
Jangan kirim mission-start lagi setelah abort; periksa penyebab dan restart.

Setelah kedua run selesai, simpan path aktual dan fit model kandidat:

```bash
source tools/ros_environment.sh
TRAIN_DIR=/home/abin/polinasi_lidar/reports/NAMA_RUN_TRAIN
VALIDATION_DIR=/home/abin/polinasi_lidar/reports/NAMA_RUN_VALIDATION
python3 tools/fit_identification_model.py \
  --train "$TRAIN_DIR/identification.json" \
  --validation "$VALIDATION_DIR/identification.json" \
  --output reports/identification_model_candidate.json
```

Untuk menyimpan model yang lulus sebagai seed **observer simulasi**, tambahkan
`--install-shadow-seed polinasi_nav/config/identified_pva_model.json` pada
perintah fit. Preset identifikasi menunjuk file tersebut; restart diperlukan.
Model gagal tidak dipasang, dan file yang ada tidak ditimpa. Ini tidak
mengaktifkan adaptive flight control atau mengubah misi survei.

Alat menolak overwrite. Gunakan nama output baru untuk percobaan baru.

## Isi dan arti hasil

- `identification.json`: perintah PVA akhir dalam map ENU, posisi/kecepatan
  respons Gazebo lewat lokalisasi ground-truth, IMU, fase, parameter kandidat
  online, dan penanda lengkap/parsial. Frame dikonversi secara numerik,
  bukan hanya mengganti nama. Tidak ada publisher kontrol dari recorder.
- `parameters_before/after.json`: parameter FCU; fitter menolak perubahan
  ATC/PSC/WPNAV/MOT atau perbedaan konfigurasi antar penerbangan.
- Model kandidat: koefisien, delay, error percepatan train, error posisi dan
  kecepatan validasi, perbandingan model nominal, serta keputusan lulus.
  Prediksi berjalan sendiri selama 2 detik per blok: tidak disetel ulang
  mengikuti pengukuran pada setiap sampel.
- Folder `<nama_model>_comparison`: CSV per sumbu dan
  `validation_comparison.png` memperlihatkan perintah, respons Gazebo, serta
  prediksi model. Prediksi diinisialisasi ulang hanya pada batas blok 2 detik
  atau ketika terdapat gap data, bukan pada setiap sampel.
- Lulus membutuhkan data lengkap, satu publisher kontrol, input informatif,
  parameter tidak menempel batas, maksimum error posisi <5 cm, RMS error
  kecepatan <5 cm/s, dan tidak lebih buruk dari model nominal.

Jika fit ditolak, model **tidak diaktifkan**. Least-squares dengan turunan
kecepatan dapat bias akibat noise; hasil ini bukan sertifikasi model.
Ground-truth flight tidak memvalidasi lokalisasi LiDAR. Pengujian perubahan
koefisien sintetis bukan pengujian payload cairan pada quadcopter nyata.

Kode utama: `identification.py`, `identification_recorder.py`,
`adaptive_model.py`, `model_fitting.py`. Arena khusus tetap memeriksa
lokalisasi dan pose FCU segar, kesesuaian frame, keterlambatan kontrol,
tracking, kecepatan, braking terhadap lantai/batas arena, serta geofence
0,65 m dari titik hover. Pembebasan syarat LiDAR hanya untuk preset
identifikasi ground-truth ini; misi survei/inspeksi tetap memerlukan LiDAR.
