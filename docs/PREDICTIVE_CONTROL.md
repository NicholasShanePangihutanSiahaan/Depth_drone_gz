# Pengendali prediktif aktif — simulasi saja

Model kini dapat menentukan koreksi percepatan yang benar-benar diterbitkan
melalui publisher MAVROS akhir yang sama. Ini **MPC model respons PVA linear**
dengan kendala nonlinear norm kecepatan/percepatan/jerk dan tube tracking;
**bukan NMPC model sikap/rotor quadcopter lengkap**, bukan L1-NMPC dan bukan MPCC.
Tidak mengubah PID ArduPilot dan tidak otomatis menaikkan speed.

Dasar pertimbangan: [Hanover dkk., adaptive NMPC](https://arxiv.org/abs/2109.04210)
membahas kompensasi ketidakpastian/payload dengan model dan adaptor yang lebih
lengkap. [Romero dkk., MPCC](https://arxiv.org/abs/2108.13205) mengoptimalkan
kemajuan sepanjang jalur sekaligus kontrol. Implementasi awal ini memakai
model lokal hasil log, bukan mengklaim telah mereproduksi kedua paper tersebut.

## Yang berubah

- Planner/FSM masih membuat jalur serta referensi P/V/A. MPC memakai posisi
  dan kecepatan terukur untuk memprediksi respons 1 detik ke depan, 10 kontrol
  berjarak 100 ms; integrasi model 50 ms. Delay per sumbu memakai riwayat
  perintah yang benar-benar diterbitkan dan timestamp aslinya.
- Optimisasi SLSQP meminimalkan error posisi/kecepatan dan besar/perubahan
  koreksi percepatan. Model affine dikondensasikan menjadi matriks kecil agar
  iterasi murah. Posisi/kecepatan referensi tetap; hanya acceleration
  feedforward dioptimalkan. Ini bukan pembagian waktu jalur optimal.
- Satu proses worker, paling banyak satu job pending. Callback kontrol tidak
  menunggu solver. Worker dimulai ketika node dibuat dan harus siap sebelum
  preflight alignment. Solusi >250 ms waktu simulasi tidak pernah diterapkan.
  Pada SURVEY/RETURN, hasil basi memicu brake/hold dan perencanaan ulang terbatas
  (maksimal tiga pemulihan per run). Pengereman harus selesai dan kecepatan
  terukur <0,08 m/s sebelum mencoba lagi; batas waktu pengereman/pemulihan
  4 detik simulasi. Nominal progress ditahan sampai solusi baru lolos safety.
  Jika tetap basi, batas percobaan habis, solver gagal/infeasible, sensor/map
  tidak sehat, atau frame meleset >1 cm, abort tetap berlaku. IDENTIFY tidak
  diulang otomatis karena protokol identifikasi memiliki waktu absolut.
  Pergantian state membuang rencana lama dengan generation token; job yang
  masih berjalan tidak ditumpuk dan hasilnya dibuang setelah selesai.
- Warmup awal maksimal 2 detik memakai referensi nominal yang sudah diperiksa,
  lalu beralih ke MPC setelah solusi tersedia. `model_controls_flight` dan
  `predictive_applied` membedakan observer dari koreksi yang benar-benar dipakai.
- Koreksi akhir dibatasi jerk per callback. Respons model dihitung ulang
  setelah pembatasan itu, memakai keadaan/map terbaru. Batas speed/acceleration,
  tube 6 cm, segment map, dan braking diperiksa lagi. Unknown tetap tidak aman.
  Safety filter bukan bukti robust collision avoidance pada hardware.
- Sebelum mengunci transform FCU, mode ini membutuhkan sampel pose stationary
  dengan stamp selisih <25 ms, rentang offset <3 mm selama sedikitnya 3 detik.
  Tidak mengganti frame label atau mengubah offset ketika drone bergerak.
- Pilihan `predictive_adaptation=true` mempromosikan RLS yang lolos gate ke
  model optimisasi, dibatasi sekitar ±20% koefisien awal dan ±0,01 m/s² bias.
  Preset awal **false**, untuk mengisolasi pengujian pengendali fixed-model.
  Pilihan adaptasi aktif belum sama dengan L1 dan tidak punya bukti kestabilan
  closed-loop. Bukan estimator massa cairan.

Misi lama tidak otomatis berubah. Semua preset ini ground-truth simulasi;
sensor-based/hardware, NMPC penuh, dan kontrol thrust/body-rate tidak diaktifkan.

## Jalankan

Terminal 1:

```bash
cd /home/abin/polinasi_lidar
bash tools/build_ros.sh
bash tools/start_predictive.sh arena
```

`arena` adalah gerakan validasi singkat di arena kosong tanpa LiDAR. Preset
eksperimental `smoke` menggunakan misi mapping kecil dengan LiDAR dan safety
map; bukan full farm-tour. Jangan menyatakan smoke sudah teruji hanya karena
arena berhasil. Simulasi default faktor 0,2; tidak mengubah langkah fisika.

Untuk zigzag survei kebun, ganti perintah launcher di terminal 1 dengan:

```bash
bash tools/start_predictive.sh tour
```

Preset `predictive_tour.json` mempertahankan seluruh 326 target dan batas
keselamatan `mapping_tour.json`, tetapi mengaktifkan MPC dengan model hasil
identifikasi. LiDAR, peta voxel, A*, eksplorasi, dan FSM mapping tetap aktif.
Hanya launcher ini yang mengaktifkan MPC; `start_mapping.sh tour ground_truth`
tetap memakai kontrol lama. Lokalisasi masih ground-truth, bukan validasi
lokalisasi LiDAR. Adaptasi model online tetap nonaktif. Kecepatan perintah
tetap maksimal 0,3 m/s; pemasangan MPC tidak otomatis mempercepat survei.
Konfigurasi dan regresi diuji, tetapi penerbangan penuh zigzag dengan MPC
belum divalidasi. Jangan jalankan kedua launcher bersamaan.

Integrasi preset `tour`: build tiga paket ROS berhasil dan 149 tes lulus
(termasuk 11 tes MPC). Tes baru memastikan rute/batas keselamatan tidak
berubah, model identifikasi diterima, serta launcher memilih mapping kebun
ground-truth dengan konfigurasi MPC. Tes launcher memakai pengganti lokal,
bukan menjalankan simulator. Ini bukan hasil penerbangan penuh.

Terminal 2:

```bash
cd /home/abin/polinasi_lidar
source tools/ros_environment.sh
.venv/bin/python tools/set_origin.py --timeout 120
python3 tools/check_ros_simulation.py --output reports/predictive_setup.json
```

Jika `passed=true`:

```bash
python3 tools/start_mission.py
ros2 topic echo /navigation/status
```

Status: `predictive_controller`, `predictive_active`, `predictive_applied`,
`predictive_reason`, `predictive_solves`, `predictive_worker_wall_max_ms`,
`predictive_model_updates`, dan parameter model. Log arena disimpan pada
`reports/<nama_run>/identification.json`; log terminal di `.dependencies/`.
Log dengan MPC aktif tidak diterima sebagai data identifikasi baseline oleh
fitter. Kode: `predictive.py`; preset `predictive_arena.json`,
`predictive_smoke.json`, dan `predictive_tour.json`. Pada misi mapping,
peta tersimpan di `reports/<nama_run>/map_3d/map.html` seperti survei biasa.

Diagnostik tambahan: `predictive_worker_ready`, `predictive_recoveries`,
`predictive_queue_wall_max_ms` (antre/pengiriman ke worker),
`predictive_worker_wall_max_ms` (perhitungan),
`predictive_receive_wall_max_ms` (selesai hingga diterima callback),
`predictive_solution_age_seconds` (umur solusi terakhir yang diperiksa,
waktu simulasi), serta `predictive_measurement_age_seconds` (umur pose saat
job dibuat). Nilai maksimum tiap tahap bukan otomatis dari job yang sama.
Status ikut disimpan dalam `map_3d/mission_status.json` dan snapshot map.
Perubahan ini diuji dengan regresi, belum diuji pada penerbangan penuh kebun.
Build tiga paket berhasil; 155 tes regresi lulus. Setelah penanganan exception
job dari state lama diperketat, 17 tes MPC terarah kembali lulus. Cakupan:
prewarm worker, pembuangan hasil/exception obsolete, pemulihan terbatas,
pengereman dengan batas acceleration/jerk, resume hanya setelah solusi fresh,
timeout, sensor hilang, dan unknown yang tidak boleh dianggap aman.

## Batas penggunaan model

Identifikasi hanya mencakup hover dan gerakan kecil. Data lama menunjukkan
selisih Z sekitar 3,8 cm; gate frame lebih ketat tidak membuktikan seluruh
penyebab telah hilang. Model tidak otomatis divalidasi untuk jarak/payload/
speed berbeda. Simulator tidak mengurangi massa cairan. Jangan memindahkan
parameter ke pesawat nyata atau mengklaim stabilitas adaptive flight control.

## Hasil uji yang benar-benar dijalankan

Run arena `ros_ground_truth_identification_20261010_055733_29097_identification`
selesai sampai LAND/disarm, tanpa failure/timing-gap; satu publisher final.
MPC dipakai pada 858 dari 862 perintah (awal warmup nominal). Tracking maksimum
7,338 mm dan RMS 2,687 mm, berdasarkan command terakhir pada stamp respons.
Maksimum command: speed 0,1053 m/s, acceleration 0,1311 m/s², jerk 0,6613 m/s³;
semuanya di bawah batas konfigurasi. Solver maksimum 163,4 ms **wall time**,
callback kontrol maksimum 51,4 ms wall time. Run memakai faktor waktu 0,2;
ini bukan bukti mampu realtime faktor 1 atau Jetson.

Laporan: `reports/predictive_arena_20261010.json`; dataset lengkap berada
di `reports/<nama_run>/identification.json`. Simulator sudah dihentikan.
Saat Ctrl+C, worker idle mencetak KeyboardInterrupt; node induk keluar normal
setelah log tersimpan. Worker sekarang menyerahkan SIGINT ke induk agar pool
ditutup bersih; perubahan ini belum diuji pada shutdown simulasi penuh lagi.
145 regresi lulus sebelum uji; delapan tes MPC terarah lulus setelah tambahan
tes promosi koefisien online dalam trust region. Adaptasi aktif hanya diuji
sintetis, bukan terbang. Smoke mapping dan full farm-tour dengan MPC belum diuji.
Tidak ada A/B berpasangan: perubahan preflight alignment dan pacing membuat
perbedaan tracking dengan run sebelumnya tidak boleh diklaim sebagai manfaat MPC.

```bash
python3 tools/check_predictive_log.py \
  reports/ros_ground_truth_identification_20261010_055733_29097_identification/identification.json \
  --output reports/predictive_arena_check_baru.json
```
