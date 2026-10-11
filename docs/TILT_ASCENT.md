# Inspeksi dengan approach rendah dan LiDAR tilt

Konfigurasi opt-in `tilt_inspection.json`, ground-truth simulation saja.
Depth kamera depan-bawah kini melengkapi occupancy saat approach;
lihat [integrasi depth simulasi](DEPTH_CAMERA_MAPPING.md).
Misi tanpa bunga/buzzer memakai kandidat batang terukur, bukan koordinat SDF.

Alur: takeoff → approach ketinggian 2 m pada radius kandidat 2 m dari pusat batang
→ hover/orient ke pohon → observasi pitch LiDAR 1,186823891 rad (68°)
tanpa sudut antara 14,3° → naik kontinu sambil membaca LiDAR
→ rem/hover pada batas koridor teramati aman → normalisasi LiDAR
→ scan → pohon berikutnya → return/land.

`tree_continuous_ascent=true` menggantikan langkah stop-to-stop 0,4 m.
Kecepatan maksimum naik 0,12 m/s (target cruise 90% batas), dengan batas
percepatan/jerk yang sama. Setiap tick menguji swept segment dan lintasan
pengereman halus pada voxel terinflasi; unknown tetap bukan free.
Cadangan waktu reaksi 0,7 s, maksimum umur map 0,35 s. Jika koridor berikutnya
tidak aman atau map terlalu tua, jalankan lintasan rem yang diperiksa kembali.
Jika hanya map terlambat, hover dengan LiDAR tetap tilt, tunggu pengukuran
baru yang terintegrasi dan segar, lalu uji ulang koridor sebelum naik lagi.
Penantian dibatasi `tree_mount_timeout` (default 20 detik simulasi); timeout
gagal dengan `ascent_mapping_recovery_timeout`, bukan masuk scanning.
Koridor/guard terhalang atau tinggi tujuan tercapai tetap mengakhiri ascent
dengan hover sebelum normalisasi LiDAR. Jika cadangan rem ikut terhalang,
laporkan `ascent_backup_invalidated`; pemeriksaan kesehatan umum tetap aktif.
Ini bukan jaminan keselamatan hardware atau bukti tinggi maksimum sebenarnya.
Mode langkah lama tersedia dengan `tree_continuous_ascent=false`.

Syarat observasi: feedback mount segar, sudut tercapai, drone diam, dwell 0,8 s,
dan scan yang terintegrasi setelah mount stabil. Selama mengubah tilt, referensi
posisi tetap; pemeriksaan kesehatan, tracking, clearance aktual dan pengereman
dari controller bersama tetap berjalan. Saat naik, sudut LiDAR ditahan tetap;
perubahan sudut baru hanya diizinkan kembali saat hover.

Kenaikan membutuhkan seluruh koridor gerak/pengereman terinflasi teramati bebas.
Unknown, daun, dan tanah tidak dihapus untuk meloloskan gerakan. Jika langkah
berikutnya tidak tersertifikasi bebas, drone mengerem dan scan
pada ketinggian yang sudah dicapai. `ascent_stop_reason` membedakan kondisi ini
dari tercapainya ketinggian tujuan. Ini **bukan bukti ketinggian aman maksimum
sebenarnya** ataupun berhasil menemukan pucuk/flower.

LiDAR kembali normal dan menerima scan baru sebelum scanning/perpindahan biasa.
Algoritma eksplorasi dengan model FOV tetap tidak dijalankan selama ASCEND.
Scanning memakai **6 viewpoint** dengan selisih sudut nominal 60°.
Saat berpindah, badan mengikuti arah lintasan dan kamera depan pitch ke bawah
25° menuju jalur (termasuk detour/eksplorasi). Saat tiba di viewpoint, drone
hover, berputar menghadap pohon, lalu kamera pitch ke atas menuju perkiraan
area atas batang. Kamera hanya mempunyai satu axis pitch; joint yaw dibuat
fixed dan tidak memiliki publisher perintah atau kewajiban feedback yaw.
Selama ORIENT/observasi LiDAR ke atas, badan juga menghadap pohon.

View dihitung setelah yaw badan sesuai, pitch kamera positif dan sesuai target,
feedback segar, dwell 0,8 s serta scan baru setelah stabil. Timeout 20 s memberi
failure `inspection_view_orientation_timeout`, bukan menunggu tanpa batas.
Log status memuat `inspection_camera_policy`, `body_heading_policy`,
`inspection_view_count`, `camera_yaw_enabled`, dan `viewpoint_settled_since`.
Enam viewpoint bukan jaminan semuanya dapat dicapai: obstacle/unknown tetap
ditolak. Sebelum merencanakan approach/perpindahan scanning, hover dan arahkan
badan ke jalur, tunggu kamera menunduk/stabil 0,4 s serta depth yang benar-benar
terintegrasi setelah stabil. Log `navigation_camera_gate` menjelaskan setiap
syarat. Baru setelah itu planner memeriksa ulang jalur.
Eksplorasi approach/scanning kini hanya boleh pada tinggi yang sama
(`tree_exploration_same_altitude=true`); tidak ada fallback turun diam-diam.
Jika tidak ada rute/view yang aman, hold dan timeout tetap berlaku.
Log `continuous_ascent` mencatat sertifikasi rem, tinggi tujuan, umur/version
map, dan alasan berhenti; `ascent_records` menyimpan akhir kenaikan serta
`map_pause`/`map_resume`. `ascent_map_pauses` dan `ascent_map_resumes`
mencatat jumlah jeda/pemulihan. Batas umur map dan margin tidak dilonggarkan.

Radius 2 m berlaku untuk approach dan kandidat scanning, bukan jaminan ruang
aman atau pasti berada di bawah daun. Target dibulatkan ke pusat voxel 0,2 m,
sehingga radius aktual dapat sedikit berbeda. Tidak ada fallback radius lebih
jauh pada konfigurasi ini; unknown tetap memerlukan pengamatan dan obstacle
tetap ditolak. Hasil percobaan nyata pengguna menjadi alasan pemilihan radius,
bukan sertifikasi keselamatan untuk model kebun ini.

FOV bawaan tetap −7° hingga +52°. Tengah penampang depan adalah +22,5°;
tilt 68° mengarahkannya ke 90,5° relatif badan mendatar. Pada penampang depan
rentangnya 61°–120°, tetapi seluruh FOV 360° ikut berotasi: sinar samping dan
belakang tidak semuanya mengarah ke atas. Observasi 14,3° kini dihapus sesuai
permintaan; cakupan depan-atasnya tidak lagi tersedia pada fase tilt.
Limit joint dan adapter simulasi kini +70°,
bukan klaim batas mount fisik atau kalibrasi hardware.

Dasar FOV: [Livox Mid-360 Quick Start Guide](https://dl.djicdn.com/downloads/Livox/Mid-360/QSG/Livox_Mid-360_Quick_Start_Guide_multi.pdf).

## Jalankan (RViz dan kamera saja)

```bash
cd /home/abin/polinasi_lidar
source tools/ros_environment.sh
/usr/bin/python3 tools/generate_sensor_gimbal_config.py
colcon --log-base log_ros build --symlink-install --build-base build_ros --install-base install_ros --base-paths polinasi_nav beehive_drone uav_interfaces --packages-select polinasi_nav
bash tools/start_tilt_inspection.sh
```

Gazebo server tetap menjalankan fisika/sensor, tetapi GUI Gazebo tidak dibuka.
Viewer kamera memakai `rqt_image_view` pada `/zed/image`.

Terminal kedua:

```bash
cd /home/abin/polinasi_lidar
source tools/ros_environment.sh
.venv/bin/python tools/set_origin.py --timeout 120
/usr/bin/python3 tools/check_ros_simulation.py --output reports/tilt_inspection_setup.json
/usr/bin/python3 tools/start_mission.py
```

Mulai hanya jika setup lulus. Untuk log terminal dan penghentian otomatis pada
kegagalan, gunakan terminal ketiga; ganti RUN dengan direktori run yang dicetak:

```bash
source tools/ros_environment.sh
/usr/bin/python3 tools/watch_tree_inspection.py --run RUN --output reports/tilt_watchdog.jsonl --seconds 7200
```

Penghentian manual sekaligus menutup viewer milik run:

```bash
/usr/bin/python3 tools/stop_simulation.py --run RUN
```

Log peta HTML dan `navigation_debug.jsonl` berada di `reports/<nama-run>/map_3d/`.

Kenaikan kontinu juga mensyaratkan buffer vertikal teramati 0,5 m
(`tree_ascent_observation_guard`) di atas jalur berhentinya. Ini menambah
ruang cadangan saat LiDAR kembali horizontal dan daun lain baru terlihat;
bukan mengurangi inflasi obstacle atau menganggap unknown kosong.
Log mencatat `observation_guard_safe` dan pemeriksaan buffer tersebut.

Titik bantu approach/scan yang masuk inflasi obstacle tidak lagi menjadi
endpoint A* yang mustahil. Pemilih subgoal mencoba titik pendek (maksimum 2 m)
yang lebih dekat ke target akhir dan seluruh segmen menuju titik itu sudah
teramati bebas. Pada APPROACH boleh turun menuju altitude approach; SCAN
tetap horizontal. Target viewpoint akhir tidak diganti dan tidak dihitung
selesai hanya karena mencapai subgoal. Unknown tetap memerlukan observasi.
Log `inspection_subgoal_diagnostics` menyimpan titik awal, titik alternatif,
target akhir dan voxel penyebabnya. Jika tidak ada alternatif, tetap hold.

Replay read-only dari peta yang tersimpan:

```bash
python3 tools/replay_inspection_subgoal.py reports/<nama-run>/map_3d/map.json
```
Status memuat `ascent_records`, pitch target/aktual, fase, izin gerakan mount,
penghalang occupied/unknown, data segar/basi, serta diagnostik pemrosesan/kontrol.
Kegagalan pengereman kini mencatat setiap percobaan kecepatan dan sel penyebab
setelah inflasi badan/margin; lihat [audit pengereman](BRAKING_DIAGNOSTICS.md).

## Batas validasi

Estimator LiDAR–IMU bergerak belum tersedia; mode sensor tetap ditolak.
Kamera kini RGB-D ideal Gazebo, bukan emulasi stereo/depth SDK ZED. Pohon dideteksi geometris;
tinggi kolom teramati bukan pucuk batang terkonfirmasi. Demo dibatasi dua pohon,
bukan bukti seluruh kebun selesai. Batas kecepatan 0,3 m/s tidak dinaikkan.
Kualitas visual bunga, gerakan daun/angin, kompensasi scan mount per titik pada
sensor nyata, dan servo fisik belum tervalidasi. Tes unit FSM bukan validasi
penerbangan lengkap; hasil run aktual harus diperiksa terpisah.

## Self-return Gazebo

Flag diagnostik opsional `simulation_lidar_hide_self=true` memisahkan visual kendaraan (bit 2) dari
mask LiDAR (bit 1) pada salinan model privat setiap run. Mesh pohon/tanah,
collision, massa, joint, dan fisika kendaraan tidak diubah; kamera tetap melihat
visual kendaraan. Filter badan dengan TF terukur tetap aktif.

Ini penyederhanaan simulasi: pantulan **dan occlusion oleh badan sendiri** tidak
disimulasikan oleh GPU LiDAR. Karena itu hasilnya tidak memvalidasi self-filter
hardware. LiDAR nyata memerlukan kalibrasi bentuk badan/mount serta masking
sinar yang terhalang badan; sinar tersebut tidak boleh dijadikan bukti ruang
bebas di belakang badan. Jangan memakai flag ini untuk menghapus objek kebun.

Audit sebelum perubahan: run `20261010_215755_58625` berhenti saat ASCEND pada
57,75 s; run `20261010_220712_60388` berhenti saat UPWARD_OBSERVE pada 50,15 s.
Pada run kedua, tiga titik near-body lolos kotak filter dengan z badan
0,307–0,338 m saat pitch bertambah. Clearance turun mendadak menjadi negatif.
`reports/tilt_run2_near_cloud.jsonl` menyimpan titik mentah dan hasil TF untuk
membedakan bukti pengukuran dari dugaan sumber artefak. Perubahan mask harus
dikonfirmasi lewat run pembanding, bukan dianggap sukses dari tes unit.

Setelah replay, akar masalah voxel palsu ditemukan pada sampler cubemap Ogre2;
lihat [perbaikan renderer LiDAR](LIDAR_RENDERING_FIX.md). Default tilt sekarang
tidak menyembunyikan visual badan. Patch renderer privat memperbaiki pembacaan
sinar, sedangkan pantulan badan tetap ditangani self-filter terukur.
