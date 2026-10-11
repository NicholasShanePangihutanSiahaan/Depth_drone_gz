# Eksperimen tilt di arena kosong

Tujuan: membandingkan self-return LiDAR dengan dan tanpa visual kendaraan.
Arena memakai `identification.sdf` (hanya lantai), tetapi **bukan** misi
identifikasi/autotune: LiDAR, IMU, kamera dan mapper tetap aktif. Drone tidak
di-arm; tester hanya menerbitkan target mount, bukan perintah terbang.

```bash
cd /home/abin/polinasi_lidar
source tools/ros_environment.sh
/usr/bin/python3 tools/generate_empty_mount_configs.py
bash tools/start_empty_mount_test.sh visible
```

Di terminal kedua:

```bash
cd /home/abin/polinasi_lidar
source tools/ros_environment.sh
/usr/bin/python3 tools/check_empty_mounts.py --config polinasi_nav/config/empty_mount_visible.json --output reports/empty_mount_visible_result.json
/usr/bin/python3 tools/stop_simulation.py --run RUN_VISIBLE
```

Ulangi dengan `hidden` dan `empty_mount_hidden_result.json`; berhentikan run
visible dahulu. RUN adalah direktori `.dependencies/ros_...` yang dicetak
launcher. Jangan menjalankan `start_mission.py` pada eksperimen ini.

RViz dan viewer kamera dibuka; GUI Gazebo tidak dibuka. Fisika/server Gazebo
tetap berjalan. Kedua viewer milik run ikut ditutup saat run dihentikan.

Pengukuran mencakup transisi dan minimal 2 detik simulasi setelah sudut tercapai.
LiDAR: 0; 0,25; 0,5; 0,7 rad; kemudian kamera yaw +1,2 / −1,2 dan pitch
0,65 / 0,3 rad, lalu semua mount kembali nol. Feedback aktual dan perubahan
pose badan dicatat. Lantai z=0 dikecualikan: titik z>0,1 m, voxel centre z>0,15 m,
dalam radius 0,9 m dari badan. Voxel sticky dapat bertahan dari langkah terdahulu;
jumlah voxel bukan jumlah titik per-scan. Ukuran voxel tetap 0,2 m.

`near_above_floor_points`: semua titik dekat badan non-lantai.
`outside_body_filter_points`: subset yang lolos kotak self-filter.
`map.near_occupied_voxels`: occupied dekat badan setelah filter dan integrasi.
Contoh koordinat dibatasi, bukan perekaman ROS bag.

Jika visible menghasilkan titik/voxel yang hilang pada hidden, itu mendukung
asal visual kendaraan atau artefak rendering terkait kendaraan. Tidak membuktikan
sensor hardware identik. Pengujian unarmed juga tidak mencakup baling-baling
berputar, sikap/gerak terbang atau kendali inspeksi airborne.

## Hasil 10 Oktober 2026

Bagian ini adalah baseline **sebelum patch rendering**. Penyebab kemudian
terisolasi ke sampler cubemap Gazebo dan replay sesudah patch menghasilkan nol
voxel palsu, dengan obstacle kubus tetap terdeteksi. Lihat hasil final dan cara
reproduksi di [LIDAR_RENDERING_FIX.md](LIDAR_RENDERING_FIX.md).

Kedua eksperimen menyelesaikan tujuh langkah, badan tetap tidak bergeser.
Visible: 168 scan, 3130 titik dekat badan non-lantai, 274 lolos self-filter.
Hidden: 145 scan, 205 titik dekat badan non-lantai, semuanya lolos self-filter.
Durasi/jumlah scan tidak sama; ini jumlah pengamatan berulang, bukan objek unik.
Pada pitch 0,7 rad **keduanya** mempunyai peak 3 titik lolos/scan dan 1 occupied
voxel dalam radius 0,9 m. Menghilangkan visual badan mengurangi self-return,
tetapi **tidak menghilangkan voxel palsu**. Dugaan self-return sebagai satu-satunya
penyebab tidak didukung. Kamera kiri/kanan tidak menghilangkan pola tersebut.

Map hidden masih memiliki 47 voxel non-lantai secara keseluruhan pada snapshot
50 s, z centre 0,3–2,3 m pada arah terkonsentrasi. Satu contoh arah sinar mentah
pada audit sebelumnya mempunyai x≈y (azimut sensor sekitar 45°). Penyebab
rendering versus transform/timestamp belum terisolasi; jangan menghapus seluruh
sektor sinar atau voxel tersebut untuk mengizinkan penerbangan. Occupancy sticky
menyimpan hit sebelumnya, sehingga artefak tetap terlihat setelah tilt kembali nol.

Error akhir mount maksimum: LiDAR visible 0,0054 rad / hidden 0,0016 rad;
yaw kamera sekitar 0,0027 rad. Pitch kamera hingga 0,127 rad (~7,3°), jadi
uji pergerakan ini bukan bukti tracking pitch kamera stabil. Target dapat dilalui
sesaat sebelum osilasi mereda; hasil akhir dicatat terpisah.

Laporan: `reports/empty_mount_comparison.json` dan dua `empty_mount_*_result.json`.
Seluruh tes paket sebelumnya lulus 219 tes; regresi mount saat eksperimen lulus
7 tes. Kedua run beserta viewer dihentikan setelah eksperimen. Ini bukan uji flight.
