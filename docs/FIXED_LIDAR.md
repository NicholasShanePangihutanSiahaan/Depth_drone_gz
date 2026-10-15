# Fixed-lidar: adaptasi misi rotating-lidar

Worktree sesi: `/home/abin/polinasi_lidar_fixed`, branch `fixed-lidar`.
Branch rotating-lidar hanya dibaca pada revisi `6779baf`, tidak diubah/merge.
MID-360 fixed horizontal `[0,0,.28,0,0,0]`; kamera depan `[.20,0,.06]`,
yaw nol dan hanya joint pitch. Dinamika simulator masih Iris; bukan model
hardware Jetson/Pixhawk/MID-360/ZED teridentifikasi.

Misi: takeoff → approach kandidat batang terukur → ascent di bawah daun →
scan enam sektor → pohon berikutnya → return home → landing → disarm.
Profil tinggi scan dan perbaikan anchor sektor diadaptasi secara selektif dari
rotating-lidar. Tinggi boleh berubah mengikuti koridor bawah daun. Kamera
menghadap pohon dari depan; pengamatan depth jalur dilakukan saat hold.
Setiap kurva 3D dan suffix live tetap diperiksa collision/braking. Target
puncak batang merupakan perkiraan geometris, bukan bunga terdeteksi.

## Aturan FOV yang diminta pengguna pada 15 Oktober

Peta terukur tetap membedakan occupied/free/unknown. Lapisan FOV terpisah
membolehkan navigasi pada sebagian **unknown hasil asumsi**, bukan hasil ukur:

- Di bawah −7° pada frame LiDAR: unknown dianggap aman di seluruh batas peta
  navigasi, tanpa batas jarak sensor. Window lokal dibangun dari sejarah peta.
- Di atas +52°: per azimuth, objek berjarak r pada ray batas diproyeksikan
  menjadi batas atas virtual z=r×cos(38°) relatif sensor. Contoh r=4 m →
  z=3,152 m. Unknown yang menyentuh/di atas batas virtual tetap terhalang.
- Ray batas no-return yang sudah diverifikasi oleh adapter Gazebo mengizinkan
  asumsi atas sampai batas peta. NaN/invalid tidak membuat asumsi atas baru.
- Obstacle terukur (tanah, batang, daun) tidak dihapus oleh asumsi. Data sensor
  baru yang menunjukkan bahaya memperbarui/membatalkan asumsi free terkait.
- Sesuai keputusan pengguna, asumsi tersimpan saat dropout, tanpa menyegarkan
  timestamp pengamatan. Gate kesehatan pose/LiDAR/IMU/peta tetap terpisah.

Array `assumed_free` dan `assumed_occupied` pada snapshot/ekspor serta
`fixed_fov_assumptions` pada log membedakan sumber bukti. HTML dan RViz memakai
ungu untuk asumsi aman, oranye untuk batas virtual, merah untuk occupied
terukur. Tidak semua unknown aman: unknown di dalam FOV yang belum terukur
atau di belakang halangan tetap ditolak. Ini eksperimen dengan prior FOV,
bukan validasi keselamatan ruang yang tidak pernah diukur.

Lokalisasi memakai LiDAR+IMU badan, estimator ICP eksperimental; bukan
FAST-LIO. Ground truth hanya masuk auditor. Depth RGBD sintetis dipakai
pemetaan inspection; tidak meniru seluruh kesalahan stereo ZED asli.
Mekanisme umur peta adaptif, smooth approach/ascent, dan global-route baru
rotating tidak disalin secara menyeluruh. Riwayat voxel terukur tetap ada;
TTL free warisan 21.600 s bukan bukti kestabilan lingkungan dinamis.

## Perintah worktree sesi ini

Dependency ROS 2 Humble/Gazebo Harmonic sudah terpasang. Bila belum tersedia,
perintah installer proyek adalah `bash tools/install_ros_system.sh` dan build
MAVROS `bash tools/build_mavros.sh`; installer tidak dijalankan ulang sesi ini.
Dependency terkompilasi yang sudah ada dipakai melalui symlink baca; overlay
build/install/run/log fixed-lidar dibuat terpisah.

```bash
cd /home/abin/polinasi_lidar_fixed
bash tools/build_ros.sh
# Awal headless; RViz dibuka sekali SETELAH approach pertama selesai:
POLINASI_RVIZ_AFTER_APPROACH=1 bash tools/run_fixed_lidar_validation.sh inspection
# Alternatif reproduksi hover:
bash tools/run_fixed_lidar_validation.sh hover
```

Runner fixed bawaan: ROS domain 96, Gazebo partition polinasi_fixed_lidar,
MAVROS UDP 14650 (`POLINASI_MAVROS_UDP_PORT`), RTF .15. Pengguna dapat mengganti POLINASI_ROS_DOMAIN_ID/POLINASI_GZ_PARTITION
secara konsisten di semua terminal. Untuk probe manual:

```bash
export POLINASI_ROS_DOMAIN_ID=96 POLINASI_GZ_PARTITION=polinasi_fixed_lidar
source /home/abin/polinasi_lidar_fixed/tools/ros_environment.sh
```

Port fixed UDP 9002/14650 dan TCP 5762 masih bersama, sehingga **jangan menjalankan
simulator kedua**, meski domain/partition berbeda. Launcher menolak port sibuk.
Gazebo GUI dan window kamera tidak dibuka. RViz milik auditor ditutup pada
akhir run; tidak menutup viewer/simulator sesi lain.

Log: `reports/fixed_lidar_validation_<tanggal>/`; run.txt menunjukkan direktori
`.dependencies/ros_<run>/` dengan session.json, ros.log, gazebo.log, sitl.log,
ros_logs dan rviz_after_approach.log. Stop hanya run terdaftar tersebut:

```bash
python3 /home/abin/polinasi_lidar_fixed/tools/stop_simulation.py --run /home/abin/polinasi_lidar_fixed/.dependencies/ros_<run>
```

Map/cloud/voxel numerik dan tampilan offline berada pada
`reports/ros_<run>/map_3d/map.json` dan map.html. Akurasi, status misi,
reference/PID, alasan hold/abort, antrean IMU, waktu pipeline dan umur data
tersimpan pada accuracy*.jsonl, accuracy.summary.json dan navigation_debug.jsonl.
Lihat [jurnal bertanggal](JURNAL_FIXED_LIDAR_20261015.md) untuk hasil tes dan
flight; keberhasilan unit tidak berarti keberhasilan dua pohon hingga disarm.
