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
Mekanisme umur peta adaptif rotating tidak disalin. Guidance A* riwayat
diadaptasi selektif dan memulihkan prior FOV fixed secara terpisah. Riwayat voxel terukur tetap ada;
TTL free warisan 21.600 s bukan bukti kestabilan lingkungan dinamis.

## Lintasan berkelanjutan dan kecepatan per fase

Konfigurasi inspection hasil `python3 tools/generate_fixed_lidar.py` kini memakai
guidance A* dari memori peta menuju tujuan approach/antarpohon/home. Worker
terpisah memperbarui guidance tiap 2 s simulasi, dengan maksimum 800.000 voxel
dan 30.000 ekspansi. Guidance berumur maksimum 8 s bukan izin menjalankan
lintasan pada peta basi: gate umur sensor/pose/peta lokal tetap berlaku.
Prefix hingga 4 m dipilih dari rute tersebut lalu direncanakan dan diperiksa
pada window lokal terbaru. Unknown tetap memerlukan observasi atau prior FOV
eksplisit yang telah diotorisasi. Batas history frontier memakai planner
exploration lama yang tetap memerlukan scan baru; bukan jalan pintas free.

Pergantian saat bergerak berlaku pada approach, transit antarpohon, return,
survey waypoint, dan kelanjutan scan parsial. Worker memakai batas posisi,
kecepatan, dan percepatan reference masa depan (lead .75 s). Kurva baru memiliki
transisi percepatan dengan jerk terbatas lalu quintic tersambung C2. Interval
kontrol yang melintasi batas pergantian dibagi secara matematis pada reference,
sehingga jitter dt tidak perlu menyebabkan lompatan. Hasil >1 s, terlambat,
fase/rate berubah, progress anchor berbeda, perubahan obstacle/unknown pada
kurva, atau bukti kamera tidak memenuhi syarat membuat kandidat ditolak.
Lintasan lama tetap berakhir dengan kecepatan nol. Kandidat diperiksa braking
pada snapshot worker, geometri pada peta live saat adopsi, serta command akhir
dan stopping corridor pada setiap tick melalui kontrol bersama/PID yang ada.
Ini bukan bukti seluruh stopping corridor masa depan tersertifikasi ulang
seketika pada setiap versi peta. Tidak ada publisher setpoint tambahan.

Scan mencoba seluruh 24 anchor jika koridor lokal sudah aman, tetap mengikuti
profil bawah daun dan validasi enam sektor kamera depan. Anchor tidak boleh
dilewati hanya agar penerbangan cepat. Ascent sudah memakai reference kontinu
reaktif dan tetap .12 m/s; takeoff, observasi kamera, serta landing/disarm
mempertahankan titik berhenti yang diperlukan. Takeoff/default .30 m/s,
transit hingga .40 m/s, scan hingga .30 m/s. Batas umum speed tetap .40 m/s,
acceleration .50 m/s², jerk 1.0 m/s³, tracking limit/margin .20 m. Ambang
perlambatan tracking, timeout dan clearance tidak dilonggarkan.

Log `history_route` mencatat alasan pemilihan/penolakan prefix; `moving_handoff`
mencatat jumlah kandidat/adopsi/penolakan, alasan, waktu, kecepatan dan selisih
PVA di sambungan. Replay matematis controller membuktikan adopsi bergerak dan
mencapai tujuan 6 m lebih cepat daripada tiga leg stop-to-stop 2 m pada fixture
yang sama. Replay ini tidak memodelkan dinamika Iris, latensi kamera/estimator,
MAVROS atau FCU. Belum ada flight untuk perubahan ini; bukan implementasi
optimasi waktu minimum global dan belum ada bukti pengurangan durasi misi nyata.

## Adaptasi lanjutan dari paper SUPER

Preset inspection sekarang menambahkan feedback speed envelope (reference cap
sekitar .34897 m/s dengan batas fisik tetap .4), profil braking per ruas,
densifikasi knot cruise, pencarian durasi, dan backup eksplisit dari output
PID. Angka transit .4 pada bagian sebelumnya adalah ceiling konfigurasi,
bukan target yang selalu dikirim setelah envelope. Kebijakan FOV tetap.
Lihat [implementasi dan evaluasi SUPER](SUPER_ADAPTATION.md) untuk batas,
benchmark, dan alasan belum mengganti voxel backend dengan CIRI atau PID
dengan MPC. Hasil flight ditulis di jurnal, terpisah dari replay/unit.

## Perintah worktree sesi ini

Dependency ROS 2 Humble/Gazebo Harmonic sudah terpasang. Bila belum tersedia,
perintah installer proyek adalah `bash tools/install_ros_system.sh` dan build
MAVROS `bash tools/build_mavros.sh`; installer tidak dijalankan ulang sesi ini.
Dependency terkompilasi yang sudah ada dipakai melalui symlink baca; overlay
build/install/run/log fixed-lidar dibuat terpisah.

```bash
cd /home/abin/polinasi_lidar_fixed
bash tools/build_ros.sh
# Jalankan HANYA setelah instruksi pengguna untuk mulai simulasi.
# Instruksi terbaru: seluruh run headless, berhenti bila terjadi fault.
POLINASI_RVIZ_AFTER_APPROACH=0 bash tools/run_fixed_lidar_validation.sh inspection
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


## Map palm_farm_flowers — 20 Oktober 2026

Launcher fixed menerima argumen kedua untuk memilih world. Misi tetap sensor
ICP–IMU, dua kandidat batang dari persepsi, kamera depan, tanpa mock bunga.
Geometri bunga pada world tetap obstacle seperti daun/batang/tanah.

```bash
cd /home/abin/polinasi_lidar_fixed
# Pertama kali: menghasilkan mesh deterministik yang belum tersedia.
/usr/bin/python3 tools/ensure_palm_flowers.py
bash tools/build_ros.sh
POLINASI_RVIZ_AFTER_APPROACH=0 bash tools/run_fixed_lidar_validation.sh inspection palm_farm_flowers
# Map lama tetap dapat dipilih:
POLINASI_RVIZ_AFTER_APPROACH=0 bash tools/run_fixed_lidar_validation.sh inspection palm_farm
```

Generator mesh berasal dari aset lokal workspace pengguna: generate_palm_flowers,
realistic_flower_farm dan fungsi geometri generate_palm_realistic. Hanya map
flowers dijalankan. Script protected_hashes dapat bekerja ketika map realistic
tidak ada pada branch fixed; tidak mengubah geometri pohon. Seed36035,
29 pohon, posisi jitter .45–1.6 m, minimum antarbatang6.5 m. Geometri rinci
ini berbeda dari palm_farm lama; keberhasilan pada map lama bukan bukti pada
map baru. Tidak ada koordinat bunga/manifest yang dimasukkan ke estimator
atau dijadikan target misi. Coverage ROI historis preset masih berasal dari
konfigurasi farm asli; bukan deteksi pohon atau sertifikat coverage farm baru.

Mesh Collada hasil generasi (~691 MB) diabaikan Git dan dibuat ulang melalui
generator; world, manifest, model metadata, dan generator disimpan di Git.
Hash SHA256 world dan semua mesh yang dirujuk dicatat pada
reports/palm_flowers/asset_audit.json, lalu disalin ke registered run sebagai
world_assets.json. Salinan pertama sesi ini memakai aset pengguna yang ada,
tanpa menulis workspace rotating-lidar. Generator juga menyimpan hash aset
farm lain yang tersedia; daftar itu dapat berbeda antarbranch walau geometri
flowers sama. Hash resource aktual adalah pembanding byte geometry.

Log validasi dan stop memakai mekanisme yang sama seperti map lama: baca
reports/fixed_lidar_validation_<tanggal>/run.txt, lalu jalankan
tools/stop_simulation.py --run dengan direktori tepat dari file itu.
Map numerik/point cloud/voxel/HTML ada di reports/ros_sensor_palm_farm_flowers_*/map_3d/.
Runtime tetap domain96, partition polinasi_fixed_lidar, UDP9002/14650 dan
TCP5762; jangan menjalankan dua simulator yang berbagi resource tersebut.


Preset inspection memakai transit_observation_guard=.2 m pada approach dan
return: rute, kurva serta live suffix menjaga buffer atas tambahan untuk
pembaruan daun. Buffer tidak mengganti clearance/tracking margin atau gate
lokalisasi dan tidak mengesahkan ruang unknown di luar prior pengguna.
Jika pengereman diinvalidasi, route_failure_diagnostics.emergency_braking
membedakan actual_remaining_stop dan nominal_next_step; exact grid dan
polinomial tersedia pada map_3d/planner_failures/failure_critical.npz.


Auditor membedakan route_changed recovery dan preventive depth_recovering.
Recovery depth tetap maksimal2 s simulasi dari timestamp started controller,
dengan depth valid/fresh dan health lain sehat; stale sensor/emergency selalu
menghentikan run. Event sementara bukan COMPLETE atau bukti lokalisasi gagal.
Timestamp tepat pada threshold memakai toleransi roundoff1e-9 s yang sama
dengan fresh_depth, tanpa memperbesar timeout fisik.


Pada inspeksi, transit APPROACH/RETURN memakai reserve observasi ke atas
`transit_observation_guard=.2` m dan lateral
`transit_lateral_observation_guard=.2` m. History A*, prefix, smoothing dan
live suffix menggunakan view konservatif yang sama; mask badan aktual dan
data occupancy/umur tidak diubah. Ini margin rekayasa untuk perubahan bukti
di voxel tetangga, belum jaminan obstacle avoidance terhadap deteksi terlambat.
Run bunga09d471d masih gagal47.35 s; status validasi terbaru ada di jurnal.
