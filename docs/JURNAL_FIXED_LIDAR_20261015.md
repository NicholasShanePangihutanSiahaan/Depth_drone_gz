# Jurnal fixed-lidar — 15 Oktober 2026

## Audit awal dan reproduksi

Basis lokal dan remote `fixed-lidar`: `818326ca991d77153e1c44260580e50629946698`.
Worktree sesi: `/home/abin/polinasi_lidar_fixed`. Worktree awal tetap
`/home/abin/polinasi_lidar`, branch rotating-lidar; perubahan lokalnya tidak
disalin atau dimasukkan commit. Tidak ditemukan AGENTS.md yang berlaku pada
root/ancestor atau tree fixed-lidar. Repository `/home/abin/p0l1n4s1` tidak diubah.
Pemeriksaan proses dan socket host sebelum flight tidak menemukan simulator.
SITL masih menggunakan UDP 9002/14550 dan TCP 5762; domain ROS sesi ini 96.
Karena port SITL belum diparameterisasi, simulator kedua tetap dilarang.

Dependency terpasang ArduPilot, plugin Gazebo, MAVROS dan venv ditautkan ke
worktree ini untuk pemakaian baca. Build/install/log/run baru ada di worktree
fixed sendiri. Tidak ada instalasi atau perubahan source dependency bersama.

### Discovery dan alur runtime

- `tools/audit_repository.py`: entry point dan executable launch valid.
  Paket aktif: uav_interfaces, beehive_drone, polinasi_nav; tiga paket lama
  mempunyai COLCON_IGNORE. `tools/build_ros.sh`: tiga paket build berhasil;
  **318 tes lulus, 0 skipped**, sebelum flight, domain tes 87.
- Entry: run_fixed_lidar_validation.sh → start_fixed_lidar.sh →
  generate_fixed_lidar.py → start_simulation.sh sensor palm_farm mapping →
  sitl_navigation.launch.py. Baseline hover dan inspection terpisah.
- Sensor gate: simulation/lidar → livox/lidar. Lokalisasi eksperimental
  scan-to-map ICP memakai cloud dan simulation/body_imu. Mode sensor tidak
  subscribe ground truth. Auditor saja memasangkan truth pada timestamp.
- Lokalisasi menerbitkan odom/base_link; map/odom identitas. Transform mount
  fixed dan kamera diurus node mount; transform FCU dihitung numerik,
  bukan mengganti label frame. ExternalNav meneruskan odometri ke MAVROS.
- Mapping IO proses terpisah: LiDAR + depth sintetis (inspection), riwayat
  sparse, ROI/corridor, navigation snapshot depth satu. Planner proses spawn;
  kontrol memeriksa lintasan serta pengereman pada peta terbaru. PID memakai
  pemeriksaan akhir. Mapping/perception tidak mempunyai publisher kontrol.
- Probe setup baseline: tepat satu publisher final /mavros/setpoint_raw/local,
  satu ExternalNav dan satu producer pada interface kontrol yang diperiksa;
  interface setpoint alternatif tidak aktif. Setup PASS.
- GPS1_TYPE/GPS2_TYPE=0, SIM_GPS_DISABLE/SIM_GPS2_DISABLE=1;
  EKF3 memakai ExternalNav untuk posisi, kecepatan, yaw. Origin/home tetap
  hanyalah metadata referensi SITL, bukan pengukuran GPS.

### Mounting dan perilaku misi

MID-360 fixed horizontal `[0,0,0.28,0,0,0]`, kamera depan `[.20,0,.06]`,
yaw nol, pitch aktif, yaw joint fixed. Kode generator SDF dan tes geometri
sesuai dokumen FIXED_LIDAR.md. Tidak ada perubahan mekanik diperlukan.
Heading scanning `_camera_heading` menghadap pusat pohon dikurangi yaw mount,
sehingga fixed memakai kamera depan. Label scan_motion_policy masih menyebut
left_camera: kesalahan pelaporan yang perlu dibetulkan, bukan bukti heading
badan salah. Target puncak batang adalah geometri teramati, bukan deteksi bunga.

### Bukti lama yang diverifikasi

`fixed_lidar_validation_20261014_082953` tercatat COMPLETE/disarmed untuk
hover pada jurnal. Log inspeksi `...20261014_085554` masih tersedia di
worktree awal (dibaca tanpa perubahan): 2.201 pasangan, RMSE 0.01348 m,
maksimum 0.03095 m. Pohon pertama berpusat `[8.9962,-.0083]`, enam sektor
valid. Posisi ini cocok dengan pohon world `[9,0]`; koordinat world hanya
pembanding audit, tidak masukan detektor/kontrol. Run terakhir berhenti
terdaftar t=229 s, sensor sehat, tanpa failure; SCAN pohon kedua pada anchor 8
menunggu depth baru untuk satu voxel unknown `[3.9,-10.1,1.9]`. Bukan bukti
lokalisasi gagal, bukan keberhasilan dua pohon.

### Batas dan pekerjaan lanjutan dari audit

- Queue IMU 500, QoS sensor bawaan depth 5; node mengintegrasi hanya IMU
  <= waktu scan, tetapi belum menunggu sampel IMU mencapai waktu scan.
  Estimator mengizinkan selisih <.1 s. Ini celah urutan integrasi pada kode;
  belum terbukti sebagai penyebab penghentian fixed sebelumnya.
- Peta menyimpan state FREE/OCCUPIED/UNKNOWN dan seen per voxel, unknown
  diinflasi sebagai blocker; endpoint di luar ROI tetap dipotong ke bagian
  ray di dalam ROI. No-return khusus organized Gazebo diverifikasi terpisah;
  NaN/dropout dan depth invalid tidak menjadi free.
- TTL free warisan **21.600 s**, map_timeout 1 s, cloud_timeout .5 s,
  pose_timeout .3 s. Timestamp scan baru tidak membuktikan semua sel koridor
  dipindai ulang. Belum ada histori kestabilan lokal adaptif; belum dapat
  mengklaim peta lama aman pada lingkungan dinamis. Tidak melonggarkan gate.
- Detektor memakai rentang tinggi dan fit XY global; bukti konsistensi batang
  beberapa lapisan belum ada. Coverage orbit harus tetap dibandingkan dengan
  evidence kandidat; enam sektor saja tidak membuktikan identitas pohon.
- SCAN awal masih memakai anchor lingkaran tinggi tetap; ketinggian sudah
  dibatasi ascent aman di bawah daun. Adaptasi ketinggian per koridor scan
  belum diimplementasikan. Jika blocked, tidak boleh dipaksa melalui obstacle.
- Dokumen CURRENT_VALIDATION.md/LOCAL_SETUP.md dan dependencies.lock.json
  memuat snapshot lama; jurnal bertanggal ini membedakan hasil terkini.

### Run reproduksi sesi ini

`reports/fixed_lidar_validation_20261015_142025`, run terdaftar
`ros_sensor_palm_farm_20261015_142028_11225_mapping`. Source tetap 818326c,
GUI mati, RTF .15, sensor LiDAR+IMU, tanpa depth pada hover. Setup PASS dan
ACK takeoff diterima. Hasil akhir akan dicatat setelah run selesai.
Map/cloud/voxel: reports/<run>/map_3d/map.json dan map.html. JSON numerik
akurasi, navigation dan localisation tersimpan di folder validation.

## Reproduksi berakhir sebelum disarm

Run 142025 menyelesaikan takeoff/hover/return, masuk LAND dan menyentuh tanah.
Auditor menghentikan run terdaftar pada t=86 s dengan
sensor_accuracy_exceeds_test_margin, masih armed. 772 pasangan: RMSE posisi
0,01072 m, maksimum 0,05862 m. Lima sampel terakhir t=85,6–86,0 mempunyai
error kecepatan .1986/.1985/.1671/.6726/.6192 m/s. Error muncul menjelang
serta saat kontak tanah, bukan drift posisi besar. Debug velocity Z terakhir
−.676 m/s meski pose dekat tanah. Log tidak menyimpan jumlah IMU masuk per
scan sehingga kehilangan sampel/urutan integrasi belum dapat dibuktikan sebagai
penyebab run ini. Tidak melonggarkan batas auditor .15 m/s atau quality gate.
Session.json stopped=true; tidak ada klaim disarm/COMPLETE.

## Implementasi setelah audit dan keputusan pengguna

Pengguna secara eksplisit mengubah kebijakan unknown: bawah −7° dianggap aman
tanpa batas range; atas +52° memakai z=r×cos(38°) untuk objek pada ray batas;
occupied terukur tetap obstacle; asumsi dipertahankan saat dropout. Ini
menggantikan larangan asumsi unknown awal hanya pada domain FOV tersebut.
Lapisan prior disimpan terpisah, tidak menulis FREE/seen terukur, ikut transport,
riwayat dan ekspor. Bukti sensor baru tetap mengalahkan prior. Gate kesehatan
flight tidak dimatikan. Tidak ada ground truth ke estimator/planner/kontrol.

Adaptasi selektif dari rotating-lidar 6779baf setelah memeriksa dependensi:
canopy_profile/kurva scan 3D + guard seluruh kurva, repair anchor sudut yang sama,
validasi batang beberapa lapisan, QoS IMU depth 256 BEST_EFFORT, menunggu IMU
mencapai timestamp scan, serta diagnostik jumlah/interval IMU. Mount fixed,
kamera depan, robust_planes khusus fixed dan pengamatan depth tetap dipertahankan.
Ascent memakai guard dibulatkan konsisten ke resolusi voxel. Smooth approach
blend/global-route/adaptive map-age rotating belum diadopsi; tidak mengklaim
peningkatan flight dari mekanisme tersebut.

Isolasi: default fixed ROS domain 96, Gazebo partition tersendiri dan ROS log
per run; port masih eksklusif bersama. Stop orphan SITL mem-resolve symlink
executable agar identitas program cocok tanpa memperluas cakupan penghentian.
Pengguna meminta RViz sesudah approach; auditor mengamati transisi
APPROACH→ASCEND/SCAN, membuka satu proses RViz privat, lalu menutupnya saat
run selesai. Awal tetap headless, tanpa Gazebo GUI/window kamera.

Tes awal 330: 329 lulus, satu ekspektasi guard .5 m perlu disesuaikan ke .6 m
karena voxel .2 m. Tes FOV menemukan delta kosong bertipe float pada mask prior;
diperbaiki menjadi bool. Suite 344 berikutnya: 340 lulus, empat tes repair
belum menyertakan helper fixture; fixture dilengkapi dan tes diulang. Hasil
final dicatat sesudah pemeriksaan selesai. Belum ada flight untuk kode baru.

### Hasil regresi final sebelum flight baru

Build tiga paket berhasil. **344 tes lulus, 0 failure/error/skipped**, 60,82 s
pytest, domain tes 87; 26 tes terarah perbaikan/visual juga lulus. Shell syntax,
syntax auditor dan diff whitespace bersih. Implementasi adaptasi misi/FOV/RViz
sudah diuji unit; hasil flight kode baru masih menunggu run berikutnya.

## Konflik QGroundControl sebelum launch baru

Setelah commit 97d045b dipush, host ss -lunp menunjukkan UDP 14550 dimiliki
QGroundControl PID 15502. Tidak ada Gazebo/SITL aktif; QGC tidak dihentikan.
Fixed runner sekarang memilih UDP 14650, diterapkan konsisten pada SITL,
URL loopback MAVROS, guard startup dan manifest session. Default launcher
umum tetap 14550. UDP 9002/TCP 5762 tetap eksklusif; ini bukan izin menjalankan
dua simulator. Perubahan diuji sebelum commit/push dan flight berikutnya.

Regresi port + adapter ROS + inspection loop: **37 lulus**, 0 gagal. Shell
syntax dan diff whitespace lulus. Endpoint 14650 tetap loopback-only; input
port invalid ditolak. Validasi koneksi aktual menunggu setup run berikutnya.


## Run inspection 1949191 dihentikan atas instruksi pengguna

Run `reports/fixed_lidar_validation_20261015_145348`, session
`.dependencies/ros_sensor_palm_farm_20261015_145350_19898_mapping` mencapai
enam sektor valid pohon ID5 sekitar [9,0], kemudian approach pohon ID3.
RViz dibuka setelah approach pertama lalu ditutup sesuai permintaan. Run
dipause pada waktu simulasi 266.802 s, kemudian dihentikan melalui session
terdaftar. Tidak ada klaim dua pohon/return/landing/disarm selesai pada run ini.

## Guidance memori dan penyambungan lintasan untuk seluruh fase yang relevan

Pengguna meminta mengurangi waktu misi, bukan hanya mengubah approach. Audit
kode menemukan approach/return masih memilih target geometris 2 m dan kurva
baru mulai dari kecepatan nol. Guidance history rotating pada revisi 6779baf
dibaca dan diadaptasi tanpa mengganti mounting/estimator fixed. Worker baru
mempertahankan bukti FOV terpisah dari free terukur. Local planner tetap
mengesahkan prefix; unknown/occupied, kurva, braking dan gate kesehatan tetap.

Penyambungan async menggunakan reference PVA terjadwal, termasuk dt yang
melintasi sambungan, dengan fallback pada stopping endpoint lama. Scan mencoba
24 anchor dan tetap mempertahankan evidence kamera per sektor. Intermediate
history endpoint tidak dihitung sebagai waypoint inspeksi yang selesai.
Profil transit .40 m/s, scan/default .30 m/s, ascent .12 m/s. Batas tracking
tidak dinaikkan; menaikkannya mengurangi cadangan clearance dan bukan cara
langsung menaikkan reference speed.

Tes baru semula gagal 18/26: retiming satu quintic dengan kecepatan awal dekat
limit dan percepatan positif tidak konvergen meskipun durasi terus bertambah.
Ini terbukti pada fixture v=.29403 m/s, a=.019008 m/s²; batas kecepatan kurva
masih sekitar .318 m/s pada durasi 20 s. Ditambahkan bridge jerk terbatas
menuju percepatan nol, kemudian quintic core dengan seed durasi lebih pendek
dan sertifikasi batas turunan. Batas PVA awal dipertahankan saat retiming.

33 tes baru/terarah lulus, termasuk rejection stale/late/worker exception,
unknown/occupied baru, mismatch anchor, perubahan adaptive rate/fase,
profile kecepatan, full control tick dengan jitter, serta replay controller
6 m yang menyambung sambil bergerak dan lebih cepat daripada tiga leg 2 m.
Build pertama: 384/385 lulus; satu regresi hint return pada mode history-off
terdeteksi dan diperbaiki dengan mempertahankan perilaku legacy ketika flag
nonaktif. Regresi terarah sesudahnya: 47 lulus. Validasi config juga menangkap
cek salah terhadap separate_mapping (sebenarnya parameter ROS, bukan JSON);
cek tetap dilakukan di node sesuai parameter runtime. Generator hover dan
inspection berhasil diload sesudah koreksi. Hasil suite final dicatat berikut.

Pengguna secara eksplisit meminta simulasi jangan dimulai. Tidak ada flight
baru selama perubahan ini. Klaim minimum waktu global atau keberhasilan misi
dua pohon belum tervalidasi. Rencana ini adalah konstruksi engineering lokal.

## Riset SUPER atas permintaan pengguna

Sumber primer yang diperiksa: [paper Science Robotics](https://doi.org/10.1126/scirobotics.ado6187),
[PDF pada repository penulis](https://github.com/hku-mars/SUPER/blob/master/misc/scirobotics.ado6187.pdf),
[kode planner](https://github.com/hku-mars/SUPER),
[CAD/BOM hardware](https://github.com/hku-mars/SUPER-Hardware),
[diskusi author tentang MPC](https://github.com/hku-mars/SUPER/issues/5), dan
[status/reproduksi MPC 2026](https://github.com/hku-mars/SUPER/issues/57).
SUPER adalah proyek HKU MaRS yang ditampilkan Livox. Planner dan optimizer
backup tersedia; controller MPC asli belum ditemukan dirilis pada sumber
resmi yang diperiksa. Foto menunjukkan MID-360 di atas badan; extrinsic
identik dengan konfigurasi kita belum dibuktikan dari CAD. Paper membedakan
exploratory trajectory dan committed trajectory dengan backup berhenti pada
known-free, termasuk pemotongan koridor oleh FOV. Ini tidak membenarkan
asumsi bahwa seluruh ruang di luar FOV aman. Belum mengimpor kode SUPER atau
mengklaim planner kita sebagai SUPER.

Hasil final: build tiga paket berhasil (2.39 s); **385 tes lulus**, 0 error,
0 failure, 0 skipped, pytest 69.88 s. Log build lokal:
`/tmp/fixed_navigation_build_final_20261015.log`; hasil XML:
`build_ros/polinasi_nav/pytest.xml`. Tidak ada simulator dijalankan.


## Flight d3f578b: penghentian pada prediksi kecepatan pengereman

Pengguna mengizinkan satu run dan meminta berhenti serta menjelaskan masalah
jika gagal. Run headless `reports/fixed_lidar_validation_20261015_203415`,
session `.dependencies/ros_sensor_palm_farm_20261015_203417_3904_mapping`.
Isolasi domain 96, partition polinasi_fixed_lidar, UDP MAVROS 14650; tidak ada
simulator/konflik port sebelum launch, tidak membuka window. Setup, GPS-off
origin/home, takeoff 2 m dan pemilihan kandidat pertama ID3 berhasil.

Handoff bergerak diterima t=32.7 s pada v=.391831 m/s. Selisih P/V/A reference
di batas adalah [0,1.60e-15,1.56e-12]. Dua kandidat disubmit, satu diterima,
satu ditolak; rejection kandidat bukan fault misi. Belum mencapai ascent/scan.

Fault pertama t=33.85 s: mission BRAKE, failure insufficient_stopping_clearance,
health_reason kosong, tracking error .02414 m, measured speed .398997 m/s.
Auditor otomatis meminta penghentian session. Verifikasi sesudah cleanup:
session stopped=true; tidak ada proses simulator/SITL/run ini atau port
9002/14650/5762 aktif. Tidak restart, tidak memperbaiki kode flight pada turn
ini, sesuai instruksi berhenti dan menjelaskan.

Audit kode: guard actual-body di control.py menggabungkan grid.safe dan
brake_safe ke nama fault yang sama. brake_safe dapat menolak predicted_speed_limit
sebelum memeriksa kurva pengereman terhadap obstacle. PID menulis percepatan
akhirnya kembali ke controller.command_a, sehingga guard tick berikutnya
menggunakan percepatan terbit terakhir, bukan hanya feedforward nominal.

Rekonstruksi offline memakai velocity bertimestamp 33.8 yang normanya persis
sama dengan measured_speed pada fault, dan output PID t=33.8:
[.0499195080,-.0088991403,.0006690415] m/s². Dengan implementasi Brake yang
sama, pada stub peta SELURUHNYA bebas, brake_safe menolak dengan alasan
predicted_speed_limit: prediksi puncak .401588908 m/s > batas .400000 m/s,
durasi pengereman 2.393981 s, reaction_time .35 s. Bukti numerik tersimpan di
braking_reconstruction.json pada direktori validasi. Ini membuktikan penolakan
dinamis tersebut dapat terjadi tanpa obstacle. Guard live tidak merekam
subreason/witness voxel, sehingga tidak mengklaim rekonstruksi seluruh peta
atau tabrakan fisik. Nama fault clearance saja tidak cukup untuk menyimpulkan
ada daun/unknown yang menutup jalur.

Konfigurasi transit .4 sama dengan batas umum speed .4, sehingga cadangan
terhadap koreksi PID dan variasi velocity sensor sangat kecil. Reference
braking sebelumnya lolos; actual velocity baru bersama percepatan terakhir
menghasilkan prediksi melewati batas. Unit/replay sebelumnya belum membuktikan
ketahanan kombinasi sensor velocity baru + output PID sebelumnya di dekat cap.
Rencana perbaikan: diagnosis subreason actual-body, profil speed dengan cadangan
dinamis PID/braking, dan regresi input rekaman. Tidak melonggarkan clearance,
health timeout, atau klasifikasi unknown agar lolos. Belum diimplementasikan.

Lokalisasi tetap healthy/tracking pada sampel akhir; IMU 20 sampel/scan dengan
interval maksimum .005 s. Audit 250 pasangan: RMSE posisi .01406 m, maksimum
.03159 m. Tidak ada bukti localisation_loss atau overspeed terukur sebagai
penyebab fault ini. Traceback KeyboardInterrupt muncul SESUDAH permintaan
stop saat worker sedang menunggu antrean, bukan exception planner awal.

Map tersimpan: `reports/ros_sensor_palm_farm_20261015_203417_3904_mapping/map_3d/`
berisi map.json (~15 MB), map.html (~15 MB), mission_status.json dan
navigation_debug.jsonl. Belum ada inspeksi sektor, return, landing, atau disarm
yang selesai pada run ini. Penerbangan sensor ini bukan validasi FAST-LIO.
