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


## Implementasi lima bagian adaptasi SUPER setelah audit braking

Pengguna meminta kelima pekerjaan dilaksanakan. Baseline 9d53448 bersih;
rotating-lidar tetap 6779baf. Ditambahkan super_policy.py dan evaluator offline,
integrasi ke continuous/BoundaryTrajectory, PID, fail/BRAKE, handoff, config,
generator fixed dan status ROS. Tidak ada perubahan mounting, FOV, timeout,
clearance, final publisher MAVROS, atau pemakaian GT pada estimator sensor.

Cadangan kecepatan dihitung dari Brake + uncertainty velocity .03 m/s + ekstra
koreksi PID .15 m/s². Tes input rekaman mereproduksi predicted_speed_limit
.401589 m/s; reference cap baru .34897 m/s. Tes awal replay masih melambat
seluruh kurva 1.4x karena peak prediksi .400096 m/s lolos sampling pre-profile
namun tertangkap sertifikasi .05 s. Ditambahkan scenario check pada retiming
lokal dan cadangan numerik konservatif; sertifikasi akhir tetap sama.

17 tes baru lulus: reproduksi input fault, PVA retiming, profil sebelum wall,
occupied/unknown pada hull, backup PVA/akhir diam, invalidasi cache, tiga fase
approach/scan/return, dropout tetap abort, controller PID dengan noise, dan
async moving handoff bersama seluruh kebijakan baru. Targeted legacy 51 lulus.
Targeted ROS adapters sempat 72/73 karena import approach_viewer pada pengujian
terisolasi tidak memiliki tools di sys.path; ini masalah harness/order import,
bukan exception node runtime. Suite lengkap dipakai untuk verifikasi akhir.

Evaluasi sintetis reference: straight 21.49→15.02 s, turn 32.24→23.93 s,
vertical path 32.24→25.51 s; bukan flight. CIRI belum diport, dievaluasi melalui
kode primer dan baseline voxel convex box. MPC lama menolak envelope
command_speed saat ini; PID dipertahankan sampai ada alasan flight dan model
baru yang cocok. Detail/provenance: docs/SUPER_ADAPTATION.md dan output numerik
reports/super_adaptation_evaluation.json. Hasil build/flight dicatat berikut.

Build final: tiga paket berhasil; **402 tes lulus**, 0 error/failure/skipped,
80.30 s pytest. Tes tambahan buffer canopy backup scan: **1 lulus**.
Log build lokal `/tmp/fixed_super_adaptation_build.log`. Commit/push sebelum
flight baru; tidak menjalankan suite berat bersamaan dengan flight.


## Flight headless 3408675, fault planner dan penghentian

Instruksi terbaru: simulasi tanpa window dan berhenti jika ada masalah.
Run validasi `reports/fixed_lidar_validation_20261015_211854`; runtime
`.dependencies/ros_sensor_palm_farm_20261015_211856_11929_mapping`.
ROS domain 96, partition polinasi_fixed_lidar, UDP MAVROS 14650/GZ 9002,
TCP SITL 5762. Tidak ada GUI dibuka. Auditor menghentikan run ketika fault;
session.json stopped=true dan stop_requested=true. Stop ulang terdaftar
tidak menemukan proses aktif; pemeriksaan port/proses tidak menunjukkan sisa
run. Tidak mematikan proses sesi lain dan tidak menjalankan ulang simulasi.

Pada t=22.8 s: SURVEY→BRAKE, tree_phase APPROACH, tree ID3,
failure survey_planner_error, replans=0. Takeoff mencapai altitude
1.996857 m; belum ada sektor inspeksi valid. Health lokalisasi tetap baik.
138 pasangan audit posisi: RMSE .0103701 m, maksimum .026079 m.
Reference speed .0004432 m/s, measured speed .01229 m/s, tracking error
.02054 m. Backup eksplisit valid, executed=true, trigger survey_planner_error,
live_rechecked=true, durasi .3 s; status ini tidak membuktikan keselamatan
semua fault atau keberhasilan landing/disarm sebelum simulator dihentikan.

Worker exception ditangkap generik sehingga type/message hilang. Tidak ada
bukti dari traceback shutdown untuk menyimpulkan exception asal. Rekonstruksi
offline peta ekspor t=22.811 s/observasi 22.6 s, window [-5,-5.2,-.6],
shape 50×50×40, start [.0126238,-.0174908,1.9887571], endpoint
[3.9017264,-.0192487,1.9950210], memunculkan ValueError
no robust route braking envelope. Tiga ruas awal lolos; semua lima faktor
speed pada ruas terminal gagal (braking_box_blocked/reaction_segment_blocked).
Profil meminta speed maju pada endpoint, walau endpoint kurva harus diam.

Perbaikan offline: pre-profile ruas terminal memakai taper ke nol; checker
kurva/PID/live-map tetap utuh. Tidak mengubah occupied/unknown, FOV pengguna,
clearance, batas speed/acc/jerk atau sensor timeout. Exception worker kini
mencatat gate, exception_type, message dalam route_failure_diagnostics.
Regresi dinding diperjelas untuk menguji perlambatan sebelum tikungan,
terpisah dari terminal berhenti. Ditambahkan unknown frontier dengan terminal
berhenti dan regresi penyimpanan detail exception.

20 tes adaptasi lulus. Rekonstruksi baru: geometri safe, durasi awal
14.603775 s, sertifikasi braking seluruh kurva .05 s lolos pada upaya kedua
dengan durasi 20.445285 s. Angka ini lintasan offline, bukan flight. Data di
`reports/fixed_lidar_validation_20261015_211854/terminal_profile_reconstruction.json`.
Karena peta ekspor bukan snapshot worker, penyebab run asli tetap inferensi
kuat; bug pre-profile terbukti lewat reproduksi dan regresi. Tidak ada klaim
flight perbaikan baru berhasil. Build/suite akhir dicatat setelah selesai.

Map/cloud/voxel tersimpan pada
`reports/ros_sensor_palm_farm_20261015_211856_11929_mapping/map_3d/`
(map.json, map.html, mission_status.json, navigation_debug.jsonl).
Log numerik pada accuracy*.jsonl dan accuracy.summary.json di direktori
validasi. Tidak ada dua pohon selesai, return, landing atau disarm tervalidasi.


Verifikasi akhir perbaikan terminal: tiga paket berhasil build, seluruh
**405 tes lulus**, 0 error/failure/skipped, 80.00 s. Log lokal
`/tmp/fixed_terminal_build.log`. Setelah itu ditemukan status route_speed_profile
belum disalin dari clone worker ke controller live. Ditambahkan penerusan
status untuk rute awal dan moving handoff; **44 tes adaptasi/handoff lulus**
(18.33 s), termasuk assertion diagnostik pada replay async. Log
`/tmp/fixed_terminal_diagnostics_tests.log`. Tidak ada flight baru untuk kedua
perbaikan ini. Perubahan teruji di-commit/push hanya ke fixed-lidar.


## Run c684d4c headless dan riset lanjutan kegagalan SCAN

Pengguna mengizinkan run baru, kemudian meminta perbandingan approach dan
riset solusi. Run `reports/fixed_lidar_validation_20261015_214724`, session
`.dependencies/ros_sensor_palm_farm_20261015_214726_17361_mapping`.
Semua window off; domain96/partition polinasi_fixed_lidar/port9002,14650,5762
terpisah dari proses lain. APPROACH tree3 20.65→59.40 s (38.75 s),
ASCEND→SCAN70.95 s, failure71.35 s: survey_worker/ValueError/
no robust route braking envelope. Tidak ada sektor inspeksi selesai.
Altitude3.1254 m; ascent_stop_reason observation_guard_limit;
tracking error .02548 m; localisation healthy/tracking. Audit623 pasangan,
posisi RMSE .014188 m, maksimum .026264 m. Backup valid/executed/live_rechecked.
Auditor stop; session.stopped=true, port/proses runtime sudah kosong.
Tidak restart. Landing/disarm sebelum penghentian simulator tidak tervalidasi.

Moving handoff31.20 s diterima, PVA jump [0,1.16e-14,2.03e-12].
Satu handoff berikutnya ditolak planner_not_ready_at_boundary; ini bukan
exception akhir. Run sebelum adaptasi SUPER 145348: approach tree5
20.45→71.65 s=51.20 s; selisih12.45 s/24.3% tidak boleh dianggap peningkatan
terkontrol karena pohon/rute berbeda. Baseline203415 tree3 gagal33.85 s
sebelum approach selesai. Waktu di atas detik simulasi; targetRTF .15 berarti
38.75 detik simulasi sekitar258.3 detik waktu nyata bila RTF tepat .15,
bukan pengukuran durasi hardware atau pengukuran wall-clock persis.

Riset primer: SUPER, TOPP-RA, Ruckig dan TOPPQuad. Audit menemukan lima
kandidat scan hanya turun hingga .09 m/s; failure bukan bukti tidak ada speed
lain yang feasible. Cadangan velocity/PID serta geometri tidak hilang ketika
nominal speed diperkecil. Belum ada bukti witness untuk sumber blokade SCAN.
Rekomendasi dan batas integrasi lengkap pada docs/SUPER_ADAPTATION.md bagian
riset penolakan braking. Tidak ada perubahan algoritma, tes baru, atau flight
baru pada tahap riset; perhitungan numerik offline menggunakan Brake yang
sudah ada. Dokumentasi riset disimpan/di-commit/push ke fixed-lidar.


## Loop otomatis diizinkan — iterasi pemulihan planner

Pengguna menginstruksikan implementasi→simulasi→stop bila fault→riset→perbaikan
dan ulangi sampai selesai atau perlu keputusan baru. Ini menggantikan batas
sebelumnya yang tidak mengulang simulasi setelah satu fault. Tetap headless,
stop hanya run terdaftar, tanpa hardware dan tanpa mengubah rotating-lidar.

Implementasi berdasarkan riset sebelumnya: refinement faktor speed sampai
.01 dari cap, bukan hanya .3; witness tiap penolakan dan exception bertipe
RouteSpeedEnvelopeError. Penolakan chord tidak langsung fatal: planner
membangun kurva PVA dan harus lolos gate geometri/braking sebenarnya. Full
curve braking juga memakai canopy guard pada scan/ascent sebagaimana backup.
Kegagalan kandidat menjadi hold/retry hanya jika stop command masih safe;
timeout blocked_timeout yang lama tetap dihitung sampai sukses, tidak reset
tiap retry. Jika hold tidak aman atau timeout habis, fault tetap terjadi.
Moving clone yang gagal tidak mengganti lintasan committed.

Snapshot worker numerik menyimpan state/seen, mask FOV, blocked/actual mask,
origin/shape, config, PVA dan points/controls/durations pada subdir
map_3d/planner_failures; maksimum delapan snapshot per run. Path tersedia
pada diagnostik. Mapping terpisah tetap menulis audit pada direktori run yang
sama melalui subdir tersendiri. Tes awal snapshot mengungkap numpy.int64
belum serializable; diperbaiki sebelum flight.

Build tiga paket dan **409 tes lulus**, 0 error/failure/skipped, 82.13 s.
Log `/tmp/fixed_recovery_build.log`. Empat regresi baru mencakup speed lebih
rendah dari .09, fallback chord tetap disertifikasi, timeout/unsafe hold,
dan snapshot exact serta bounded. Belum flight pada saat commit ini.
Ini refinement bounded lokal, bukan implementasi TOPP-RA/Ruckig/MINCO penuh.
Tidak mengubah sensor timeout, clearance, mounting atau kebijakan FOV.


## 16 Oktober 2026 — loop dilanjutkan, retiming braking lokal

Run 9b63761 `fixed_lidar_validation_20261015_221813` melewati fault SCAN lama.
APPROACH21.35→53.90 s=32.55 s; ASCEND→SCAN65.45 s. Reference scan diterima,
namun ditemukan masalah performa: durasi hasil optimasi42.8142 s menjadi
164.4750 s karena empat global slow(1.4), dengan7755 braking checks,
27.735 s wall sertifikasi/28.103 s worker. Satu ruas constrained .045 m/s
membuat ruas bebas ikut lambat. Run dihentikan sengaja untuk audit performa,
bukan karena fault flight: stop_reason registered_simulation_stopped,
last t77.3 s/failure kosong, belum sektor selesai. Session stopped=true;
681 pasangan audit RMSE .0145843 m, maksimum .0325121 m. Tidak ada klaim
landing/disarm. Map/cloud dan snapshot worker di direktori run terkait.

Riset ulang TOPP-RA dan optimizer backup SUPER menguatkan pengaturan waktu
sepanjang path, bukan skala global terburuk. Implementasi lokal engineering:
retime ruas gagal, turunkan PVA knot bersama, sesuaikan tetangga jika perlu,
cek bounds turunan konservatif dan geometri lagi. Start/end PVA dipertahankan;
seluruh kurva tetap wajib lolos braking setelah usulan. Moving BoundaryTrajectory
mempertahankan fallback retiming C2 lama; tidak memaksa perubahan boundary.
Ini bukan TOPP-RA atau MINCO penuh dan bukan optimum waktu global.

Sertifikasi per-segmen mencakup kedua endpoint dan interval .05 s. Cache
hanya untuk kontrol/durasi identik dalam job frozen, ditambah identity,
origin/res, hash mask blocked dan parameter braking untuk invalidasi.
Peta berubah bahkan tanpa version increment tidak dapat memakai hasil lama.
Snapshot exact sekarang juga disimpan pada penolakan braking pertama sebelum
retiming, sehingga kandidat diterima setelah perlambatan tetap bisa diaudit.
Status menampilkan local_braking_retiming, cache hits dan tanda recovered.

Tes awal terlalu mengasumsikan ruas tetangga pertama tidak berubah; retiming
yang menjaga bounds memang dapat merambat satu ruas. Skenario diperbaiki
agar ruas jauh berada di luar tetangga. Kemudian build tiga paket dan
**412 tes lulus**, 0 error/failure/skipped,85.94 s. Log lokal
`/tmp/fixed_local_retime_build.log`. Tiga regresi baru: C2/distant cruise,
moving PVA tidak ditimpa, cache invalidasi unknown. Commit/push sebelum
flight headless berikutnya; hasil flight belum tervalidasi saat commit.


## 16 Oktober 2026 — endpoint terlalu dekat crop window

Setelah daemon restart, verifikasi HEAD301b206 bersih,412 tes lulus,
remote masih9b63761. Push301b206 diselesaikan; tidak ada port/proses simulasi
aktif sebelum run baru. Headless run `fixed_lidar_validation_20261016_113409`,
session `.dependencies/ros_sensor_palm_farm_20261016_113411_5290_mapping`.
Takeoff sukses; treeID4 dipilih, APPROACH belum bergerak karena kandidat
ditolak. Pada snapshot,12 retiming masih gagal pada nominal speed .002593 m/s:
uncertainty/PID scenario .032593 m/s, reason braking_box_blocked,
witness outside_window pada cell[4.1,-.1,2.1]. Komputasi42.16 s wall/11747
checks. Ini masalah endpoint/window, bukan bukti daun atau ICP failure.
Run dihentikan untuk audit, stop_reason registered_simulation_stopped;
last t42.65 s, failure kosong,337 pasangan RMSE .0137754 m/max .0292188 m.
Session stopped=true. Tidak ada SCAN/return/landing/disarm tervalidasi.

Snapshot exact `failure_725308285617.npz` membuktikan endpoint
[3.993495702,-.005659918,2.001034563] statically safe tetapi near-zero robust
stop gagal outside_window. Memperlambat lagi tidak menghilangkan uncertainty
floor. local_route_goal tadinya memilih endpoint hanya dengan body safe.

Riset primer FASTER (Tordesillas et al., https://arxiv.org/abs/2001.04420)
menegaskan perlunya backup/terminal feasible pada free-known space; bukan
alasan menganggap crop unknown aman atau memaksa berhenti di frontier.
Usulan engineering lokal: evaluasi moving-side near-zero robust stop pada
calon endpoint history-prefix, pilih endpoint terdahulu bila perlu. Full
curve/live/PID tetap wajib. Tidak memperbesar map, menambah load atau
melonggarkan clearance/unknown. Terminal reserve precheck menghentikan
retiming sia-sia pada endpoint yang tidak layak. Timeout blocked lama kini
tetap diberlakukan meski worker pending, bukan menunggu komputasi selesai.

Replay independen snapshot yang sama: endpoint baru
[3.893746026,-.005458583,2.001069094], sekitar10 cm lebih pendek. Dengan
guidance sudah dipilih, SurveyController membangun kurva/braking dengan
ready=true,failure kosong,2 upaya,496 checks,1 cache hit,16.6020634 s durasi
dan1.18694371 s wall. Ini replay lintasan, bukan seluruh ROS/flight ulang.
Regresi menambah endpoint static-safe tetapi reserve-outside dan pending
worker yang tidak boleh memperpanjang blocked_timeout. Hasil build dicatat
setelah selesai sebelum push/run berikutnya.

Build tiga paket berhasil; **414 tes lulus**, 0 error/failure/skipped,88.63 s.
Log `/tmp/fixed_terminal_reserve_build.log`. Commit/push sebelum flight baru.


## 16 Oktober 2026 — PID menolak backup setelah headroom diperbarui

Run57c105d `fixed_lidar_validation_20261016_114500` membuktikan perbaikan
endpoint: initial trajectory16.60056 s,2 upaya,495 checks,1 cached segment,
1.65306 s worker; dua moving handoff diterima. APPROACH20.25→58.90 s
(38.65 s), ASCEND→SCAN72.30 s. Fault73.75 s pid_final_safety; auditor stop
dan session stopped=true, stop ulang tidak mengirim signal ke proses lain.
649 pasangan posisi RMSE .0149772 m,max .0393737 m, localisation tracking.
Tidak ada sektor scan/return/landing/disarm selesai. Preset actual flight
tracking_limit/tracking_margin .2 m tetap; angka .1 m sebelumnya adalah
fixture tes, bukan batas preset run ini.

PID log: position error [.04949,-.01327,.00586] m, actual speed .01527 m/s,
reference velocity/acceleration nol pada [6.8914131,-.0108174,3.3192240].
Actual body brake dan reference brake lolos untuk correction1,.5,0.
Semua explicit backup gagal no_bounded_collision_free_stop pada grid guard.
Ini berbeda dari planner exception. Snapshot worker SCAN72.5 s masih memiliki
stationary brake safe; bukan bukti grid live73.5 s tetap safe.

Rekonstruksi ekspor akhir map73.76 s/observasi73.5 s: body safe pada reference
tetapi guard .2 m dan stationary backup tidak safe. Di3.1192240 m (20 cm
lebih rendah), body/guard/backup lolos; di3.2192240 dan3.2692240 belum lolos.
Ini menunjukkan headroom terpakai setelah update, bukan bukti drift permanen
atau menabrak. Ekspor bukan exact live snapshot PID; snapshot PID ditambahkan
untuk run berikut. Safety abort dipertahankan saat guard tidak terpenuhi.

Riset FaSTrack https://arxiv.org/abs/2102.07039 membedakan model planner dan
tracking serta memakai TEB terhitung. Implementasi kita TIDAK menghitung TEB
atau mengklaim jaminan FaSTrack. Usulan engineering yang diuji: satu voxel
tambahan pada perencanaan fixed+feedback+adaptive scan. Ascent guard planning
base .2 + .2 m voxel, eksekusi backup tetap base .2 m. Canopy profile memilih
target di bawah observed ceiling minus reserve, dengan pencarian sampai
target+reserve agar area lapang tetap dapat mencapai target geometris.
Tidak mengubah occupied/unknown/FOV, timestamp, timeout, badan atau clearance.
Margin bounded ini bukan bukti aman dari arbitrer noise/gerakan objek;
freshness dan abort pada invalidasi tetap berlaku.

Diagnosis guard kini menelusuri footprint tambahan di atas badan, sehingga
unknown/occupied sumber inflasi terlihat. Explicit backup menyimpan subreason
dan start_safe. Fault PID menyimpan exact numeric live guarded grid dan input
PID pada failure_critical.npz, bahkan setelah delapan snapshot biasa terisi
(maksimum8 biasa+1 kritis). Tidak memakai GT.

Tes awal snapshot PID gagal pada fixture SimpleNamespace tanpa c; pencatatan
kini hanya dilakukan bila audit directory tersedia. Regresi planning reserve
tidak boleh mengharuskan selisih actual height tepat1voxel bila hysteresis
lama membatasi tinggi: tes diperbaiki untuk memeriksa reserve terhadap ceiling.
Tes baru: one-voxel ceiling update saat ascent, clear target/roof pada scan,
witness unknown guard, dan critical snapshot saat budget penuh. Build/suite
akhir dicatat berikut sebelum push dan flight baru.

Build tiga paket berhasil; **418 tes lulus**,0 error/failure/skipped,87.86 s.
Log `/tmp/fixed_map_reserve_build.log`. Commit/push sebelum flight berikutnya.


## 16 Oktober 2026 — dua pohon valid, bug metadata pada return

Run879d89e `fixed_lidar_validation_20261016_120222`, session
`.dependencies/ros_sensor_palm_farm_20261016_120224_11883_mapping`:
TAKEOFF9.4; tree2 APPROACH20.65→65.75 s (45.10 s), ASCEND→SCAN75.45 s,
tree2 complete154.25 s dengan6/6 sektor valid. Tree4 APPROACH154.30→202.25 s
(47.95 s), ASCEND→SCAN223.10 s, complete304.35 s dengan6/6 sektor valid.
RETURN304.40 s, worker fault305.30 s: KeyError '3', survey_planner_error.
Auditor stop, session stopped=true, stop ulang tidak men-signal proses lain.
2962 pasangan pose RMSE .0172643 m/max .0423100 m; sensor tracking.
Tidak ada return/landing/disarm berhasil, meskipun dua inspeksi selesai.

Cadangan planning bekerja pada kasus run ini: scan pertama31.2539 s setelah
dua upaya/3.2091 s worker; scan kedua29.3012 s/dua upaya/3.8409 s worker.
Initial approach16.6536 s/1.5653 s worker. Sembilan handoff diterima,11
ditolak dalam kondisi guard/deadline/reference; bukan misi minimum waktu
global atau jaminan statistik. Kamera continuous_front_tree dan12 sektor
valid tersimpan pada tree_scan_records/accuracy.navigation.jsonl. Tidak
menyimpulkan keberhasilan misi penuh dari orbit/coverage saja.

Root cause direproduksi tanpa simulator: SurveyController stateRETURN,
command[3,0,2], batch_end_index23 dan batch_knots{16:3,23:9};
_prepare_trajectory→_trajectory_from_points→knots[knot] menghasilkan
KeyError3 pada survey.py. _plan_route dulu mengembalikan return route
sebelum mereset metadata batch scan; densify route baru memakai indeks
route lama. Ini lifecycle data, bukan error koordinat, sensor, atau braking.

Perbaikan: reset batch_end_index/batch_knots pada awal setiap _plan_route;
_trajectory_from_points di luar SURVEY tidak membawa metadata anchor scan.
Tidak memfilter indeks invalid diam-diam pada fase scan. Worker exception
kini menyimpan traceback agar lokasi remote diketahui. Regresi mencakup
return dengan metadata scan stale, direct return curve, dan anchor mapping
SURVEY setelah densifikasi tetap benar. Replay async/PID yang tadinya selesai
di stateRETURN diperpanjang sampai home/DESCEND.

Tes awal salah mengharapkan home pada mode legacy yang memang memilih prefix
2 m. Skenario diperbaiki ke history-route mode yang dipakai flight, bukan
mengubah algoritma agar cocok tes. Suite lengkap awal420pass/1assertionfail;
suite final diulang setelah koreksi fixture. Hasil dicatat sebelum push/run.
Ini perbaikan invariant state; tidak memerlukan algoritma paper baru dan
tidak mengubah clearance/timeout/FOV/controller/publisher.

Build tiga paket berhasil; **421 tes lulus**,0 error/failure/skipped,90.85 s.
Log `/tmp/fixed_return_metadata_final_build.log`. Commit/push sebelum full run.


## 16 Oktober 2026 — full mission berhasil pada 1fb9b7c

Run `reports/fixed_lidar_validation_20261016_125204`, session
`.dependencies/ros_sensor_palm_farm_20261016_125206_20833_mapping`, menjalankan
HEAD1fb9b7c tanpa GUI/RViz (`POLINASI_RVIZ_AFTER_APPROACH=0`). Worktree clean,
branch fixed-lidar dan remote sudah sinkron sebelum launch. Audit host memastikan
UDP9002/14650 dan TCP5762 kosong. Domain96, partition polinasi_fixed_lidar.
Tidak ada perubahan kecepatan/controller/clearance/FOV pada iterasi flight ini;
peningkatan kecepatan ditunda sesuai arahan pengguna.

Hasil: **COMPLETE345.10 s, armed=false, FCU LAND, failure kosong**. Auditor
exit0 dan menghentikan hanya registered session; session.json stopped=true.
Audit port/proses setelah stop kosong. Tidak ada window dibuka.

Timestamp berikut memakai detik simulasi, bukan detik wall-clock:

| Fase | Mulai | Akhir | Durasi |
|---|---:|---:|---:|
| TAKEOFF, sejak sampel pertama | 9.15 | 20.55 | 11.40 |
| Pohon pertama APPROACH | 20.55 | 61.25 | 40.70 |
| Pohon pertama ASCEND | 61.25 | 70.95 | 9.70 |
| Pohon pertama SCAN | 70.95 | 153.00 | 82.05 |
| Pohon kedua APPROACH | 153.05 | 205.70 | 52.65 |
| Pohon kedua ASCEND | 205.70 | 211.70 | 6.00 |
| Pohon kedua SCAN, termasuk recovery | 211.70 | 300.35 | 88.65 |
| RETURN | 300.40 | 338.60 | 38.20 |
| DESCEND | 338.60 | 338.80 | 0.20 |
| LAND sampai COMPLETE/disarm | 338.80 | 345.10 | 6.30 |

Total sejak sampel pertama335.95 s (5 menit35.95 detik simulasi); sampel
pertama bukan bukti timestamp perintah takeoff. FCU berubah GUIDED→LAND338.85 s.
DESCEND .20 s hanya fase software sebelum menyerahkan landing ke FCU; bukan
klaim turun2 m dalam .20 s. Gerakan turun berlangsung pada fase LAND.

tree_scan_records berisi12 record inspection_valid=true, masing-masing6 view
untuk ID4 dan6. Pusat sensor yang disimpan [8.97735,-.02131] dan
[-4.50595,-9.00276], sehingga dua ID ini tidak menunjuk pusat yang sama.
Keduanya memakai continuous_front_tree; bukti kandidat tree6 memiliki low
stem support dan4 lapisan konsisten. Tinggi ascent berhenti2.91979 dan2.52019 m
karena observation_guard_limit; scan mengikuti profil tinggi yang tersedia.
Ini inspeksi target geometris, bukan deteksi bunga atau bukti mencapai puncak
batang sebenarnya. Ground truth tidak menjadi input estimator; estimator
experimental_scan_to_map_ICP_body_IMU, bukan FAST-LIO.

Transisi RETURN melewati titik KeyError lama dan mencapai home, sehingga
perbaikan lifecycle metadata kini sudah diuji flight penuh. Ada satu recovery
yang harus dibedakan dari abort fatal:252.70 s BRAKE/route_changed ketika
sertifikat suffix menemukan future segment15 di luar navigation window.
Diagnosis causes=[outside_window], footprint_outside_cells27; posisi nominal
dan aktual saat itu tetap safe. Controller menyelesaikan recovery254.45 s
(1.75 s), lalu meneruskan scan dan menyelesaikan misi. Tidak ada bukti dari
kejadian ini bahwa objek baru bergerak atau lokalisasi hilang. Delapan moving
handoff diterima,11 kandidat ditolak. Penolakan bukan seluruhnya fault misi.

3359 pasangan pose sensor vs GT audit menghasilkan RMSE .0182656 m,
maximum .0531658 m. Posisi sensor akhir [.0162765,-.0089144,.1993132] m;
error horizontal terhadap home[0,0] .0185578 m. Referensi navigation tetap
di hover saat FCU LAND sehingga tracking_error akhir1.80078 m tidak mengukur
error terhadap lintasan landing FCU. Clearance negatif saat kontak tanah
pada takeoff/landing juga tidak boleh dilaporkan sebagai clearance transit.
Rata-rata kecepatan terukur gabungan approach .21672 m/s, scan aktif .16315 m/s,
return .21737 m/s (integral kecepatan terhadap waktu simulasi / durasi fase).
Maksimum tracking error approach .14598 m, ascent .16151 m, scan aktif .10883 m,
return .08279 m. Tidak ada klaim optimasi minimum waktu atau perbandingan
SUPER sebelum-sesudah yang terkontrol dari satu run ini.

Artefak numerik di folder validation: accuracy.navigation.jsonl,
accuracy.localisation.jsonl, accuracy.jsonl, accuracy.summary.json, setup.json.
Peta tersimpan di
`reports/ros_sensor_palm_farm_20261016_125206_20833_mapping/map_3d/`:
map.json (54,002,806 byte), map.html (53,091,682 byte), navigation_debug.jsonl,
mission_status.json dan planner_failures/*.npz. Ekspor akhir345.151 s/observasi
344.9 s, frame map/resolusi voxel .2 m,561962 point,33886 occupied dan538775
measured-free voxels; unknown di luar daftar tetap implicit unknown.
Mask assumed-free/assumed-occupied dipisah dari pengamatan terukur.
Peta historis adalah artefak audit, bukan sertifikat keamanan seluruh farm.

Status validasi: implementasi returnfix sudah unit+flight (build tiga paket,
421 tes lulus sebelum flight; tidak diulang selama mengukur timing flight).
Run ini satu full nominal sensor-simulation mission pada farm statis dan
prior FOV eksplisit pengguna. Kebijakan unknown bawah/atas serta persistensi
prior saat dropout tetap aktif; prior bukan bukti ruang terukur aman. Tidak
menambah validasi hardware, CIRI/MPC SUPER, minimum waktu, atau flight fault
injection blocked-route/dropout/delay melalui keberhasilan run nominal ini.
Tidak perlu memperbaiki kode tambahan untuk menyatakan hasil nominal ini.
Dokumentasi hasil di-commit/push setelah audit; artefak besar tetap lokal.


## 20 Oktober 2026 — baseline world palm_farm_flowers

Pengguna meminta loop pada map palm_farm_flowers. Audit awal: fixed-lidar
fc84ca0 clean; rotating-lidar6779baf memiliki perubahan lokal pengguna dan map
flowers belum tracked. Tidak checkout/merge/menulis worktree rotating. Tidak
membaca credential/.aws. Tidak ada AGENTS pada root/ancestor/worktree.
Audit port/proses host UDP9002/14650/TCP5762 kosong sebelum persiapan.

Map baru29 pohon dengan individual realistic meshes dan bunga statis,
internal world name tetap polinasi_palm_farm. Posisi pohon jitter, collision
dan visual sama. Map baru bukan sekadar menambah bunga ke world lama.
Preset navigasi flowers bawaan mengaktifkan mock bunga dan mounting lama;
tidak dipakai untuk flight fixed. Tetap memakai preset fixed yang sudah
unit/flight, prior FOV pengguna dan kamera depan pitch-only. Kecepatan belum
dinaikkan. Misi memilih dua kandidat batang sensor, tanpa deteksi bunga.

Implementasi: selector scenario pada launcher fixed/validation/Gazebo/generic,
simulation_scenario di config dan package discovery aset flowers. Salin hanya
world/aset/generator terkait dari workspace pengguna. Mesh hasil generasi691 MB
di-ignore, generator+world+metadata+manifest dipush agar reproducible. Patch
protected_hashes melewati world realistic yang tidak ada pada branch ini.
Tidak menyalin controller/estimator/misi rotating terbaru. Ensure memeriksa
semua resource rujukan dan menulis SHA256 audit; run menyimpan world_assets.json.
Regresi baru memeriksa29 pohon, preservasi semua model obstacle setelah prepare,
fixed LiDAR/kamera/IMU, dua pohon sensor dan mock disabled, serta resource hilang
dan hash berubah. Coverage polygon lama hanya metadata configured farm,
bukan bukti terukur coverage world baru. Hasil suite/flight dicatat berikut.

Build tiga paket berhasil;424 tes lulus dalam94.15 s,0 failures/errors/skipped.
Log /tmp/fixed_flowers_build.log. Setelah perapian helper/provenance dan docs,
regresi flowers3 tes diulang lulus. Hash world dan32 mesh yang dirujuk
dibandingkan dengan workspace sumber sebelum flight; semua identik byte.
Tidak ada flight map baru yang diklaim sebelum menjalankannya.


## 20 Oktober 2026 — flowers run1 blocked_route dan repair endpoint scan

Run de9f53a: reports/fixed_lidar_validation_20261020_075230, session
.dependencies/ros_sensor_palm_farm_flowers_20261020_075233_30017_mapping.
Auditor menghentikan run pada116.95 s, BRAKE/SCAN/blocked_route, tree1 anchor2,
0 sektor valid. Session stopped=true dan audit host sesudahnya tanpa proses
atau port simulator aktif. Tidak ada GUI, tidak ada hardware.
1078 pasangan pose sensor vs GT audit: RMSE .00869650 m/max .02177817 m.
Health reason kosong, sehingga bukan bukti kegagalan lokalisasi.

Planner memilih detour pada approach; initial prefix39.8081 s, ruas dekat
occupied dibatasi .0523454 m/s oleh feedback braking. Moving handoff12 kandidat
ditolak camera_observation_required. Ini perlambatan yang teramati; iterasi
ini tidak menaikkan kecepatan atau menghilangkan camera gate.

Saat SCAN, peta baru membatalkan suffix104.15 s karena occupied dekat
[-9.1,.3,2.5] m. Current nominal/actual safe; recovery selesai104.85 s.
Mulai104.90 s retry kandidat terminal berulang, deadline116.90 s. Endpoint
[-10.00765678,.88567375,2.38323806] statically safe tetapi near-zero robust
arrival (.030001 m/s setelah ketidakpastian, PID acceleration scenario)
menyentuh inflated occupied/canopy mask. Ini witness konservatif hull box,
bukan bukti drone telah bertabrakan. Tidak menghapus occupied atau unknown.

Penyebab program terverifikasi lewat snapshot:
map_3d/planner_failures/failure_12003016626336.npz, allow_pickle=False.
Rekonstruksi blocked mask dari raw state/seen/FOV dan canopy guard identik
byte pada snapshot (guarded mask equality=true). Sebelumnya Survey batching
memeriksa posisi/route aman, tetapi tidak menyaring terminal stopping reserve
sebelum menambah anchor. Scan repair hanya dipicu occupied_goal; goal yang
statically safe tetapi tidak punya stopping reserve terus terpilih.

Perbaikan tahap awal memotong batch sebelum endpoint tidak feasible. Replay
lanjutan masih gagal pada anchor7 karena arah datang A* berbeda dari chord
command→goal. Karena itu diperbaiki pula route-aware repair: kandidat pada
sudut anchor yang sama, tinggi/outward dalam batas lama, diuji endpoint dari
arah segmen terakhir rute A* sebenarnya. Evaluasi kandidat maksimum24 per
repair. Termasuk floor tepat agar langkah voxel tidak melewatkan floor.
Full curve, canopy, braking dan PID final tetap menguji hasil; proposal ini
bukan sertifikat eksekusi sendiri. Anchor tidak dilewati, radius tidak dipotong,
coverage tidak bertambah hanya karena target diganti. Timeout tetap12 s.

Endpoint helper menggunakan persis near-zero direction/uncertainty scenario
terminal gate lama. Hook Survey default tidak mengubah endpoint policy untuk
controller lain; continuous scan menyaring ujung setiap prefix. Jika anchor
pertama gagal setelah A*, resampling dan replan dibatasi satu retry geometry
dalam job. Diagnosis scan_endpoint_rejections dan scan_target_diagnostics
mencatat anchor, witness, jumlah kandidat dan alasan rute gagal.

Replay exact setelah perubahan: anchor2→batch_end6 lolos full certificate,
durasi14.7038 s; anchor7 bergeser outward .20 m ke[-9.72377,1.39141,2.46702],
durasi4.08444 s; anchor8 outward .20 m ke[-10.15675,1.01898,2.38324],
durasi3.97717 s. Anchor9 juga lolos setelah retiming8.24617 s. Anchor10
unknown pada snapshot beku ditolak; tidak menganggap snapshot ini sebagai
prediksi observasi/flight masa depan. Menurunkan endpoint lama saja sampai
2.08324 m pada arah masuk lama tetap gagal; pergeseran lateral relevan.

Riset primer yang diperiksa ulang: FASTER https://arxiv.org/abs/2001.04420
dan public SUPER backup optimizer
https://github.com/hku-mars/SUPER/blob/master/super_planner/src/traj_opt/backup_traj_optimizer_s4.cpp.
Kelayakan backup merupakan bagian perencanaan; usulan engineering lokal di
atas bukan port optimizer/CIRI/FASTER dan tetap conditional pada prior FOV
pengguna. Tidak mengklaim measured-known-free guarantee untuk assumed-free.

Regresi targeted awal55 pass. Setelah tambahan route-aware repair24 pass;
suite controller16 pass termasuk tiga regresi baru: statically-free anchor
perlu diturunkan, prefix tidak memakai endpoint unstoppable, dan occupied
voxel nyata membuat arah detour gagal meskipun chord lulus. Regresi terakhir
membuktikan resampling pada sudut sama mengembalikan endpoint feasible.
Hasil build/suite lengkap dan flight berikut ditambahkan setelah tersedia.

Build tiga paket berhasil;427 tes lulus dalam100.47 s,0 errors/failures/skipped.
Log /tmp/fixed_scan_endpoint_build.log. Replay dan regresi bukan keberhasilan
flight; commit/push sebelum menjalankan ulang sensor mission flowers.


## 20 Oktober 2026 — flowers run2: cadangan berhenti terkena pembaruan peta

Run2dd946a reports/fixed_lidar_validation_20261020_081703, registered session
.dependencies/ros_sensor_palm_farm_flowers_20261020_081705_35609_mapping,
berhenti35.40 s dengan emergency_unavoidable:route_changed saat APPROACH,
sebelum menguji repair scan pada flight. Auditor stop, session stopped=true;
audit host port/proses kosong.259 pasangan pose, RMSE .00815984 m/max
.01821977 m, health tetap tracking. Tidak ada inspeksi pohon selesai.

Initial curve direct, worker1.4931 s; suffix dibatalkan sekitar34.502 s
oleh occupied di[-4.3,-.3,2.3], nominal dan actual pada posisi saat itu aman.
Cadangan PID terakhir34.45 s valid pada map285, duration2.0937103 s,
endpoint[-2.194565,-.664065,1.840921]. Fail route_changed memilih cached backup
yang compatible dan live-rechecked. Setelah~.9 s braking, map terbaru membuat
_braking menolak kelanjutan; log lama hanya menyebut emergency_unavoidable.
Tidak membuktikan tabrakan atau kegagalan lokalisasi.

Rekonstruksi end export35.415 s/observasi35.2 s ke navigation window akhir
[-7.2,-5.8,-.6], shape50x50x40: clearance -.054159091234535994 m tepat sama
dengan log (metrik clearance ini konservatif, bukan kontak geometri).
Brake direkonstruksi dari PID PVA terakhir34.45 s, duration identik2.0937103 s.
Pada elapsed .8/.85/.9/.95, remaining stop actual=True lolos, tetapi langkah
nominal berikut gagal occupied. Witness[-2.5,-.9,2.7] berada dalam inflated
tracking reserve. Pada snapshot sebelumnya32.1 s, nearest overhead occupied
sekitar[-2.7,-.7,2.9]; pengamatan kemudian menambah occupied lebih dekat.
Ini replay end export, bukan exact live snapshot tiap tick. Penentuan gate
didukung oleh rekonstruksi tetapi snapshot kritis ditambah agar run berikut
tidak perlu inferensi dari state akhir. Tidak menghapus margin tracking.

Usulan engineering: transit_observation_guard=.2 m pada preset fixed inspection.
Approach/return route search dan kurva nominal menyisakan satu voxel ekstra
di atas badan; live suffix memeriksa guard itu juga agar leaf update memicu
brake/replan sebelum ruang nominal minimum habis. Backup runtime tetap diperiksa
dengan clearance/margin fisik lama; extra buffer merupakan cadangan lintasan,
bukan pelonggaran batas. Full PVA, braking, PID, sensor/map freshness tetap.
Buffer ini tidak memprediksi obstacle yang sama sekali belum teramati dan
bukan jaminan formal terhadap pertumbuhan peta arbitrer atau prior FOV salah.
Keberhasilan pengurangan masalah harus dibuktikan flight berikut.

_braking sekarang memisahkan gate actual_remaining_stop vs nominal_next_step,
menyimpan witness, polinomial stop, elapsed/duration, PVA dan exact grid pada
failure_critical.npz. Backup status valid=false/invalidated beserta map version
ketika tidak berlaku lagi; tidak meninggalkan valid=true yang basi.
Anggaran tetap8 snapshot biasa+1 kritis. Ini pencatatan; keputusan berhenti
atau menolak langkah yang tidak aman tidak dilonggarkan.

60 regresi terkait lulus sebelum tambahan tes. Dua fixture guard baru awalnya
gagal menemukan pasangan roof voxel karena z2.8 tepat pada batas voxel dan
Bezier hull bisa mencakup voxel sebelah; fixture dipindah2.85 m, tanpa mengubah
algoritma agar tes lolos. Dua regresi transit approach/return lalu lulus,
serta regresi occupied wall yang membuktikan actual stop dapat lolos ketika
nominal step gagal, dengan critical snapshot dan status invalidated benar.
Hasil suite lengkap dan flight baru dicatat setelah tersedia.

Suite setelah buffer lokal430 pass/104.43 s. Replay kemudian memperlihatkan
endpoint history lama tidak memenuhi guard lokal (endpoint_not_safe). Agar
tidak mengejar tujuan yang ditolak tahap berikut, plan_history_route memakai
guard transit yang sama; ContinuousTree._history_goal menyeleksi prefix pada
mask guarded tanpa menumpuk guard dua kali. Regresi global menempatkan leaf
yang tidak memblokir body route tetapi memblokir buffer; A* guarded mencari
detour.28 regresi history/controller lulus.

Replay snapshot32.1 s dengan history guarded: detour partial valid melalui
[-1.7,-.5,1.7]→[-4.3,-1.3,1.5]→[-4.9,-1.3,1.5]→[-5.1,-1.3,1.7].
Semua segmen memenuhi guarded mask; prefix lokal[-5.1,-1.3,1.7] feasible.
Partial bukan bukti tujuan inspeksi tercapai. Suite akhir diulang setelah
perubahan global; hasil final dan run berikut dicatat sesudah tersedia.

Suite final setelah penyamaan history/local:431 tes lulus,0 errors/failures/
skipped,100.045 s. Bukti persisten build_ros/polinasi_nav/pytest.xml, timestamp
2026-10-20T08:33:07.225523. Pemeriksaan pascainterupsi memastikan tidak ada
simulator atau test runner tersisa. Commit/push sebelum flight ketiga.


## 20 Oktober 2026 — flowers run3: auditor menghentikan preventive depth brake

Run d4f9e96 reports/fixed_lidar_validation_20261020_093818, registered session
.dependencies/ros_sensor_palm_farm_flowers_20261020_093822_5581_mapping,
berhenti33.00 s APPROACH tree2, failure/stop_reason depth_recovering. Session
stopped=true, audit port/proses kosong.236 pasangan pose: RMSE .01004115 m,
max .02359878 m, sensor health tracking. Belum menguji scan repair di flight.
Transit curve pada log nominal_safe=true, guard .2 m/guard_safe=true, detour
di ketinggian sekitar1.70 m. Tidak menyimpulkan buffer terbukti untuk seluruh
approach karena flight berhenti sebelum selesai.

depth_motion_recovery melaporkan certified_brake_started, started33.0,
integrated_age=.6000000000000014, valid_age=.6000000000000014, threshold .6,
hold_timeout2.0. last_integrated/last_valid32.4. Last received frame32.6;
last mapping job submitted32.58/completed32.74 mengintegrasikan depth32.4.
Perbedaan status depth_safety age .55 berasal dari assess tick sebelumnya;
recovery memiliki umur yang dipakai pada keputusan33.0 s. Pipeline latest
map received→setpoint1.7796 s wall/.30 s sim; tidak mengklaim dropout fisik
atau deadlock hanya karena ada jeda satu frame pada map yang terintegrasi.

Root cause1 terverifikasi: DepthMotionRecovery membandingkan age>timeout*.6
tanpa toleransi roundoff; 33.-32.4 > .6 meski secara desimal tepat sama.
fresh_depth yang dipakai guard lain sudah memiliki1e-9 s tolerance. Perbaikan
menggunakan tolerance sama pada preventive threshold; threshold tetap .6 s,
timeout tetap1 s, recovery tetap2 s. Regresi33.0 tidak memulai brake,
33.05 (umur .65) tetap memulai certified brake.

Root cause2 terverifikasi: auditor sensor RecoveryWatch hanya mengenali
route_changed sebagai bounded recovery; depth_recovering langsung dianggap
terminal. Generic inspection-loop auditor dan unit controller sebelumnya
sudah memperlakukan depth_recovering sebagai sementara. Cached backup pada
run3 valid/executed/live_rechecked, duration1.988413 s, endpoint
[-1.438138,-.365326,1.615669]. Auditor berhenti pada tick awal, sehingga run3
TIDAK membuktikan controller gagal atau berhasil menyelesaikan recovery.

Perbaikan auditor read-only: kenali depth_recovering hanya ketika state
BRAKE/HOLD_ABORT dan action certified_brake_started/braking_or_waiting,
metadata finite valid, valid frame masih fresh dan tidak ada health fault.
Deadline ditambatkan ke timestamp started milik controller, tidak ke saat
auditor pertama menerima pesan; maksimum tetap2 s sim, wall watchdog60 s,
dan maksimum3 recovery/20 s. Tidak membuka jalur resume atau mengirim kontrol;
controller sendiri tetap mensyaratkan map baru setelah recovery dimulai,
actual velocity<.04, reference<.01, brake selesai dan pose/body/reference aman.
Sensor loss, stale_depth_camera, emergency, timeout, metadata/state tidak
sesuai tetap stop. Event depth_brake_started/depth_recovery_ended dicatat.

Tes targeted pertama29 pass/2 import failures: saat file ROS adapters diuji
mandiri, import approach_viewer tidak mendapat tools di sys.path, meskipun
full suite sebelumnya mengisinya lewat modul lain. Fixture import mandiri
diperbaiki, tidak mengubah runtime CLI. Target akhir31 pass dalam .74 s.
Regresi audit mencakup resume, hold, limit2 s, data valid basi, emergency,
lokalisasi hilang dan metadata tidak lengkap. Build/suite final dan run4
dicatat setelah tersedia. Tidak memperbesar batas cloud/pose/map/clearance.

Build tiga paket berhasil;433 tes lulus dalam86.89 s,0 errors/failures/skipped.
Log /tmp/fixed_depth_recovery_build.log dan build_ros/polinasi_nav/pytest.xml.
Commit/push sebelum run4; belum mengklaim keberhasilan flight perubahan ini.


## 20 Oktober 2026 — flowers run4: cadangan vertikal tidak menangkap pertumbuhan lateral

Run09d471d reports/fixed_lidar_validation_20261020_210322, registered session
.dependencies/ros_sensor_palm_farm_flowers_20261020_210325_33599_mapping,
berhenti47.35 s BRAKE/APPROACH tree1, emergency_unavoidable:route_changed.
Session stopped=true, audit port/proses kosong.379 pasangan pose, RMSE
.0092684963 m, maksimum .01783208436 m. Tidak ada health fault atau bukti
tabrakan aktual. Belum selesai approach; belum validasi misi dua pohon.

Snapshot exact map387/observasi47.1: failure_critical.npz di folder map_3d/
planner_failures run tersebut, gate nominal_next_step, actual_remaining_stop
masih lulus. Reference[-4.195666365,-1.288848637,1.5], v[-.241480558,
-.036258991,0], next[-4.207672489,-1.290525615,1.499997555]. Voxel nominal
[-4.1,-1.3,1.5] terblokir occupied[-4.3,-.3,2.3]; badan aktual masih safe.
Label emergency_unavoidable berarti gate controller gagal, bukan pembuktian
bahwa setiap manuver fisik mustahil. Tidak mengubah gate untuk meloloskannya.

Penyebab terverifikasi lewat tiga snapshot: voxel occupied baru tersebut
masih FREE pada map261/32.2 s dan318/39.1 s; pada keduanya sudah ada occupied
[-4.3,-.1,2.3], hanya .2 m dari voxel baru. Pada map387 voxel baru OCCUPIED,
seen47.1. Ini perubahan bukti sensor, bukan bukti daun bergerak atau ICP drift.
Cadangan transit sebelumnya hanya mengecek perpindahan ke atas .2 m. Ia
tidak menahan pertumbuhan occupied lateral yang mengenai footprint nominal.
Tidak ada bukti cache melewatkan occupied yang sudah diketahui pada voxel
baru; tidak menyatakan kapan persis pertama terlihat di antara snapshot318
dan387 karena riwayat per-voxel setiap frame tidak direkam penuh.

Perbaikan rekayasa: tetap reserve vertikal .2 m, tambah reserve lateral .2 m
khusus APPROACH/RETURN. Planner history, pemilihan prefix lokal, A*, kurva
hasil smoothing/retiming dan live unexecuted-suffix memakai view yang sama.
Ekspansi lateral dibulatkan satu voxel per sumbu (box XY konservatif), upward
satu voxel; marker view mencegah ekspansi ganda. Occupancy, unknown, priors,
timestamp, actual-body mask, clearance, tracking_margin, speed, timeout,
emergency/final PID gates dan policy scan tidak dilonggarkan. Cadangan ini
tambahan engineering margin untuk voxel tetangga, bukan bukti formal
robustness/SUPER/CIRI atau jaminan untuk obstacle yang baru terlihat arbitrer.
Backup tetap di-check ulang; tidak diperbolehkan melewati occupied nominal.

Regresi mengambil koordinat dan ukuran drone run4: buffer vertikal saja
menerima chord sebelum hit baru, buffer lateral menolaknya dengan hit lama.
Sesudah hit baru, chord nominal lama gagal seperti flight. Tes tambahan
memastikan retiming mempertahankan class/buffer, cache dicabut ketika muncul
unknown, view tidak mengubah raw/state/ages/actual-mask, penggunaan berulang
tidak menggandakan reserve, dan final trajectory memakai buffer pada kedua
fase transit.32 tes targeted lulus3.69 s. Full build/suite dan run5 dicatat
setelah tersedia. Belum mengklaim buffer ini berhasil dalam flight.

Build tiga paket berhasil;437 tes lulus98.26 s,0 errors/failures/skipped.
Bukti build_ros/polinasi_nav/pytest.xml dan /tmp/fixed_lateral_build.log.
Replay exact map261/32.2 s: history dengan reserve baru memilih detour
[-3.9,-1.5,1.5] ke reachable prefix[-4.9,-1.5,1.5],117 ekspansi;
seluruh edge lulus view lokal yang sama. Goal lama kini tidak aman pada
reserve baru, sehingga prefix partial tidak diklaim mencapai final goal.
Commit/push sebelum run5; hasil flight dicatat setelah tersedia.


## 20 Oktober 2026 — flowers run5: scan lengkap, endpoint belum siap transit

Runb6b1f74 reports/fixed_lidar_validation_20261020_211925, session
.dependencies/ros_sensor_palm_farm_flowers_20261020_211927_36980_mapping,
berhenti197.75 s BRAKE/APPROACH tree5, global_route_unavailable. Auditor
sudah meminta cleanup ketika sesi ini mengirim stop terdaftar; tidak ada
penghentian proses luar. Session stopped=true.1885 pasangan pose, RMSE
.00889611555 m, maksimum .02502530813 m, sensor estimator tracking. GT
hanya pembanding audit. Tidak ada bukti benturan atau kegagalan lokalisasi.

Takeoff tercatat9.15, approach20.70→87.90 (67.20 s simulasi), lalu ascent
dan scan. Pohon1 centre[-8.490262631,-.430426432], enam sektor front-camera
valid pada scan stamps106.1,121.8,138.9,153.8,166.2,181.5. Satu route_changed
pada98.801 akibat occupied[-9.1,.3,2.5] membatalkan suffix yang belum dilalui;
current nominal/actual masih aman. Certified brake/replan pulih99.30.
Beberapa kandidat scan ditolak braking checker lalu lolos retiming yang
diperiksa ulang. Belum membuktikan moving handoff berhasil: accepted0,
rejected31. Tidak mengklaim optimasi kecepatan selesai.

Pohon berikutnya id5 centre[-4.614801027,-10.238346938], berbeda dari pohon1.
Approach goal[-5.001648248,-8.274045490,2.0]. Setelah kamera siap185.4,
planner berulang mengembalikan history_start_not_safe hingga timeout
global_route_unavailable197.75. Ini bukan menunggu tanpa batas: gate wait
tetap menghentikan. Scan lama berakhir reference[-6.501186126,-.285123500,
2.1] yang lolos raw/scan reserve, tetapi gagal reserve lateral transit baru.
Perbedaan syarat endpoint dua fase belum diperiksa saat membentuk scan.

Replay ekspor map197.761/observasi197.4 (bukan snapshot live persis197.75)
menunjukkan occupied[-5.3,-.3,3.1] memblokir nominal tetangga
[-6.3,-.3,2.3]. Terminal reference aman nominal tetapi reserve transit
memeriksa tetangga tersebut. Menurunkan endpoint pada XY sama ke1.9 m
lulus reserve;2.0 masih satu voxel lama dan tidak cukup. Ini validasi
berdasarkan peta historis akhir, bukan bukti penerbangan atau ground truth
yang diumpankan ke planner.

Perbaikan: endpoint anchor scan TERAKHIR dan terminal braking-nya harus
memenuhi reserve transit yang sama sebelum diterima, baik ketika batching,
memilih target atau mereparasi final approach direction. Jika belum aman,
tetap sektor/sudut/radius yang sama atau outward sesuai batas lama; tidak
menghapus anchor. Untuk anchor terakhir saja, floor turun memakai floor
transit/exploration1.0 m di dalam batas scan_target_max_descent=.6 m dari
nominal. Sebelumnya floor minimal takeoff2.0 m menolak alternatif1.9.
Ini pilihan geometris departure di bawah daun; clearance terhadap tanah,
batang, daun, unknown, mask nominal/actual dan batas dinamika tetap sama.
Semua kandidat tetap memerlukan A*, smoothing dan whole-curve braking
certificate. Di replay diperlukan penurunan .2 m, bukan penurunan sampai
floor1.0. Pengamatan baru setelah arrival dapat membatalkan departure
reserve; sector terakhir belum dihitung dan target direncanakan ulang.

Tambahan audit: global_route_unavailable kini menyimpan penyebab history
terkini dan exact critical snapshot, bukan menampilkan diagnostic kandidat
scan lama yang sudah recovered. Timeout tidak berubah. Eksplorasi tidak
diteruskan setelah failure pada tick yang sama.

40 tes targeted lulus4.59 s. Regresi: endpoint scan lama raw-safe namun
transit-unsafe diperbaiki turun .2 m pada XY yang sama; obstacle terukur di
bawah menolak repair, sector tidak diskip, observasi baru membatalkan
penghitungan sektor terakhir, timeout history mencatat cause/snapshot baru.
Replay final map: shared MissionController._prepare_trajectory menghasilkan
kurva aman2.793402373 s ke endpoint1.9 m, seluruh braking checks lulus,
reserve transit endpoint lulus. Fixture replay awal belum mengisi command
dan observed_stem_top sehingga gagal setup; setelah mengisi keduanya sesuai
state misi replay lengkap lulus. Tidak ada perubahan produksi untuk menutupi
kesalahan fixture. Build/regresi final dan run6 dicatat setelah tersedia.

Build tiga paket berhasil;441 tes lulus93.11 s,0 errors/failures/skipped.
Bukti build_ros/polinasi_nav/pytest.xml dan /tmp/fixed_exit_build.log.
Replay history akhir dari endpoint1.9 m juga menghasilkan reachable partial
detour menuju pohon5 lewat[-5.1,-2.3,1.3] ke[-3.5,-6.7,1.3],431 ekspansi;
tidak lagi history_start_not_safe. Prefix masih partial; tujuan final tetap
memerlukan observasi baru, bukan klaim seluruh approach sudah aman.
Commit/push sebelum run6; hasil flight belum tersedia.


## 20 Oktober 2026 — flowers run6: cadangan langsung dan koreksi PID

Run5cad7d0 reports/fixed_lidar_validation_20261020_215038, registered session
.dependencies/ros_sensor_palm_farm_flowers_20261020_215040_41781_mapping,
berhenti249.40 s BRAKE/SCAN tree1 anchor21, pid_final_safety,5 sektor valid.
Auditor sudah stop; session stopped=true, host port/proses kosong.2401
pasangan pose, RMSE .00980471701 m, maksimum .02186948878 m, estimator
tracking. Tidak ada bukti collision aktual. Perbaikan endpoint terminal
scan5cad7d0 belum diuji flight karena belum mencapai anchor terakhir.

Takeoff tercatat9.00, approach20.50→63.40 (42.90 s), SCAN66.65. Variasi
approach dari run5 bukan bukti pengaruh perbaikan terminal scan: perbaikan
tersebut baru aktif pada anchor terakhir. Recovery route_changed karena
suffix keluar rolling window terjadi87 s dan220 s; berhasil kembali scan.
Satu moving C2 handoff diterima/live-map-checked; ini bukti acceptance satu
sambungan saja. Retiming sempat memperpanjang segmen ke99.1331 s, kurva
total41.1337→139.5512 s setelah10 retime. Reference .0027 m/s: bukan deadlock
atau dropout. Masih masalah efisiensi yang perlu diaudit; tidak menaikkan
batas kecepatan agar menutupi kegagalan ini. Run dilanjutkan sampai fault
terminal karena validasi misi belum selesai dan speed improvement ditunda.

Critical snapshot map2038/observasi249.2 menyimpan PID dan guarded grid
secara persis. Reference p[-7.071925142,-1.869292164,2.273857812],
v[.172836106,.189469053,.082638065], actual p[-7.134802158,-1.916098371,
2.266614501], actual v[.158738067,.190654317,.119869457]. Position error
sekitar8 cm. Semua3 correction scales1,.5,0 lulus brake_safe actual,
reference dan velocity-uncertainty di raw nominal grid. Ketiganya gagal
explicit_backup pada grid dengan reserve canopy .2. Witness occupied
[-6.7,-2.9,3.3], inflated cell[-6.9,-1.9,2.3]. Voxel ini sudah OCCUPIED
pada exact snapshot map1995/244.0 (seen243.9), tetap occupied pada critical
(seen246.3). Jadi tidak boleh menyebut ini obstacle baru249.4 atau drift.

Gap terverifikasi: planner check_braking mengecek stop SETELAH reaction
segment dan robust along-motion scenario; runtime juga meminta immediate
PVA backup dengan acceleration hasil PID/jerk limiter. Kedua tes bukan
sertifikat yang identik: raw grid vs reserve dan awal/percepatan brake
berbeda. Pada nominal PVA snapshot immediate backup masih ada, tetapi
acceleration aktual yang diusulkan PID membuat base backup gagal. Zero
feedback pun masih berbeda dari nominal karena jerk limiter mempertahankan
kontinuitas acceleration sebelumnya. Tidak mengklaim nominal checker
membuktikan semua arah galat PID; skenario feedback terbatas bukan TEB formal.

Penelitian primer: kode resmi SUPER menggunakan BackupTrajOpt, kendala
posisi/kecepatan/percepatan/jerk serta variabel temporal/geometris:
https://github.com/hku-mars/SUPER/blob/master/super_planner/src/traj_opt/backup_traj_optimizer_s4.cpp
FASTER mempertahankan safe backup dalam known-free:
https://arxiv.org/abs/2001.04420 . Implementasi kita tetap quintic finite
candidate search, bukan MINCO/CIRI atau jaminan safety SUPER/FASTER. Tidak
menyalin kode sumber paper untuk perubahan ini.

Perbaikan: fixed inspection mengaktifkan explicit_backup_fallback_search.
Base backup tetap jalan cepat. HANYA bila gagal, gunakan pencarian existing
yang sebelumnya offline:20 kandidat tambahan (4 duration scales x5 endpoint
offsets), maksimum21 pada pencarian, preserve initial PVA, final v/a nol.
Semua kandidat diperiksa collision pada reserve yang sama serta derivative
hulls v/a/jerk. Tidak menghilangkan occupied/unknown atau reserve. Catat
candidate index/count, search mode, derivative bounds, alasan base reject,
dan fallback wall time. Failure tetap terminal bila tidak ada kandidat.
Planner kini juga menguji immediate backup pada setiap PVA sample kurva,
selain reaction stop dan skenario feedback. Signature cache memasukkan
opsi explicit backup agar hasil tidak dipakai lintas kebijakan.

Replay exact PID setelah raw-grid rebuild: hasil canopy mask byte-identical
dengan critical snapshot. Koreksi1 masih gagal, correction .5 menemukan
candidate6 (duration scale .85, zero offset), duration1.528422919 s;
batas v=.2707330474,a=.2727460428,jerk=.7629929326. PID tanpa failure,
cadangan live-safe. Satu pengukuran callback replay47.03 ms, fallback terakhir
15.60 ms; ini bukan pengukuran timing flight atau bukti Jetson real-time.
Tidak mengaktifkan suite tes ketika flight berjalan.

Tes existing awal42 pass31.90 s; perintah pertama salah menyebut test_pid.py
yang tidak ada, diperbaiki ke test_pid_control.py. Regresi baru meniru
occupied/PVA run6, menguji fallback bounded/PVA/terminal, PID lama gagal dan
baru menerima cadangan valid, abort memakai cached optimized backup yang
di-recheck, unknown baru mencabut cache dan all-unknown tetap ditolak.
Probe PVA lain pada obstacle sama membuktikan delayed+robust stop bisa
lulus ketika immediate stop gagal; planner baru menangkap perbedaannya.
Hasil targeted/final/build/run7 dicatat setelah tersedia.

Tes targeted backup39 pass34.25 s. Build tiga paket berhasil;446 tes lulus
91.80 s,0 errors/failures/skipped. Bukti build_ros/polinasi_nav/pytest.xml
dan /tmp/fixed_backup_build.log. Commit/push sebelum run7; belum validasi
flight fallback backup atau terminal scan-to-transit.


## 20 Oktober 2026 — flowers run7: uncertainty check terlambat sesudah referensi maju

Run18af5b1 reports/fixed_lidar_validation_20261020_223010, registered session
.dependencies/ros_sensor_palm_farm_flowers_20261020_223013_47099_mapping,
berhenti242.75 s BRAKE/SCAN tree1 anchor8, pid_final_safety,2 sektor valid.
Auditor sudah melakukan cleanup.2340 pasangan pose, RMSE .00984922940 m,
maksimum .02053082270 m, health tracking. Tidak ada bukti collision aktual.
Perbaikan terminal scan masih belum tervalidasi flight. Satu map-change
brake pada sekitar74 s pulih74.30; belum menyelesaikan orbit pertama.

Ada masalah efisiensi tambahan: curve167.113621 s dengan segmen144.834987 s
dan reference sekitar.0027 m/s. Kali ini before duration optimisation sudah
175.763196 s, local_braking_retiming kosong: durasi besar berasal dari
profil kecepatan/initial quintic timing, bukan loop local retime seperti
run6. Drone tidak deadlock; elapsed naik sampai149.05/167.11 pada226.60.
Run diteruskan untuk validasi misi hingga fatal fault. Ini bukan hasil
minimum-time; masalah performa tetap terbuka.

Critical map2091/observasi242.4: candidate p[-10.022164495,1.084411216,
2.443061003], v[-.043868936,-.210380881,-.057385889], nominal a
[.043047612,-.004048601,-.002165949]. Actual p[-10.015101055,1.101671384,
2.455711299], v[-.050583380,-.226518227,-.060643041], tracking error .03312 m.
PID correction scales1,.5,0 semua lulus actual/reference brake checks;
velocity uncertainty+.03 m/s semuanya gagal occupied[-9.3,.1,2.9],
inflated cell[-10.1,.7,2.3]. Pada critical snapshot voxel occupied tersebut
seen202.9: sudah tercatat sekurangnya39.85 s sebelum fault, bukan obstacle
baru pada tick242.75. Tidak menyimpulkan drift/dropout. Backup fallback
18af5b1 belum dipanggil pada tick gagal karena gate uncertainty lebih dulu
menolak kandidat. Ini kasus berbeda dari run6, bukan bukti pencarian21
kandidat gagal menemukan solusi pada run7.

Root cause terverifikasi: live geometry recheck dan directional/final
nominal braking gate tidak mengecek uncertainty scenario sebelum self.command
dimajukan. PositionPID baru memeriksanya sesudah PVA referensi diganti.
Peta dapat tetap geometrically-safe untuk kurva sementara stopping envelope
tidak lagi aman. Sertifikat worker sebelumnya pada frozen map tidak
menggantikan pemeriksaan safety calon referensi terhadap peta live.
Pada exact current grid run7, skenario maximum forward PID juga gagal;
jadi tidak mengklaim kegagalan run7 semata-mata karena skenario zero PID
yang belum diuji planner. Gap zero PID ditemukan pada audit kontrak:
nominal(no uncertainty) plus uncertainty+maximum-PID bukan sertifikat
untuk uncertainty tanpa tambahan percepatan; geometri stop tidak monoton
terhadap acceleration. Tetap hanya finite scenarios, bukan formal robust
coverage untuk semua vektor PID/estimator error.

Perbaikan1: feedback_brake_safe mengecek uncertainty-only dan, jika ada,
uncertainty+forward maximum PID, dengan feedback_scenario dan daftar checked
scenarios pada log. PID yang memanggil correction_acceleration=0 tidak
menduplikasi tes. Planner/profile/terminal dan runtime memakai helper sama.
Perbaikan2: sebelum mengubah command PVA, final control gate mengecek calon
referensi dengan helper pada map live. Bila ditolak, simpan critical snapshot
reference_feedback_reserve_revoked, kandidat dan PVA sebelumnya, lalu gunakan
bounded route_changed brake/replan existing. PVA/backup terakhir yang benar
benar dikirim tidak ditimpa kandidat yang gagal. Tidak menghapus pemeriksa
PID, menurunkan uncertainty, memperbesar timeout atau melonggarkan clearance.
Health dan actual-body checks tetap mendahului keputusan ini.

Replay critical: raw rebuild lalu upward reserve byte-identical dengan
mask snapshot. PVA sebelumnya di242.70:
p[-10.019917069,1.094919972,2.445926133],
v[-.046031309,-.209865041,-.057190280],
a[.029641446,.008620295,-.016603392]. Cached explicit stop pada map live
masih valid. Calon nominal p242.75 lulus brake_safe raw, tetapi uncertainty
gagal. Dengan gate baru MissionController.tick menghasilkan route_changed,
state BRAKE, cached backup executed=true, bukan mem-publish kandidat dan
kemudian pid_final_safety. Ini replay satu tick, bukan validasi seluruh
flight. Snapshot/PVA sample dipakai sebagai input regresi kontrol; ground
truth tetap tidak dipakai sebagai state estimator/planner.

64 tes gabungan existing backup+continuous tree lulus40.95 s. Regresi baru
menguji dua skenario feedback/tanpa duplikasi saat extra=0 dan sample PVA
run7: nominal-safe/uncertainty-unsafe, cadangan terakhir dieksekusi, lalu
brake selesai kembali SURVEY tanpa menerbitkan candidate yang ditolak.
Hasil targeted/final build dan run8 dicatat setelah tersedia.

Tes targeted41 pass42.26 s. Build tiga paket berhasil;448 tes lulus108.83 s,
0 errors/failures/skipped. Bukti build_ros/polinasi_nav/pytest.xml dan
/tmp/fixed_reference_build.log. Commit/push sebelum run8; belum validasi
flight pre-commit recovery atau penyelesaian dua pohon di flowers map.


## 22 Oktober 2026 — audit run8 dan perbaikan sambungan endpoint

Baseline tetap ce3bb66 pada worktree /home/abin/polinasi_lidar_fixed;
rotating-lidar6779baf dan perubahan lokal pengguna di workspace asal tidak
diubah. User menghentikan simulasi, lalu meminta masuk loop kembali.
Tidak ada AGENTS.md yang berlaku di ancestor/worktree. Runtime host audit
tidak menemukan Gazebo/SITL/RViz atau port9002/14650/5762 aktif sebelum tes.

Run8 reports/fixed_lidar_validation_20261020_230814, registered session
.dependencies/ros_sensor_palm_farm_flowers_20261020_230817_52090_mapping,
berhenti85.20 s BRAKE/APPROACH tree1 global_route_unavailable.761 pasangan
pose, RMSE .00774778236 m, maksimum .01944069591 m, tanpa health fault.
Sebelumnya map-change brake37.85 pulih39.65; diagnosis menunjukkan revokasi
geometry suffix, bukan bukti flight gate feedback baru sudah terpicu.
Session stopped=true; stop pengguna sesudahnya idempotent tanpa signal luar.
Belum selesai approach, belum inspeksi dua pohon pada map bunga.

History request32/stamp83.733/map83.4 mengembalikan endpoint_grid_connection,
path kosong. Start[-6.3125,-.65,1.75], target approach
[-6.486624940,-.326961701,2.0]. Wait_since73.15; timeout sekitar12 s
berakhir85.20. Current nominal safe=true dan actual speed sekitar.0063 m/s.
Ini berbeda dari history_start_not_safe pada transisi run5 dan PID fault
run6/7. Tidak menyebutnya collision, GPS loss atau ICP failure.

Penyebab kode: goal dianggap full bila single-cell grid.safe(goal) dan
komponen grid sama, tanpa memeriksa sambungannya ke centre. A* kemudian
memeriksa supercover connector dan menolak endpoint_grid_connection.
Goal2.0 berada di muka voxel: pada search origin yang tercatat,
floor memilih centre1.9000000000000004 yang bebas, tetapi connector juga
menyentuh tetangga centre2.1 yang terblokir setelah reserve. Grid lokal
dengan representasi origin sedikit berbeda memilih upper cell dan menolak
point itu juga. Tidak memperbaiki dengan mengganti label frame atau
menghapus supercover; kedua sisi batas tetap harus diperiksa.

Replay memakai critical snapshot map708/observasi84.9 (bukan exact frozen
history map83.4). Raw grid rebuild byte-identical dengan mask snapshot.
Rekonstruksi search region yang sama dari data snapshot menghasilkan
endpoint_grid_connection yang sama; start connector aman, goal single-cell
aman tetapi goal connector gagal pada upper cell2.1. Kontrol all-free pada
origin/shape sama berhasil sebelum maupun sesudah ceiling cap: ini bukan
semata-mata roundoff atau ceiling cap tanpa obstacle/reserve.

Perbaikan di global_route.py: full destination hanya diterima jika point
dan connector ke centre lulus aturan supercover yang sama dengan A*.
Jika tidak, tetap simpan tujuan asli dan cari reachable free-centre terdekat
sebagai partial route. Tambahkan goal_connection diagnostic (point/connector
dan blockage). Shortcut ketika jarak destination<.12 kini juga harus
lulus line_safe(current,destination); dekat tidak otomatis boleh menembus
tetangga blocked. Occupied/unknown, FOV prior, clearance, reserve, timeout,
batas dinamika, altitude policy dan mounting tidak diubah.

Replay setelah perbaikan menghasilkan direct partial prefix ke
[-6.5,-.3,1.9], final_goal_reached_by_route=false. Tujuan asli tetap2.0 m.
Shared MissionController._prepare_trajectory dengan controller fixed transit
menghasilkan kurva3.599476935 s, seluruh smoothing/whole-curve braking
checks lulus, failure kosong. Prefix berjarak sekitar.10 m dari view target;
tidak mengubah toleransi arrival yang sudah ada. Ini replay planner pada
map akhir, bukan penerbangan atau pembaruan timestamp sensor. Prior ages
tidak tersedia di snapshot geometry dan sengaja tidak dianggap fresh.

Regresi tambahan menggunakan ukuran drone/reserve/koordinat run8 dengan
occupied terukur: point-safe/connector-unsafe menghasilkan partial prefix,
tidak mengubah original goal atau occupied. Kasus near-goal .05 m tetap
ditolak jika supercover menyentuh roof meskipun single-cell point-safe.
12 tes targeted lulus2.12 s. Build/regresi penuh dan run9 dicatat sesudah
tersedia; belum mengklaim flight fix endpoint atau misi bunga sukses.

Build tiga paket berhasil;450 tes lulus96.94 s,0 errors/failures/skipped.
Bukti build_ros/polinasi_nav/pytest.xml dan /tmp/fixed_endpoint_build.log.
Commit/push sebelum run9; validasi flight perubahan endpoint masih belum
tersedia dan keberhasilan misi dua pohon pada map bunga belum tercapai.


### 22 Oktober 2026 — run9 dan preferensi jarak lunak pada transit

Run9 commit42691db: reports/fixed_lidar_validation_20261022_071758,
registered session .dependencies/ros_sensor_palm_farm_flowers_20261022_071801_12657_mapping.
Run dihentikan oleh auditor pada64.55 s simulasi, BRAKE/APPROACH tree2,
global_route_unavailable/history_start_not_safe; belum selesai approach.
554 pasangan pose audit: RMSE .0098459693 m, maksimum .0226603413 m.
Tidak ada bukti fault lokalisasi sebagai penyebab. Session stopped=true.

Pada49.3 s map407, live suffix tidak lagi memenuhi reserve atas/lateral,
sementara pemeriksaan badan nominal dan aktual masih aman. Brake pulih
sekitar50.85 s; planner menunggu rute dari52.501 hingga64.55 s. History
request22/map62.6 menolak start[-4.30993860,-1.44565333,1.49892530],
goal[-6.51486298,-.33515512,2]. Critical snapshot map538/observasi64.3
tersimpan di reports/ros_sensor_palm_farm_flowers_20261022_071801_12657_mapping/map_3d/planner_failures/failure_critical.npz.
Rebuild raw mask identik: posisi berhenti aman untuk mask nominal, tetapi
terblokir pada transit reserve. Ada occupied[-4.3,-.3,2.3] yang mempengaruhi
mask tetangga. Snapshot akhir tidak membuktikan kapan voxel itu pertama
terlihat dan tidak cukup untuk menyimpulkan drift. Tidak menambahkan escape
yang membolehkan lintasan menerobos reserve dari start yang sudah blocked.

Perbaikan prospektif: A* transit diberi biaya jarak dari mask yang SUDAH
di-inflate. Preset transit_clearance_preference=.4 m dan
transit_clearance_cost_weight=2; biaya 1+2*max(0,1-distance/.4)^2.
Ini preferensi lunak, bukan perubahan hard clearance atau jaminan aman
terhadap seluruh pembaruan map. Unknown, measured occupied, user FOV prior,
health, timeout, kecepatan, reserve .2 m atas/lateral tetap sama. Hanya view
transit mengaktifkan biaya; scan memakai kebijakan geometrinya sendiri.
A* memakai heuristic Euclidean yang tetap admissible karena biaya>=1.
Shortcut tidak boleh menaikkan biaya lintasan yang digantikannya. Kandidat
chord langsung yang sudah line-safe tetap dapat dipakai jika lebih murah
atau pencarian preferensi habis anggaran. Chord tetap menjalani validasi
kurva dan pengereman; bila chord blocked, batas pencarian tidak dilonggarkan.

Rujukan prinsip cost-aware A*: dokumentasi primer Nav2 Humble Smac,
https://api.nav2.org/nav2-humble/html/md_nav2_smac_planner_README.html
(cost_travel_multiplier memberi preferensi menjauhi area berbiaya tinggi,
dengan tambahan beban pencarian). Implementasi ini biaya 3D rekayasa sendiri,
bukan port Nav2, CIRI, MINCO atau SUPER penuh.

Replay dari posisi lebih awal[-2.8,-1,1.5] ke[-6.5,-.3,1.9] pada snapshot
AKHIR run9, bukan exact map49.3 dan bukan recovery dari start gagal:
tanpa biaya, rute melewati[-3.3,-1.3,1.1],[-5.5,-1.3,1.1]; dengan biaya,
rute mengambil sisi[-3.7,-1.7,1.3],[-5.1,-1.7,1.3]. Keduanya lulus full
controller/curve/backup checks. Durasi kurva39.8219034 menjadi19.1280823 s;
A*177 menjadi411 expansions dan74.42 menjadi197.67 ms. Full preparation
2890.45 menjadi1601.56 ms pada satu replay; bukan benchmark atau hasil
flight. History dimatikan hanya pada harness replay untuk mengisolasi local
planner; tidak pada preset flight. Tidak memasukkan ground truth ke planner.

Lima regresi baru: preferensi menjauhi dinding tanpa mengubah mask/state;
shortcut mempertahankan biaya; chord pendek dan batas pencarian tetap;
unknown wall tetap blocked; opt-in hanya transit; pertumbuhan satu voxel
memblokir chord lama namun preferred polyline dan smoothed curve tetap aman.
61 tes targeted lulus24.44 s. Build/regresi penuh dan flight berikutnya
akan dicatat setelah selesai. Belum ada keberhasilan dua pohon pada map bunga.

Build tiga paket berhasil;455 tes lulus96.99 s,0 errors/failures/skipped.
Bukti build_ros/polinasi_nav/pytest.xml; log /tmp/fixed_cost_build.log.
Audit host sebelum flight berikutnya: port9002/14650/5762 bebas dan tidak
ada proses Gazebo/SITL/RViz. Perubahan akan di-commit/push sebelum run10.


### 22 Oktober 2026 — run10, guard cadangan transit dan bukti sebelum/sesudah

Run10 b419a17 (sudah push sebelum flight), reports/fixed_lidar_validation_20261022_073940,
registered session .dependencies/ros_sensor_palm_farm_flowers_20261022_073943_16004_mapping.
Auditor menghentikan pada64.60 s simulasi/global_route_unavailable,
BRAKE/APPROACH tree1.554 pasangan pose: RMSE .00892677842 m,
maksimum .0180463821 m; tidak ada fault kesehatan lokalisasi tercatat.
Session stopped=true, tidak ada simulator sesi lain yang dihentikan.
Belum menyelesaikan approach. Preferensi lunak belum terbukti mengatasi
kegagalan ini dan tidak disebut perbaikan flight sukses.

Replan1 diterima27.1 s: curve14.3974147 s, target prefix[-3.396,-1.5,1.5].
Replan2 diterima44.5 s: prefix[-6.5,-.3,1.9]. Sekitar48.5 s map396,
live suffix gagal pada[-4.308048,-1.5,1.413519] (nominal/actual body aman),
route_changed pulih50.65 s. Posisi berhenti[-4.674690,-1.500247,1.376709]
nominal-safe tetapi history request22/map62.4 menolak history_start_not_safe.
Critical snapshot berada di reports/ros_sensor_palm_farm_flowers_20261022_073943_16004_mapping/map_3d/planner_failures/failure_critical.npz.

Replay peta akhir dari[-3.4,-1.5,1.5] ke[-6.5,-.3,1.9]: menaikkan
preferensi/bobot dari(.4,2) ke(.8,2),(.8,4),(1.2,4) tidak mengubah rute
terpilih; expansions268/630/1149/1381. Karena tidak memberi bukti perbaikan,
parameter tersebut TIDAK dinaikkan. Peta sebelum revokasi tidak tersimpan;
seen adalah timestamp pengamatan terbaru, bukan first-seen. Snapshot akhir
saja tidak cukup untuk mengklaim first-seen occupied atau mekanisme kausal
lengkap. Tidak mengasumsikan lingkungan diam karena peta sebelumnya sama.

Audit menemukan inkonsistensi nyata: ContinuousTreeInspectionController
membungkus kurva transit dengan reserve atas/XY, tetapi super_policy.backup_grid
hanya menambahkan reserve pada SCAN/ASCEND. Akibatnya final feedback check,
explicit backup kandidat dan whole-curve braking check transit masih dapat
menggunakan raw mask. Perbaikan: backup_grid memberi transit_guard_grid yang
sama pada SURVEY/APPROACH dan RETURN; saat BRAKE, resume_state menjaga fase
asal agar cached backup tidak tiba-tiba kehilangan reserve. Semua candidate
masih harus lulus hard guard. Emergency fallback Brake tetap memeriksa
actual/nominal body stopping seperti semula; ketika observasi sudah mencabut
reserve, fallback itu tidak diklaim menjaga reserve tambahan atau membolehkan
melanjutkan navigasi. Tidak ada escape dari start blocked yang ditambahkan.

Tambahan bukti audit bounded: evidence_accepted.npz menyimpan exact grid,
curve controls/durations saat kandidat lolos pada detached planning worker;
evidence_revoked.npz menyimpan raw grid/kurva sebelum fail(route_changed)
menghapus kurva. Dua slot ditimpa pada event berikutnya, terpisah dari maksimal
8 ordinary failure dan failure_critical.npz. Metadata mencakup guard cells
atas/lateral, grid_config, replans, elapsed, map stamp/version dan waktu wall
monotonic. Ini bukan ground truth dan tidak memasukkan data audit ke estimator.
Penulisan evidence revokasi dilakukan setelah memilih brake, sebelum kurva
lama dihapus; overhead snapshot perlu dipantau pada run berikutnya.

Regresi baru membuktikan point raw-safe dapat gagal reserve dan raw feedback
stop lulus sementara guarded stop ditolak; APPROACH/RETURN/BRAKE memakai mask
reserve yang sama dan TAKEOFF tidak berubah. Regresi evidence membuktikan
peta/kurva sebelum dan sesudah berbeda tetap tersedia, independent dari
critical snapshot serta tetap bounded. Hasil build/tes dan flight dicatat
setelah tersedia; perubahan belum flight validated.

Build tiga paket berhasil;457 tes lulus110.88 s,0 errors/failures/skipped.
Bukti build_ros/polinasi_nav/pytest.xml dan /tmp/fixed_backup_guard_build.log.
Tes targeted awal67 lulus/1 gagal karena ekspektasi guard satu voxel pada
fixture resolusi .1 m; diperbaiki memakai ceil(.2/res-1e-9), bukan mengubah
implementasi pembulatan atau ukuran reserve. Full suite setelahnya lulus.
Commit/push dilakukan sebelum run11; flight belum tervalidasi.


### 22 Oktober 2026 — run11 mencapai transisi pohon, entry kedua blocked

Flight b0ba099, reports/fixed_lidar_validation_20261022_075552,
registered session .dependencies/ros_sensor_palm_farm_flowers_20261022_075555_19872_mapping.
Sensor-mode: ground truth hanya auditor, estimator eksperimental scan-to-map
ICP+IMU, bukan FAST-LIO. Auditor menghentikan run pada320.90 s simulasi,
blocked_route, BRAKE/APPROACH tree3. Session stopped=true.3115 pasangan
pose: RMSE .01129093568 m, maksimum .0247004473 m. Tidak ada health fault
lokalisasi tercatat. Tidak ada GUI/RViz atau hardware yang dioperasikan.

Milestone dari navigation JSONL: APPROACH tree1 mulai20.50 s dan ASCEND
mulai73.85 s, durasi approach53.35 s SIMULASI. SCAN mulai77.10 s, enam
sektor valid selesai259.95 s; APPROACH tree3 mulai260.00 s. Pusat tree1
[-8.49279660,-.42920296], tree3[-4.59039740,-10.24937155]. Enam view
first-tree memiliki camera_policy continuous_front_tree dan inspeksi valid.
Anchor terakhir direpair dari2.1 ke1.9 m pada255.3 s dengan reserve transit,
kemudian berhasil berangkat ke pohon lain. Ini flight pertama loop terkini
yang membuktikan exit-anchor/transisi bekerja, belum misi bunga dua pohon.
Belum RETURN, landing atau disarm.

Recovery bounded: route_changed pada82.2→83.8,91.3→92.25 dan
164.45→164.85 s. Yang terakhir reference_feedback_reserve_revoked,
curve suffix sendiri masih aman: finite feedback-stop scenario menolak
calon referensi, command belum diganti, backup terakhir dieksekusi.
Snapshot accepted/revoked/critical tersedia. Depth recovery kedua pohon
tercatat268.25→270.00,280.25→282.20,300.85→301.90 s; gate .1 s/2 s
recovery tetap, tanpa pelonggaran timeout.

Masalah efisiensi belum selesai: kurva scan sekitar125.05 s memiliki total
142.9415768 s; segment2 awal local speed profile .021 m/s (dibatasi karena
feedback braking_box_blocked), duration optimization118.8364514 s lalu dua
local retimes memberi segmen2 78.82106635 s dan total142.94 s. Replan setelah
recovery164.85 menghasilkan kurva56.9562263 s. Ini bottleneck geometri/
pengereman, bukan bukti menaikkan maximum speed akan menyelesaikannya.
Tidak menjalankan suite berat atau mengubah parameter saat flight.

Kegagalan akhir berbeda: tujuan entry tree3
[-4.99684253,-8.29242717,2.0] repeatedly newly_observed_occupied_view.
route_blockage occupied_blocked/goal_not_safe, unknown_count0,
occupied_count57;80 scan baru tanpa kandidat pengganti sampai deadline.
Obstacle sampel sekitar[-5.5,-7.3,2.1..2.7],[-5.7,-7.5,2.5..2.9], dekat
jalur masuk pohon kedua. Tanpa semantic ID tidak dapat membuktikan daun
milik pohon mana, walau lebih dekat ke tree3 daripada tree1.

Penyebab kode terbukti: preset tree_view_distances=[2.] dan entry tetap pada
satu tree_angle/height2.0. Ketika target occupied, ia ditolak, views kosong,
lalu fallback membentuk koordinat occupied yang sama. Rejected log berulang
identik. Ini bukan izin menghapus obstacle atau menambah toleransi goal.
Run11 critical snapshot terakhir masih164.1 s karena blocked_route bukan
critical reason dan8 ordinary slots habis. Map akhir320.7 s lengkap tetap
tersimpan di map_3d/map.json (occupied/free/prior/observation timestamp),
point cloud dan map.html. Bukti akhir direkonstruksi dari export sensor itu,
bukan dari geometri/ground truth world; tidak sama dengan exact frozen
navigation window saat timeout.

Perbaikan entry (preset fixed inspection opt-in):
- fixed_approach_target_repair=True. Pencarian bounded24 arah di sekitar
  pusat pohon terukur, hanya radius konfigurasi2 m; tinggi target asli2 m
  serta voxel centres di bawahnya sampai exploration_min_altitude1 m.
  Target tidak dibulatkan naik melewati cap. Candidate harus point-safe,
  supercover-connectable ke centre dan berada pada komponen free yang sama
  dengan start dalam transit guard. Ini destination proposal, bukan flight
  certificate; history A*, local curve, feedback stop dan live checks tetap.
- Jika entry occupied terukur, sudah memiliki blocked_since, tidak ada
  kandidat tersambung dan ada kandidat pohon terkonfirmasi lain, entry itu
  dapat ditunda30 s (defer_blocked_tree_entry=True). Log deferred_tree_entries
  menyimpan ID, posisi, alasan, timestamp dan inspected=False. Tidak menambah
  completed_trees atau sektor dan tidak mereset blocked/history deadlines.
  Mission memilih dua pohon terukur berbeda, bukan daftar ID wajib. Tidak
  mengklaim pohon yang ditunda sudah diinspeksi atau selamanya unreachable.
  Tanpa alternatif, original observation/failure policy tetap berlaku.
  Jika kandidat hanya tertunda dan demo dua pohon belum terpenuhi, timeout
  pencarian menjadi no_reachable_tree_entry, bukan COMPLETE dengan satu.
- scan_floor shared profile/repair mempertahankan safe low entry: regular
  floor=max(exploration_min, min(takeoff_altitude,initial_scan_height),
  initial_scan_height-max_descent). Untuk entry normal>2 m kebijakan lama
  tetap. Departure floor tetap memakai minimum transit dan bounded descent.
  Tidak memaksa scan naik ke2 m dari entry1.3 m yang sudah diketahui aman.
- Fatal blocked_route kini menyimpan current blockage/destination/deadline
  dan critical snapshot meskipun budget ordinary habis. Dua evidence slots
  accepted/revoked tetap terpisah dan bounded.

Analisis replay: pada export akhir semua entry radius2 m pada1.9–2.1 m
terblokir. Sejumlah point-safe entries1.1–1.5 m ditemukan, tetapi komponennya
terpisah dari start. Audit DIAGNOSTIK dengan unknown dikeluarkan dari sumber
inflasi pun tetap terpisah; mask audit itu TIDAK dipakai planner/kontrol.
Tidak menyimpulkan cukup menunggu scan, atau mengubah unknown menjadi free.
Memperbesar cost preference/bobot juga sebelumnya tidak mengubah rute.

Frozen supervisor replay dari[-3.6998,-7.1,1.103] pada export320.7:
menunda tree3, memilih confirmed tree4; completed tetap[1], blocked_since
308.6 tetap308.6. Proposed goal[3.08470637,-8.15655652,2]. Restorasi
SparseHistory mempertahankan known-cell timestamps, surveyed-pad flags dan
FOV derivation timestamps, bukan memalsukan fresh observations. History
menemukan detour PARTIAL sampai[-.3,-5.3,1.9], bukan mengklaim goal tercapai.
Untuk pemeriksaan geometri TERPISAH saja, harness mengosongkan timer abort
yang memang sudah habis pada run asli: full preparation prefix menuju
[-.54,-5.42,1.78] berhasil, kurva51.1849882 s, tanpa failure. Tidak mengubah
preset flight atau mengklaim stopped flight boleh diteruskan melewati timeout.
Replay ini belum membuktikan tree4 bisa diinspeksi, return atau landing.

Rujukan primer pemisahan supervisor/waypoint failure: dokumentasi resmi Nav2
Waypoint Follower dan source nav2_waypoint_follower/waypoint_follower.cpp,
https://docs.nav2.org/rolling/configuration_and_development/configuration_guide/core_servers/waypoint_follower/
https://github.com/ros-navigation/navigation2/blob/main/nav2_waypoint_follower/src/waypoint_follower.cpp
Nav2 membedakan waypoint gagal dan sukses serta menyediakan kebijakan
melanjutkan waypoint berikutnya. Implementasi fixed ini supervisor tree-entry
rekayasa sendiri, bukan port Nav2/SUPER atau klaim penelitian algoritme baru.
Syarat inspeksi dua pohon dan6 sektor/tree tetap berlaku; failed entry tidak
berubah menjadi success. Default stop_on_failure Nav2 tidak disalin.

Regresi baru: blocked entry resampling mempertahankan radius/cap/mask dan
lulus A*; unknown wall memisahkan safe island dan tidak dibuka; deferral
memilih ID lain tanpa menghitung inspeksi atau reset deadline, dapat dicoba
lagi sesudah30 s dengan sensor tetap wajib sehat; tanpa alternatif tidak
mengarang completion; deferred-only tidak menyelesaikan demo kurang dua;
safe low scan floor tidak dipaksa ke takeoff height; critical timeout
snapshot tersimpan setelah8 ordinary slots.26 targeted lulus5.95 s sebelum
regresi critical snapshot terakhir ditambahkan; full suite sedang berjalan.

Build tiga paket berhasil;464 tes lulus99.62 s,0 errors/failures/skipped.
Bukti build_ros/polinasi_nav/pytest.xml dan /tmp/fixed_entry_build.log.
Commit/push sebelum run12. Belum flight validated untuk pemilihan entry/
deferral baru atau keberhasilan dua pohon pada map bunga.


### 22 Oktober 2026 — run12 fresh depth tetapi settling deadline habis

Flight5ce3e3b, reports/fixed_lidar_validation_20261022_084830,
registered session .dependencies/ros_sensor_palm_farm_flowers_20261022_084832_26394_mapping.
Session stopped=true pada176.90 s simulasi/stale_depth_camera,
HOLD_ABORT/APPROACH tree4.1678 pasangan pose: RMSE .01218397703 m,
maksimum .0233123625 m. Sensor ICP+IMU tanpa ground truth input.
Approach pertama20.60→72.201 s (51.601 s simulasi), SCAN mulai75.45.
Enam sektor pohon pertama (ID2 pada run ini) selesai dan sudah berangkat
ke pohon kedua ID4. ID katalog dapat berbeda antar-run; tidak mencocokkan
pohon fisik hanya dari nomor ID. Belum return/landing/disarm. Entry deferral
baru belum tervalidasi flight: run berakhir SEBELUM kegagalan entry run11.
Tidak membandingkan waktu ini sebagai benchmark kausal sebelum/sesudah SUPER.

Bukti sensor: pada176.9 integrated/valid depth stamp176.6, age .30 s,
cloud age .10 s dan depth_safety action fresh, threshold depth1.0 s.
DepthMotionRecovery dimulai174.85 karena integrated age .65 s melewati
preventive .6, valid age .25. Data kembali fresh175.0 tetapi recovery belum
boleh resume sebelum brake selesai, actual velocity<.04, reference<.01,
post-start map fresh serta body-safe. Cadangan yang dieksekusi berdurasi
1.9805796995 s, v bound .3346830, a bound .2686519, jerk bound .6290619.
Pada176.4 actual .01138 m/s tetapi reference masih .03532 dan brake belum
selesai. Pada176.8 HOLD_ABORT/reference≈0 tetapi actual .06262 m/s;
176.9 actual .07440 dan deadline2.0 s sudah terlewati (elapsed2.05).
Kode mengubah failure menjadi stale_depth_camera walau image/integrasi fresh.
Ini bukti kegagalan settling deadline, bukan bukti kamera mati/lokalisasi loss
atau ICP drift. Tidak menaikkan timeout atau ambang actual settling.

Log juga memuat depth TF extrapolation: requested176.8, latest TF176.7.
Itu membuktikan setidaknya satu frame tidak diproyeksikan karena transform
pada timestamp belum tersedia. Belum membuktikan starvation antrean permanen
atau akar seluruh integration lag; tidak mengganti TF dengan latest stamp,
mengganti label frame atau menyatakan dropout aman. Perbaikan antrean kamera
belum diimplementasikan pada perubahan ini.

Masalah model: konstruksi base Brake memilih durasi konservatif dari
max(.3,3*|v|/a_max,sqrt(12*|v|/j_max),3*|a|/j_max), lalu runtime explicit_backup
memakainya selama valid. Pencarian durasi lebih singkat yang sudah ada hanya
aktif ketika base gagal, sehingga base1.98 s yang valid tetap dipilih meski
batas a/j masih menyisakan ruang. Hampir seluruh recovery2 s habis untuk
kurva referensi, tanpa cukup waktu settling aktual. Belum membuktikan durasi
lebih pendek pasti menyelesaikan tracking/settling closed-loop.

Perbaikan fixed inspection opt-in explicit_backup_duration_optimisation=True:
periksa tiga kandidat straight-stop (base, .85x, .70x). Initial PVA tetap,
terminal v/a=0, endpoint disesuaikan sepanjang displacement base dan koefisien
quintic dihitung ulang, bukan sekadar mempercepat replay sampel. Kandidat
diurutkan menurut durasi dan pertama yang memenuhi derivative hull speed/a/j
serta seluruh live inflated-mask path dipilih. Tidak mengubah limits atau
menganggap slew limiter bukti aman. Bila tak ada yang valid, existing bounded
endpoint fallback tetap berlaku; zero-speed case tetap satu kandidat .3 s.
Perubahan berlaku pada sertifikasi planner dan cached backup final publisher
sehingga brake yang dieksekusi sama dengan kandidat tersertifikasi.

Sumber primer: SUPER backup_traj_optimizer_s4.cpp mengoptimalkan waktu,
initial/terminal state dan posisi dalam koridor dengan batas dinamika,
https://github.com/hku-mars/SUPER/blob/master/super_planner/src/traj_opt/backup_traj_optimizer_s4.cpp
Referensi implementasi melibatkan MINCO/L-BFGS/corridor; perubahan fixed ini
pencarian diskret terbatas tiga quintic stops sendiri, bukan port optimizer
SUPER dan bukan klaim optimum global. Toleransi1.2x bounds yang tampak pada
salah satu check source SUPER TIDAK disalin; hard bounds lokal tetap sama.

Replay sembilan snapshot run12 mempertahankan exact blocked mask dan PVA
metadata: moving examples1.9776146→1.6809724,2.0868015→1.4607611,
2.0856705→1.4599694,1.9563706→1.3694594,1.8437221→1.5671638 s;
zero-speed cases tetap .3→.3 s. Semua kandidat hasil lulus batas yang sama.
Pengukuran wall sekitar1–2 ms/call saat offline replay BERSAMA regresi,
bukan benchmark timing flight atau jaminan Jetson. Snapshot exact PVA trigger
174.85 tidak tersedia, sehingga ini bukan exact replay closed-loop abort.

Diagnostic DepthMotionRecovery kini menyimpan failure_gate
(valid_frame_stale/recovery_settling_deadline), actual/reference speed,
brake presence/duration, post-start-map dan vektor reference PVA serta
measured position/velocity. Failure string/gates/deadline tetap agar auditor
tidak menyembunyikan kegagalan. Diagnostic lintasan lama117.7 s pada status
akhir run12 tidak dipakai sebagai penyebab depth failure.

Regresi baru: shorter backup menjaga initial PVA dan terminal v/a nol,
lulus full path/derivative bounds; occupied start tetap ditolak dan zero speed
bounded; fresh depth tidak membatalkan settling deadline, dengan failure_gate
jelas.53 targeted lulus39.10 s; build tiga paket dan467 full tests lulus
100.77 s (0 errors/failures/skipped). Penambahan vektor diagnostic setelah
build diperiksa py_compile dan8 depth-health tests lulus.38 s.
Bukti build_ros/polinasi_nav/pytest.xml, /tmp/fixed_fast_backup_build.log,
/tmp/fixed_fast_backup_replay.log. Perubahan belum flight validated dan
belum ada misi bunga dua pohon yang COMPLETE/disarm. Commit/push sebelumrun13.


### 22 Oktober 2026 — run13 entry point-safe tetapi arrival reserve gagal

Flightc5a8d3a, reports/fixed_lidar_validation_20261022_091757,
registered session .dependencies/ros_sensor_palm_farm_flowers_20261022_091800_30379_mapping.
Berhenti83.15 s simulasi/blocked_route, BRAKE/APPROACH pohon pertama ID2.
Session stopped=true.741 pasangan pose: RMSE .01051978785 m,
maksimum .0196707233 m. Tidak ada recovery depth atau health fault tercatat;
optimasi backup durasi baru belum tervalidasi terhadap kegagalan run12.
Belum scan/inspeksi dua pohon/return/landing/disarm. Tidak ada GUI atau
hardware yang dioperasikan; kode seluruh run sudah push sebelum launch.

Snapshot fatal baru berfungsi: failure_critical.npz merekam map657,
observasi82.8, command[-6.5,-.9,1.9], tujuan
[-6.584355166,-.853671854,2.0], blocked_since71.10. Pada status akhir,
route_blockage planner_no_route/goal_not_safe, unknown_count0,
occupied_count0. Label goal_not_safe berasal dari _choose_goal=False secara
generik; bukan bukti single-cell tujuan occupied. History result justru
observed_frontier_reached dengan terminal_reserve_rejections pada tujuan.
Raw/guard point-safe dan line_safe menuju tujuan TRUE pada replay exact
snapshot, tetapi route_terminal_reserve_check FALSE: uncertainty+forward PID
menyentuh guard cell centre[-6.7,-.9,2.1]. Footprint raw blocker kosong karena
witness adalah mask yang sudah diperluas buffer, bukan raw cell tujuan.
Jadi ini inkonsistensi pemilihan entry dan kedatangan, bukan kehilangan GPS,
ICP failure atau bukti obstacle dihapus. Tidak menghapus guard.

Penyebab kode: selektor baru hanya menilai safe point, connector dan komponen
free; jika endpoint tetap point-safe ia dipertahankan, meskipun history/live
arrival check sudah menolaknya. Dengan demikian tujuan yang geometri
posisinya sah tetapi near-zero robust arrival-nya gagal dipilih berulang
sampai blocked timeout12 s. Ini mirip pelajaran scan terminal direction,
tetapi sekarang pada entry APPROACH yang baru ditambahkan, bukan klaim
perbaikan scan sebelumnya sudah menangani semua supervisor.

Perbaikan: candidate geometris diurutkan dengan skor yang sama, kemudian
maksimal12 kandidat menjalani A* dengan limit expansions yang sama dan
route_terminal_reserve_check dari edge akhir A* yang SEBENARNYA. Tidak
mengasumsikan arah masuk radial/XY sama dengan detour 3D. Diagnostic
route_candidates_evaluated, rejected.route/terminal_reserve dan witness
menjelaskan penolakan. Budget habis dinyatakan entry_search_budget, bukan
membuka unknown atau menambah budget A*. Seluruh smoothing/full-curve/
braking/live validation tetap menentukan flight.

Selektor juga membaca terminal_reserve_rejections live history, atau terminal
failure hasil kurva, yang witness map_version-nya sama dengan snapshot saat
ini. Jika point tujuan cocok, entry dievaluasi ulang meski single-cell tetap
free; kandidat dengan koordinat yang baru ditolak tidak dipilih lagi pada
snapshot itu. Ini bukan permanent blacklist obstacle/goal antar-map dan tidak
mereset blocked/history deadlines. No-terminal-safe-entry dapat ditunda hanya
dengan syarat occupied-entry/alternatif yang sama seperti sebelumnya.

Replay exact raw mask critical83.15 dan guard yang sama. Pusat aktif belum
tersimpan di snapshot lama: direkonstruksi dari katalog19.6 s sebagai
[-8.539690995,-.433360307], radius ke recorded target tepat2.0 m dan selected
offset sebelumnya -15 derajat. Tidak memakai pusat katalog terbaru yang
sudah bergeser sekitar2.5 cm; pergeseran itu dapat menghasilkan terminal
check berbeda dan tidak boleh diklaim exact old mission. Metadata snapshot
berikutnya kini langsung menyimpan active_tree, tree_angle dan view_target
untuk menghindari rekonstruksi ini. Ini sensor mission state, bukan GT.

Pada replay selektor menolak target lama dan alternatif ketinggian1.9 m
(uncertainty-only menyentuh guard cell[-6.7,-.9,1.9]), lalu memilih entry
[-6.759766252,-1.345428237,2.0], offset -30 derajat dari arah masuk awal.
Dua route candidates diperiksa; sekitar103 ms pada satu replay offline,
bukan benchmark flight. Radius tetap2 m, cap2 m dan minimum transit1 m.
Dengan actual local A* path dari snapshot dipasok lewat interface history
(source jelas frozen_local_A_star_not_live_history), full preparation
berhasil, kurva11.1426071 s setelah certified retiming. Perubahan goal pada
flight tetap menunggu route history baru; replay ini tidak membuktikan route
worker live atau flight lolos. Tidak mengosongkan timer run asli untuk
mengklaim abort boleh dilanjutkan.

Dua regresi baru membuat measured partial wall: entry point/connector/chord
free tetapi near-zero feedback stop gagal; selektor memakai actual A* edge
untuk memilih alternatif yang terminal-safe. Live same-map arrival rejection
memicu seleksi ulang tanpa mereset blocked_since atau menghitung inspeksi.
21 targeted tests lulus7.11 s. Full build/regresi sedang berjalan; belum
flight validated atau berhasil dua pohon pada palm_farm_flowers.

Build tiga paket berhasil;469 tes lulus276.62 s,0 errors/failures/skipped.
Bukti build_ros/polinasi_nav/pytest.xml dan /tmp/fixed_arrival_build.log.
Tidak ada flight saat suite berat dijalankan. Commit/push sebelum run14;
perbaikan kedatangan entry dan pemendekan recovery brake belum flight validated.


### 22 Oktober 2026 sore — run14 early stale depth, antrean menunggu TF

Flight86adf75, reports/fixed_lidar_validation_20261022_190710,
registered session .dependencies/ros_sensor_palm_farm_flowers_20261022_190718_12602_mapping.
Auditor menghentikan pada21.052 s simulasi/stale_depth_camera tepat setelah
TAKEOFF→APPROACH tree1.116 pasangan pose: RMSE .00817304639 m,
maksimum .0192399526 m. Belum lintasan approach/scan/dua pohon/return/landing.
Session stopped=true. Fixed entry arrival fix dan fast backup belum flight
validated oleh run ini, karena belum ada kurva atau recovery dieksekusi.
Tidak membuka GUI/RViz, tidak menjalankan hardware atau menghentikan sesi lain.

Berbeda dari run12: valid depth DAN integrated depth sama-sama20.0 s,
umur1.052 s melewati timeout1.0; depth_safety action fail, stationary_safe
true. DepthMotionRecovery inactive, recoveries0: drone sudah diam, bukan
recovery settling deadline. Cloud age .152 s, map measurement age .552 s.
Frame kamera tetap diterima105,40 submitted (jangan menganggap65 sisanya
semuanya gagal karena TF). Last received20.8; penolakan aktual terakhir:
lookup requested20.6, latest TF20.5, extrapolation into future. Job20.66
mengintegrasikan LiDAR20.5 tanpa depth, selesai20.88; depth map tetap20.0.
Tidak menyimpulkan kamera mati atau error ICP dari failure string tersebut.

Mekanisme kode yang dapat direproduksi: MappingIONode sebelumnya menyimpan
satu depth_pending terbaru. depth_observation memproyeksikan frame kemudian
lookup exact-stamp map dan base_link TF; bila TF belum sampai stamp frame,
fungsi return, tetapi callback image berikut dapat menimpa frame tertunda
sebelum dicoba lagi. Jika urutan callback berulang menempatkan gambar baru
di depan TF, frame lama yang sebenarnya sudah transformable tidak pernah
dipakai. Log membuktikan mismatch timestamp dan frame terima terus berjalan;
belum merekam semua urutan event per frame untuk mengklaim mekanisme ini
sebagai satu-satunya penyebab seluruh run14. Tes reproduksi deterministik
memastikan pola starvation ini ada pada kebijakan latest-only.

Rujukan primer: ROS2 official tutorial tf2_ros::MessageFilter,
https://github.com/ros2/ros2_documentation/blob/rolling/source/ROS-Framework/client-libraries/Working-with-Client-Libraries/Tf2/Using-Stamped-Datatypes-With-Tf2-Ros-MessageFilter.rst
Tutorial menjelaskan sensor message disimpan sampai transform ke frame target
tersedia. Implementasi Python lokal ini antrean nonblocking sendiri, bukan
klaim memasang MessageFilter C++ baru atau mengubah ROS distro/dependencies.
Tidak memakai latest TF, extrapolation, pergeseran timestamp, label-frame
fix atau ground-truth transform pada mode sensor.

Perbaikan: DepthTransformQueue kapasitas6 frame, sorted menurut stamp,
bukan antrean tak berbatas. Pada pemrosesan, buang frame <=last submitted,
future atau aged>=timeout yang SAMA seperti sebelumnya. Cari frame TERBARU
yang dua transform pada header.stamp aslinya tersedia. Frame lebih baru yang
belum transformable tetap menunggu; lookup tidak blocking/spin-wait. Baru
setelah transform tersedia, proyeksi dan body self-filter berjalan. Setelah
berhasil, last_valid/submitted tetap stamp akuisisi asli, bukan receipt/now;
frame lama dibuang/superseded. Mapping integration/health gate tetap memakai
stamp sensor, timeout1.0 dan recovery2.0 tidak diubah. Kapasitas6 adalah
pilihan rekayasa bounded untuk depth5 Hz simulator, bukan tuning hardware
ZED/Jetson yang sudah tervalidasi.

Intrinsics mismatch, invalid projection atau no_valid_depth_rays membuang
frame tersebut tanpa menyegarkan last_valid; NaN/dropout tidak memberi free
space. Queue stats mencatat pending/capacity/oldest/newest, overflow/expired/
obsolete/superseded, TF wait attempts, selected original stamp, receive delay,
queue wait dan acquisition age. Metadata transform memakai lookup map dan
base_link pada waktu yang SAMA; perubahan mounting/extrinsic tidak dilakukan.
Sumber proyeksi simulator tetap simulated_RGBD_not_ZED_stereo, tidak diklaim
hasil ZED2i fisik.

Lima regresi pure queue:20 siklus gambar datang satu periode sebelum TF
masih memproses stamp sebelumnya secara monoton; stale/obsolete tidak
menyegarkan; overflow, out-of-order dan duplikat bounded; unavailable TF dan
invalid frame tidak commit; lookup menggunakan stamp20.0 ketika latest20.2
belum tersedia, bukan transform pada waktu processing20.4.5 tes lulus.03 s.
Adapter ROS memakai Image+CameraInfo, exact-time transform stub dengan origin
tergantung stamp; frame20.0 diproyeksikan memakai TF20.0,20.2 diproses saat TF
menyusul, frame NaN22.0 tidak menghasilkan rays atau memperbarui valid20.2.
30 queue/ROS adapter tests lulus1.83 s. Tes awal29 lulus/1 gagal karena
perbandingan float lookup20.200000000000003 dengan20.2 secara persis; tes
memakai toleransi1e-9 untuk representasi, bukan memperbesar gate umur data.
Build/regresi penuh sedang berjalan. Belum flight validated atau misi bunga
COMPLETE/disarm.

Build tiga paket berhasil;475 tests lulus275.78 s,0 errors/failures/skipped.
Bukti build_ros/polinasi_nav/pytest.xml dan /tmp/fixed_depth_queue_build.log.
Commit/push sebelum run15. Antrean TF depth, arrival-entry fix dan fast brake
belum flight validated sebagai solusi misi dua pohon pada map bunga.


### 22 Oktober 2026 sore — run15 exact pose timeout gagal karena float

Flight065e9a2, reports/fixed_lidar_validation_20261022_192302,
registered session .dependencies/ros_sensor_palm_farm_flowers_20261022_192310_16124_mapping.
Auditor menghentikan pada29.900000000000002 s simulasi/stale_localisation,
BRAKE/APPROACH tree1.204 pasangan pose: RMSE .01032581906 m,
maksimum .02055307581 m. Session dihentikan melalui mekanisme registered run;
belum scan/dua pohon/return/landing/disarm. Tidak ada GUI atau hardware.

Depth queue benar-benar dipakai: pada18.45 selected stamp18.0 dengan
acquisition age.14 s saat proyeksi, receive delay.017 s dan queue wait.123 s;
18 TF wait attempts, pending1. Tidak boleh menyebut queue menghapus semua
jeda mapping: pada28.65 integrated/valid depth age.65 s memicu preventive
recovery, reference baru mulai bergerak sangat kecil; backup.3 s. Recovery
pulih29.30 s tanpa menambah hold timeout2 s. Pada abort29.90 depth age.90 s,
masih dalam camera timeout1.0. Penerbangan ini menunjukkan antrean bekerja dan
recovery depth pendek berhasil, bukan bukti fast brake pada kecepatan tinggi
atau deferral/entry/dua pohon sudah tervalidasi.

Diagnosis stale_localisation TERBUKTI dari angka log, bukan dugaan CPU/drift:
last accuracy pose stamp29.6 dan position
[.02926677160,-.03344435085,1.9802770715] sama persis dengan measured position
pada last_navigation/depth_motion_recovery. Waktu pemeriksaan
29.900000000000002 minus29.6 = .3000000000000007, config pose_timeout=.3.
Health.reason menggunakan0<=age<=timeout tanpa toleransi, sehingga threshold
yang secara desimal tepat.3 ditolak. Last localisation diagnostic29.8 masih
healthy/tracking, scan/IMU29.6,20 sampel IMU dengan max interval.005 s,
ICP correction.00243 m dan residual RMSE.04891 m. Ini tidak membuktikan ICP
permanen gagal atau drift besar. Timestamps pose match memberi bukti sebab
lebih kuat daripada hanya healthy flag setelah abort.

Sumber primer representasi numerik: dokumentasi resmi Python menjelaskan
fraksi desimal umumnya tidak tepat dalam binary float dan perbandingan angka
hasil operasi harus memperhitungkan rounding,
https://docs.python.org/3/tutorial/floatingpoint.html
Perbaikan Health memakai helper fresh_stamp inklusif dengan tolerance1e-9 s
untuk roundoff yang SUDAH dipakai fresh_depth. Fresh_depth kini delegasi helper
sama. Nilai timeout fisik pose/IMU.3, LiDAR.5, depth1.0 dan flag localisation_ok
TIDAK diubah. Toleransi satu nanodetik dicatat eksplisit pada sensor_freshness;
bukan izin jeda puluhan milidetik. NaN/infinity/missing/future timestamps tetap
invalid. Contoh genuinely late.109 s pada gate.1 s tetap ditolak; tidak
menyalin pelonggaran dari sesi rotating atau menyamakan kasus itu dengan
roundoff. local frontend gate.1 s pada IMU tidak diubah.

Status ROS kini menambah sensor_freshness berisi measurement stamps,
pose/cloud/IMU ages, thresholds, numeric tolerance dan localisation_ok.
Missing/nonfinite diagnostics diubah None untuk JSON audit yang valid;
stamp tidak diubah menjadi receipt/now dan tidak dipalsukan agar fresh.
Regresi exact recorded29.900000000000002−29.6 lewat;10 ns extra di luar.3,
real pose/LiDAR/IMU lateness dan future/nonfinite tetap ditolak, serta JSON
status tidak mengandung NaN/Infinity.30 targeted tests lulus43.90 s; full
build/regresi sedang berjalan.

Risiko timing belum selesai: pose saat abort memang sudah berada DI batas
usia sehingga margin kecil; pada run ini mapping worker terakhir sekitar
1508 ms wall dan planner7996 ms wall. Audit host setelah run hanya membaca
CPU/memori (available sekitar11 GiB, load3.50), tidak menghentikan aplikasi
pengguna. Belum ada bukti bahwa background app tertentu menyebabkan fault;
tidak mengubah prioritas/power setting. Fix roundoff tidak menjamin keterlambatan
nyata berikutnya tidak terjadi. Gate umur data tetap wajib menghentikan jika
melewati batas secara nyata. Perubahan belum flight validated atau misi bunga
COMPLETE/disarm.

Build tiga paket berhasil;478 tests lulus274.85 s,0 errors/failures/skipped.
Bukti build_ros/polinasi_nav/pytest.xml dan /tmp/fixed_health_precision_build.log.
Commit/push sebelum run16. Tidak menjalankan flight saat regresi berat.
Fix numeric boundary belum flight validated; genuine late pose tetap abort.


### 22 Oktober 2026 sore — run16 IMU callback tertahan saat ICP

Flight15675a7, reports/fixed_lidar_validation_20261022_194004,
registered session .dependencies/ros_sensor_palm_farm_flowers_20261022_194012_19507_mapping.
Berhenti30.30 s simulasi/localisation_loss, BRAKE/APPROACH tree1.
210 pasangan pose: RMSE .00962039932 m, maksimum .02349342026 m.
Session stopped=true; belum scan/dua pohon/return/landing/disarm. Tidak ada
GUI, hardware atau proses sesi lain yang dihentikan. Recovery depth sempat
pulih29.10 s; tidak menambah deadline. Fix roundoff run15 bukan solusi
terhadap flag estimator false pada run ini.

Bukti pembeda: sensor_freshness pose30.1/age.2, cloud30.2/age.1,
IMU langsung navigation30.295/age.005; localisation_ok=False.
Last localisation debug30.266: ICP reason tracking tetapi healthy=False,
received_imu_stamp30.11 (umur .156 s), core IMU/scan30.1, pending2.
ICP residual .0514527 m, translation correction .0097005 m, inlier .94148;
ini tidak membuktikan ICP gagal permanen atau drift besar. IMU200 Hz masih
diterima node navigasi, tetapi input callback frontend tertinggal sendiri.

Audit kode membuktikan run() memakai rclpy.spin single-thread dan scan timer,
IMU, clock serta health semuanya default mutually-exclusive group. imu()
hanya enqueue, namun tidak mendapat kesempatan saat scan() integrasi lalu
registrasi ICP berjalan. Setelah callback panjang, health dapat dipanggil
sebelum antrean IMU dikosongkan dan menolak raw last_imu age>.1. Node health
sebelumnya menampilkan e.reason tracking, sehingga alasan gate tidak jelas.
Ada bukti interval staleness dan struktur serial; belum ada trace tiap
scheduler event untuk menyatakan semua jitter sistem hanya berasal dari ICP.

Rujukan primer callback groups ROS2:
https://github.com/ros2/ros2_documentation/blob/rolling/source/ROS-Framework/nodes/Working-with-nodes/Using-callback-groups.rst
Dokumentasi menjelaskan grup mutually-exclusive mencegah callback satu grup
berjalan bersamaan; menambah multi-thread executor saja tetap serial bila
semuanya ada pada grup yang sama. Kode rclpy Humble terpasang
/opt/ros/humble/local/lib/python3.10/dist-packages/rclpy/time_source.py
menempatkan subscription /clock pada default group. Karena itu memindahkan
IMU saja tanpa memisahkan scan juga dapat membuat clock tertahan dan
menolak IMU baru sebagai future timestamp. Ini analisis engineering terhadap
source terpasang, bukan klaim hard real-time guarantee dari ROS executor.

Perbaikan fixed sensor frontend:
- Scan timer memakai grup mutually-exclusive terpisah; imu/cloud enqueue,
  clock, FCU feedback/TF reception dan health tetap pada default group.
  run() memakai MultiThreadedExecutor(num_threads=2) hanya untuk sensor body-IMU
  frontend (fixed flag aktif). GT dan node lain, termasuk final setpoint
  publisher, tetap executor lama. Hanya SATU scan callback dapat mengubah
  estimator. Tidak menambah estimator atau publisher odometry kedua.
- Input Lock melindungi enqueue, raw IMU timestamp dan pemindahan batch.
  imu() tidak mengintegrasikan core. Scan hanya mengambil sampel<=timestamp
  scan (tolerance numeric yang sama), lalu melepas lock sebelum ICP. Sampel
  yang lebih baru tetap antre. Buffer500 IMU dan DDS QoS256 tidak diubah;
  overflows dicatat. Tidak mengoreksi scan lalu menerapkan IMU pre-scan lagi.
- Pending cloud yang baru tiba saat lookup TF tidak ikut dihapus ketika
  selected cloud lama diproses/ditolak. Frame/transform tetap pada timestamp
  asli, tidak memakai latest TF atau merestamp predicted pose.
- Health membaca snapshot hasil estimator yang sudah selesai, bukan membaca
  e.p/e.r/registration ketika sedang dimutasi koreksi ICP. Received IMU dan
  pose measurement timestamps tetap live. Scan processing busy/start/end sim
  dan monotonic wall, pending IMU/overflows, pose/IMU ages serta health_gate
  dicatat. reason kini memberi imu_receive_stale/pose_measurement_stale/
  no_good_pose; frontend_reason terpisah menyimpan alasan ICP sebelumnya.
  Ini tidak mengabaikan false health selama ICP: jika actual IMU hilang atau
  accepted pose benar-benar aged-out, gate tetap false walau scan sibuk.
- Gate pose dan input/health IMU memakai fresh_stamp inklusif yang sama,
  hanya numeric tolerance1e-9, limit .3/.1 tidak dinaikkan. Lateness .109
  pada .1 tetap invalid. Source health memakai last_good yang sudah diterima,
  tidak membuat hasil rejected scan menjadi accepted pose baru.

Regresi ROS lama awal27 lulus1.80 s. Tambahan race test memastikan newer
cloud saat TF lookup tetap pending. Tes konkurensi memakai executor ROS2
aktual dan scan stub yang ditahan Event, bukan simulasi penerbangan: clock
1.0 dan IMU .995/1.0 masuk saat ICP callback belum selesai; core integrasi
hanya sampai scan.99 dan IMU1.0 tetap antre. Saat clock maju1.2 tanpa IMU
baru, health_gate imu_receive_stale/BoolFalse tetap keluar SEBELUM ICP
selesai. Ini membuktikan penerimaan/clock independen dan dropout tidak ditutupi.
29 ROS targeted tests lulus1.87 s. API QoS clock tes dibuat eksplisit memakai
QoSProfile(depth=1,BEST_EFFORT), karena Humble ini tidak mengekspor alias
qos_profile_clock. Tidak menambah dependency atau mengubah QoS sensor global.

Batas: dua thread bukan jaminan deadline hardware; jika ICP sendiri lebih
lambat dari pose timeout, pose tetap stale. Snapshot diagnostic mewakili hasil
proses selesai, sedangkan freshness memakai accepted measurement time.
Belum flight validated, belum membuktikan semua jitter/IMU drop hilang,
masih estimator eksperimental scan-to-map ICP+body IMU, bukan FAST-LIO.
Full build/regresi selesai: 480 tests lulus, 0 error/failure/skipped,
278.13 s; tiga package build berhasil. Belum misi bunga COMPLETE/disarm.


### 22 Oktober 2026 — run17: katalog pohon belum tersedia sebelum arming

Commit 2f4352d sudah push; run headless reports/fixed_lidar_validation_20261022_200619,
session .dependencies/ros_sensor_palm_farm_flowers_20261022_200627_24316_mapping.
Runner membatalkan setup setelah tiga pemeriksaan, tanpa arming/flight.
setup.json: localisation_healthy=true, measurements_fresh=true, kontrol satu
producer=true, depth fresh=true; fcu_externalnav_aligned=false. Pada snapshot
13.95 s, health_reason=waiting_for_confirmed_measured_tree, tree_catalog_stamp=null,
perception_statistics={}, fcu_frame_aligned=false. Pose stamp13.7, cloud13.8.
Tidak ada exception tree detector selama run. Infinity pada position_difference
adalah sentinel check_ros_simulation ketika lookup map<-fcu_local gagal,
BUKAN bukti posisi estimator Inf atau drift. Alignment dibuat NavigationNode
hanya setelah seluruh prasyarat termasuk measured tree tersedia.

Audit sumber TreeDetectorNode membuktikan antrean maxlen1 selalu diganti cloud
terbaru dan lookup memakai timestamp cloud tersebut. Dengan TF pose13.7 dan
cloud13.8, cloud terbaru belum transformable. Scan lama yang TF-nya menyusul
bisa hilang sebelum tick berikutnya. Ini mekanisme starvation yang dapat
mereproduksi katalog tidak keluar; belum ada trace lookup per tick pada run17,
jadi frekuensi kegagalan TF live run17 belum terbukti. Tidak mengubah batas
alignment, freshness atau validasi kandidat untuk meloloskan setup.

Perbaikan memakai kembali bounded DepthTransformQueue6 pada detector:
pilih cloud TERBARU yang exact-time map<-mid360 tersedia, simpan newer pending,
expire dengan cloud_timeout .5 yang sama, acknowledge monotonic submitted stamp.
Tidak memakai latest transform, label frame palsu, atau restamp. Submit tetap
maksimum1/s dengan satu worker. Queue age/drop/wait/selected stamp dicatat
setiap1sim s pada ros.log serta hasil katalog/perception_statistics. Tidak ada
GT masuk estimator, deteksi, planner atau kontrol. Ini prinsip stamped message
filter yang sudah dirujuk pada perbaikan depth sebelumnya, bukan algoritma SUPER.
Tes adapter mereproduksi cloud13.6/13.7/13.8 dengan TF sampai13.7: submit13.7,
simpan13.8, lalu clock15 tanpa data baru tidak submit ulang cloud kedaluwarsa.
Targeted43 tes lulus16.24s; build tiga package berhasil dan full481 tes
lulus275.31s, tanpa error/failure/skipped. Perbaikan belum flight validated.


### 22 Oktober 2026 — run18: setup lolos, depth map stale sesudah takeoff

Commit2308c6f dipush, run reports/fixed_lidar_validation_20261022_204312,
session .dependencies/ros_sensor_palm_farm_flowers_20261022_204320_30402_mapping.
Setup/alignment/arming lolos. Log detector membuktikan scan5.2 dipilih dengan
age.20/queuewait.128s meskipun newer scan5.3 pending; perbaikan antrean detector
berjalan live. Takeoff lalu SURVEY/APPROACH tree1 pada20.75s, belum trajectory
approach aktif. Run dihentikan auditor pada21.401: stale_depth_camera,
last_valid=last_integrated20.4, age1.001>timeout1s. lastreceived20.8 adalah
snapshot stats, bukan bukti kamera berhenti pada20.8. Map LiDAR20.5 age.901,
cloud21.3 age.101. Recovery inactive karena belum trajectory/commandv aktif;
actualspeed.049325, reference speed.001439. Tidak menambah timeout/dropout grace.
121 pasangan audit RMSE.0082818933m, max.0178873294m. Tidak ada terminal
localisation_loss; ini belum membuktikan IMU/pose deadline selalu terpenuhi.

Pipeline map20.5: receive-to-submit942.865ms wall, mapworker1421.992ms,
result delivery280.189ms, snapshot/decode80.318ms; total2964.138ms wall
measurement-to-publish.5sim s. Ini membuktikan keterlambatan pipeline, bukan
bukti suatu aplikasi host tertentu penyebabnya. Pembuat peta memakai pending
maxlen1 dan exact latest scan TF: pola starvation sama dengan detector.
Kontribusi relatif TF wait vs komputasi pada run18 belum terukur per tick.

Perbaikan terbatas pada MappingIONode: bounded queue6 memakai helper sama,
pilih newest exact map/mount-transformable dalam cloud_timeout.5s yang sama,
pertahankan newer TF-pending, ack submitted stamp monotonik. NavigationNode
map_pending mendukung queue ini, legacy deque jalur lain tetap sesuai semula.
Statistik mapping_lidar_transform_queue ikut snapshot; freshness cloud receive,
map LiDAR, depth valid dan depth integrated tetap terpisah. Tidak mengganti
timestamp, menganggap data hilang sebagai free, atau menambah producer kontrol.
Tes adapter memasukkan20.7/20.8/20.9 dengan TF sampai20.8, memverifikasi submit
map20.8 dan simpan20.9; clock22 tanpa scan baru tidak submit data expired.
38 targeted ROS/mapping/queue tests lulus2.86s. Build tiga package berhasil.
Suite penuh:481 passed,1 failed/482 pada274.34s. Satu failure berasal dari
assert lama test_mapping_isolation node.pending.maxlen==1, yang mengasumsikan
implementasi antrean lama. Assert itu dihapus; pemeriksaan semua publisher
kontrol/flight tetap ada. Tes perilaku antrean baru memeriksa original stamp,
exact TF, newer retention, expiry, dan tidak refresh saat dropout.
Recheck via colcon -k mapping_process_has_no_control_or_flight_command_publishers:
1 passed1.89s,481 deselected; colcon test-result0failure. Jadi seluruh482 kasus
sudah mendapat hasil lulus melalui suite+recheck, bukan satu full invocation
482pass. Kode runtime tidak berubah sesudah suite penuh tersebut.
Perbaikan belum flight validated dan belum jaminan seluruh backlog teratasi.
Run18 stopped=true terverifikasi pada session.json; peta map.json/map.html
masing-masing12MB dan navigation_debug.jsonl tersimpan di reports/ros_sensor_*
run18/map_3d, tidak dimasukkan commit.


### 22 Oktober 2026 — run19: health pose lama tertinggal setelah pose baru masuk

Commit9854251 dipush. Run reports/fixed_lidar_validation_20261022_205704,
session .dependencies/ros_sensor_palm_farm_flowers_20261022_205712_34185_mapping.
Setup/arming/takeoff lolos, lalu terminal localisation_loss21.55s,
APPROACH tree1 belum trajectory aktif/sektor inspeksi.119 audit pairs,
RMSE.0093413510m/max.0213694729m. Depth integrated20.8 age.75 masih fresh;
valid21 age.55, received21.2. Mapping queue memilih scan21.0 age.26s,
newer21.1 pending. Jadi antrean mapper berfungsi live, tetapi ini belum
validasi bebas depth timeout sepanjang flight.

Localisation debug21.501: healthyFalse/pose_measurement_stale,
acceptedpose21.2 age.301>limit.3 (nyata, bukan roundoff). IMU21.5 age.001,
queue39/overflow0, ICP scan21.3 berjalan sejak21.423, frontend_reason tracking.
Residual scan21.2 .050574m, inlier.93944, correction.01262m, tidak membuktikan
kegagalan ICP permanen/drift. Pada navigation21.55, pose21.3 SUDAH diterima
(age.25), tetapi localisation_ok masihFalse dari timer health sebelumnya.
Sumber LocalisationNode menerbitkan health hanya timer.1s, bukan saat pose
accepted, sehingga kondisi gagal pose lama bertahan sampai timer selanjutnya.
IMU queue input fix tetap bekerja: raw IMU age.001 saat ICP busy.

Perbaikan: commit accepted pose, estimator diagnostic snapshot, dan health
update segera pada scan selesai di bawah RLock keluaran yang sama dengan
timer health. Heavy ICP tetap di luar lock; input IMU tidak mengambil lock
keluaran. Tidak mengabaikan False, tidak menambah debounce/grace, tidak
mengubah limit pose.3/IMU.1. Timer tetap menerbitkan kegagalan yang nyata
selama ICP busy; accepted scan baru mengevaluasi ulang semua gate segera,
sehingga tidak menunggu tick.1s untuk menerbitkan sehat bagi pose baru.
Jika navigator benar-benar memeriksa saat pose stale sebelum hasil baru
tersedia, fault tetap sah dan masih mungkin. Ini perbaikan publikasi status,
BUKAN klaim ICP latency sudah diperbaiki. Menggunakan praktik serialisasi
callback output ROS2 yang dirujuk sebelumnya, bukan implementasi SUPER.

Tambahan last_scan_processing menyimpan stage waktu wall IMU, preprocessing,
registration, start/end sim/wall, accepted flag melintasi scan berikutnya;
scan_processing live tetap menunjukkan pekerjaan busy. Diperlukan untuk
audit biaya ICP bila deadline tetap gagal. Tes actual node mereproduksi
False pada21.501 untuk pose21.2, accepted21.3 langsung menerbitkanTrue tanpa
timer; accepted berikutnya dengan IMU stale tetap menerbitkanFalse.
41 targeted ROS/sync/localisation tes lulus2.50s; build tiga package berhasil
dan seluruh483 tes full suite lulus276.25s,0error/failure/skipped.
Ini sekaligus memverifikasi ulang semua kasus antrean mapping/isolasi pada
satu invocation penuh. Flight perbaikan health belum tervalidasi.


### 22 Oktober 2026 — run20: deadline pose nyata dan biaya decoding point cloud

Commitb19a244 dipush, run reports/fixed_lidar_validation_20261022_211046,
session .dependencies/ros_sensor_palm_farm_flowers_20261022_211054_37807_mapping.
Setup/arming/takeoff lolos; terminal stale_localisation23.55s, APPROACH tree1,
belum trajectory/sektor aktif. Pose23.2 age.35s NYATA melewati.3; IMU23.55
age0, cloud23.4 age.15, localisation_okTrue dari health23.5 (age.3000000000000007
masih roundoff-valid). Jadi tetap abort dari gate pose langsung, tidak
tertutupi healthTrue. Depth integrated22.8 age.75 fresh.141 audit pairs
RMSE.0087049068m/max.0191252652m. Run dihentikan oleh auditor/session cleanup.

Perbaikan event health terbukti live: False pose stale23.201 disusul True
segera saat accepted scan23.0 selesai23.224; False23.302 disusul True23.345
ketika scan23.1 selesai. Namun latency nyata masih perlu dikurangi; tidak
mengubah gate/debounce untuk meloloskan misi. Stage scan23.2: start23.345,
end23.449; IMU8.462ms, preprocessing233.552ms, registration417.457ms WALL.
Scan22.9 preprocessing436.816ms/registration516.124ms. Ini mengukur stage,
bukan bukti seluruh biaya berasal dari satu baris atau aplikasi host.

Audit cloud_points memakai list Python per titik setelah read_points Humble
SUDAH menghasilkan structured NumPy array. SensorGate juga mengulang tuple
per titik untuk XYZ/ring. Referensi primer: definisi PointCloud2 Humble
https://github.com/ros2/common_interfaces/blob/humble/sensor_msgs/msg/PointCloud2.msg
serta sensor_msgs_py terpasang /opt/ros/humble/lib/python3.10/site-packages/
sensor_msgs_py/point_cloud2.py (dtype_from_fields/read_points_numpy).
Perbaikan cloud_fields membaca view NumPy ber-stride row_step/point_step,
byte order eksplisit, field offsets memakai dtype_from_fields resmi; output
XYZ float64 dan finite filtering cloud_points tetap sama. Raw fields pada
SensorGate tetap mempertahankan NaN/Inf/ring hingga aturan normalize dan
body_shadow yang sama; invalid tidak diubah jadi free. Tidak mengubah
jumlah titik, voxel resolusi, iterasi/gate ICP atau sumber GT. Byte buffer
input tidak dimutasi, termasuk big-endian; organized row padding ditangani.

Benchmark sintetis offline, simulator berhenti, seed42,22 iterasi buang2warmup,
median:11520points legacy45.591544ms -> vectorised.759034ms (60.07x),
24000points95.573419 ->2.172659ms (43.99x),60000points244.965177 ->5.934681ms
(41.28x). np.testing.assert_array_equal hasil lama/baru lulus termasuk
NaN/Inf filtering. 11520 sesuai360x32 model sumber, bukan rekaman scan flight.
Angka ini ONLYdecoder, bukan bukti ICP60x lebih cepat atau deadline flight
terpenuhi. Script/hasil disimpan reports/fixed_cloud_decode_20261022.
53 targeted ROS/IMU/localisation/simulated-ray tests lulus2.50s, termasuk
reordered/padded/mixed fields, little/big endian tanpa mutasi, row padding,
empty/malformed stride, finite filtering dan raw invalid/ring preservation.
Build tiga package berhasil; full487 tests lulus276.68s,
0error/failure/skipped. Perbaikan decoder belum flight validated.


### 26 Oktober 2026 — lanjut loop; audit run21 dan batas numerik umur peta

User meminta stop/keluar loop22Okt, lalu mengizinkan lanjut26Okt. Audit worktree
fixed-lidar9adab20 bersih, original tetap rotating-lidar6779baf dengan perubahan
lokal yang tidak disentuh. Worktree dan resource runtime diperiksa kembali;
UDP9002/14650,TCP5762 dan proses simulator/RViz bebas. Tidak membuka GUI.

Run21 sebelum stop: reports/fixed_lidar_validation_20261022_212458,
session .dependencies/ros_sensor_palm_farm_flowers_20261022_212506_41792_mapping.
Terminal stale_integrated_map30.200000000000003, mapstamp29.2,
age1.0000000000000036 terhadap batas inklusif1s. Kode gate memakai > tanpa
fresh_stamp, berbeda dari pose/depth yang sudah mengatasi cancellation.
Ini bukti false positive batas numerik, bukan bukti seluruh pipeline map cepat.
210 audit pairs RMSE.0084821356m/max.0196630793m; pose30 age.2, IMU30.2 age0,
localisation_okTrue. Depth valid29.6 age.6, integrated29.2 age1+roundoff.
Recovery depth28.3 ->28.85 selesai. Belum trajectory approach selesai/sektor.

Optimasi decoder tervalidasi berjalan live: scan30.0 start30.07 end30.124,
IMU19.806ms/preprocessing61.987ms/registration283.419ms wall. Ini sampel,
bukan controlled whole-flight speedup atau jaminan deadline. Map masih mahal:
worker29.2 total1656.175ms wall, lidar350.190/depth185.012/restore-inflate-
delta1120.973ms. Pipeline total3687.704ms wall, measurement-to-setpoint.6sim s.
Biaya map adalah kendala yang tetap harus ditelusuri jika timeout nyata muncul.

Perbaikan memakai fresh_stamp sama pada penerimaan snapshot dan gate map
MISSION: batas1s tetap, tolerance1e-9 hanya numeric, future/nonfinite ditolak.
Tidak memperbarui umur map dengan waktu transport atau menutupi dropout.
Tes actual ROS snapshot mereproduksi30.2-29.2>1floating tetapi diterima;
30.20001 untuk observasi sama tetap ditolak. Targeted7 tes lulus1.09s;
build tiga package berhasil dan full489 tests lulus286.25s,0error/failure/
skipped. Tes awal diperbaiki pada fixture grid.rebuild sebelum serialisasi
serta memakai timestamp30.200000000000003 persis dari log.
Perbaikan batas umur peta belum flight validated.


### 26 Oktober 2026 — run22: pose stale nyata; exact ICP correspondence pruning

55a60b1 sudah push. Run reports/fixed_lidar_validation_20261026_143211,
session .dependencies/ros_sensor_palm_farm_flowers_20261026_143219_7574_mapping.
Setup/arming/takeoff lolos; terminal stale_localisation27.65, APPROACH tree1,
planning pending, belum trajectory/inspeksi. Pose27.3 age.35>.3, IMU27.65
age0, LiDAR27.5 age.15, localisation_okTrue dari health27.6. Depth integrated
26.6 age1.05>1; map26.7 age.95. Jadi bukan false map-boundary yang sama;
pose/depth processing sungguh terlambat.184 audit pairs RMSE.0088757310m,
max.0210614416m. Auditor/session cleanup sudah menghentikan run.

Local ICP27.3 mulai27.375 selesai27.509, IMU21.608ms/preprocess102.029ms/
registration766.423ms wall. Raw IMU27.595 age.005, antrean36/overflow0;
frontend tracking,residual.050954m,reference17572, tidak ada bukti drift
permanen. Map26.7 pipeline4126.255ms wall dengan receive-submit1453.525,
worker1795.129,resultdelivery486.974,snapshot221.640. Tidak melonggarkan gate.

Profiling offline memakai resampling1800permukaan historical map + noise.004,
15scan diterima. Referensi48751points (lebih besar daripada live17572 dan
limit normal40000 karena harness langsung menyuntik offline reference).
Bukan replay raw scans/flight. Dari total0.617s,query cKDTree123calls menyita
.357s; scan total.585s. Ini mengarahkan optimasi query, bukan membuktikan
semua jitter live disebabkan query. Audit host read-only tidak memberi
bukti quota throttling; tidak menghentikan aplikasi pengguna.

Referensi primer SciPy cKDTree.query:
https://docs.scipy.org/doc/scipy/reference/generated/scipy.spatial.cKDTree.query.html
Opsi distance_upper_bound memang memangkas pencarian; eps default0 exact,
missing distanceinf/index tree.n. Perbaikan memakai batas.5m SAMA dengan
inlier gate lama distances<.5, bukan approximate/peningkatan threshold.
Missing index dipetakan0 hanya untuk indexing array normal yang masked;
distanceinf tetap bukan inlier. KNN12 untuk pembangunan normal tetap sama,
jumlahscan/point/resolusi/iterasi/rank/residual/posegate tidak diubah.

Benchmark offline48751reference,2000queries (1800resampled/noisy surface +
200faroutliers),45iterasi buang5warmup: median unbounded5.153073ms,
bounded2.626659ms,1.9618x. Inlier mask, distance dan index untuk inlier
array_equal. Angka ONLYquery offline dengan10%faroutlier, bukan flight gain.
Tes sequential8scan robust ICP dengan noise/outliers: acceptance/reason,
p/r/v/covariance sampai1e-12 dan residual/timestamp sama dengan unbounded.
Tes exact.5border/missing/outlier aman.47 targeted ICP/ROS/IMU tes lulus2.63s.
Script/profiling/benchmark tersimpan reports/fixed_icp_pruning_20261026.
Build tiga package berhasil; full491 tests lulus291.99s,
0error/failure/skipped. Optimasi ICP belum flight validated.


### 26 Oktober 2026 — run23: lokalisasi melewati gate, recovery depth berulang

Commit7724920 dipush. Run reports/fixed_lidar_validation_20261026_144754,
session .dependencies/ros_sensor_palm_farm_flowers_20261026_144802_11509_mapping.
Setup/arming/takeoff lolos, first APPROACH belum selesai. Auditor berhenti
35.05s dengan repeated_depth_recovery setelah empat brake starts28.85,
30.85,32.8,35.05 (tiga recovery sebelumnya selesai29.55/31.5/33.6).
Masih mempertahankan batas pengulangan/grace/timing, tidak melonggarkannya.
257 audit pairs RMSE.0074337290m/max.0172949725m. Tidak ada terminal
localisation_loss/stale_localisation pada run ini; bukan bukti estimator
bebas jitter selamanya. Lastscan34.9 start34.95 end34.981, IMU7.058ms,
preprocess89.733ms,registration112.592ms WALL. Ini sampel live, bukan
controlled speedup yang bisa seluruhnya diatribusikan kepada exact query.

Map34.4 worker1633.949ms wall, LiDAR442.757/depth185.749/restore-inflate-
delta1005.443ms. Pipeline2994.320ms wall,measurement-to-setpoint.5sim s;
umurdepth.65 saat brake, thresholdpreventive.6. Pipeline lambat memicu
pengereman/replan berulang meskipun validdepth age.45/actualspeed.0176.
Peta harus diproses lebih cepat; tidak mengizinkan peta basi demi progress.

Profiling offline memakai history run23 beku dan12rebuildwindow: total5.275s,
restore3.089s(.257/window),fov_priorlookup1.382s(.115/window),rebuild.744s.
Ini TIDAK mencakup ray integration atau current-scan FOV classify; bukan
profiling keseluruhan pipeline flight. Restore memakai nested list tuple
state/stamp lalu np.asarray; FOV lookup juga membentuk list scalars.
Perbaikan memakai structured np.fromiter dengan count eksplisit dan state
int8/stampfloat64; lookup FOV scalar fromiter, prior membership map. Prinsip
preallocation referensi primer https://numpy.org/doc/stable/reference/generated/numpy.fromiter.html
Dipakai NumPy yang terpasang, tidak perlu dependency baru.

RollingMapper menghitung local/global key array satu kali dalam satu update,
lalu restore dan FOV lookup berbagi koordinat itu. Tidak cache state/seen,
tidak menyimpan tambahan array pada snapshot/transport/history global,
tidak enumerasi seluruh farm, tidak memperbesar map/window. Original
occupied/free/unknown, TTL, surveyed launch pad dan persistent FOV prior
semantics/timestamps tetap. Lidar/depth input timestamp, inflation, EDT,
collision/braking/clearance tidak diubah.

Benchmark offline terhadap fungsi legacy dari git7724920,16repeats buang2:
restore median194.789941ms ->130.131178ms,1.4969x. Percobaan awal199.119919
->125.118536ms,1.59x; variasi timing ini bukan flight guarantee. Frozen window
pada now35.05 dan500 membuktikan array_equal state,seen,assumed_free,
assumed_occupied,blocked,blocked_actual,distance,free_distance dengan legacy.
27 targeted coverage/FOV/transport tests lulus3.71s. Tambahan regression
memastikan coordinate reuse tetap membaca occupancy/timestamp terbaru,
expired measured free unknown, occupied tetap, prior hanya policy existing,
FOV lookup tidak restamp derivation. Fixture awal diubah dari exact voxel
boundary ke voxel centres dan memakai riwayat independen tiap window.
Script/profiling/benchmark di reports/fixed_map_restore_20261026.
Build tiga package berhasil; full493 tests lulus281.26s,
0error/failure/skipped. Optimasi restore belum flight validated.


### 26 Oktober 2026 — run24: restore lebih ringan, recovery tetap berulang

ab19179 sudah push. Run reports/fixed_lidar_validation_20261026_150747,
session .dependencies/ros_sensor_palm_farm_flowers_20261026_150755_15996_mapping.
Terminal repeated_depth_recovery35.05, APPROACH tree1 belum selesai.
Brake starts28.75/30.75/32.85/35.05, resume29.25/31.304/33.5.259audit pairs,
RMSE.0110657610m,max.0190005601m. Tidak ada terminal localisation fault.
Map34.4 worker1585.649ms wall: LiDAR584.582/depth238.023/restore-inflate-
delta763.044ms. Substage restore lebih rendah daripada sampel run23 1005ms,
namun wholeworker masih1586vs1634ms dan pengulangan recovery sama. Tidak
mengklaim causalcontrolled gain atau keberhasilan mengurangi jeda flight.
Pipeline3264.236ms,measurement-to-setpoint.55sim s. Depth age.65 saat brake.
LastICP34.9 start34.94 end34.978,registration175.960ms wall.

Perbaikan berikut adalah exact memoisation geometri, BUKAN toleransi umur
peta adaptif dan BUKAN asumsi lingkungan stabil. VoxelMap.rebuild tetap
memeriksa expiry measured free pada now terlebih dahulu, memperbarui
state/version seperti semula. Mask sumber mencakup measured occupied,
unknown dan persistent FOV priors dengan kebijakan semula. Inflasi hanya
reuse jika origin,resolusi,kernel plan/actual dan SEMUA elemen mask sumber
identik. EDT occupied hanya reuse jika mask occupied identik; EDT free hanya
reuse jika inflated blocked identik. Perubahan lain menghitung ulang.

Cache privat dimiliki RollingMapper worker, bukan history global/grid
transport. Input mask dan output geometri disalin independen; mutasi map
hasil tidak merusak cache dan map hasil tetap writable untuk expiry di
navigation. Measurement seen/last_observation/depth/IMU/pose/timeouts tidak
cache/restamp. Integrasi LiDAR/depth aktual tetap dilakukan pada tiap job;
ketiadaan scan baru tetap stale. Tidak menggunakan unchanged map sebagai
bukti lingkungan diam. geometry_reuse flags dicatat di mapping_worker_stages
untuk membuktikan hit/miss live, belum tahu hitrate saat bergerak/noise.
Ini memoisation perhitungan deterministik milik rekayasa sendiri, bukan
klaim metode dari paper SUPER atau mekanisme umur map rotating-lidar.

37 targeted geometry/coverage/FOV/transport tes lulus3.91s: perubahan
occupied,unknown,FOV free/virtual occupied,TTL,origin,resolution,kernel plan
&actual harus menghasilkan arrays setara rebuild penuh. Observasi baru
mempertahankan timestamp baru walau geometri sama. Mutasi output tidak
poison cache. Benchmark offline IDENTICAL geometry saja,26repeat buang2,
legacy function gitab19179 median61.720718ms ->cached1.736060ms,35.55x,
array_equal state/seen/blocked/blocked_actual/distance/free_distance.
Ini hanya rebuild, tidak termasuk integrasi/restore dan bukan flight gain.
Script/hasil di reports/fixed_geometry_cache_20261026. Harness pertama
memerlukan import UNKNOWN agar fungsi legacy dapat dieksekusi; sudah dibetulkan.
Build tiga package berhasil; full503 tests lulus279.74s,
0error/failure/skipped. Reuse geometri belum flight validated.


### 26 Oktober 2026 — run25: reuse geometri jarang; indeks blok exact untuk riwayat

9c8ea37 sudah push. Run reports/fixed_lidar_validation_20261026_152619,
session .dependencies/ros_sensor_palm_farm_flowers_20261026_152627_20225_mapping.
Terminal repeated_depth_recovery34.25s, masih APPROACH tree1, belum sektor.
251audit pairs RMSE.0100117422m/max.0220667613m. Brake28.25/30.3/32.35/34.25,
resume28.95/31.05/32.901. Tidak ada terminal localisation failure.
Dari104completed mapping jobs yang teramati (dedup worker_finished_ns),
inflation reuse0,occupiedEDT1,freeEDT7; window origin berubah42kali.
Jadi benchmark identical-mask35x tidak mewakili manfaat flight ini. Cache
geometri dinonaktifkan DEFAULT; tetap opsional mapping_geometry_reuse=true
untuk eksperimen terpisah, dengan test safety/expiry yang sudah ada.

Lastmap33.6 worker1713.459ms wall: lidar574.777/depth202.568/restore-inflate-
delta936.115ms; pipeline3260.575ms wall,.55sim measurement-to-setpoint.
LastICP34.1 start34.157 end34.206,registration228.990ms. Peta tetap bottleneck.
Tidak menaikkan threshold.6,hold2s,jumlah recovery,clearance atau speed cap.

Referensi primer layout block-hash Voxblox Layer/Block:
https://github.com/ethz-asl/voxblox/blob/master/voxblox/include/voxblox/core/layer.h
Kode upstream diperiksa via raw.githubusercontent.com. Yang diadaptasi hanya
ide akses voxel per blok kecil, BUKAN memakai TSDF/ESDF Voxblox, integratornya,
atau mengklaim SUPER/FAST-LIO. Implementasi baru BlockWindowIndex Python/NumPy
memakai blok16^3, state int8/seenfloat64/priorbool/FOVcodeint8. Blok hilang
UNKNOWN. Slice blok yang overlap window menggantikan lookup dictionary
per voxel. Tidak mengurangi resolusi/ray, memperluas free, atau mengubah
mounting/frame. Ekspor dictionary canonical tetap ada untuk audit/history
routing dan delta, index hanya di worker RollingMapper, tidak dikirim ke
navigation/history-route process. Perubahan state/timestamp/prior/FOV index
mengikuti update/apply_delta/fov_priors yang sama; occupied precedence dan
TTL tetap. Assumption derivation timestamps tetap dictionary canonical;
lookup tidak pernah refresh timestamp.

mapping_block_index=true adalah default worker; false menyediakan backend
pembanding. SparseHistory pada proses lain default tanpa index. Statistik
block count/side/array bytes dicatat di mapping_worker_stages.block_window_index.
Biaya memori bertambah bounded oleh blok yang benar-benar dialokasikan,
bukan full dense whole-farm grid. Dimensi lastblock tetap dicek terhadap
bounds; unseen slots UNKNOWN,seen-inf,priorFalse,FOV0.

30targeted index/coverage/FOV/transport tests lulus5.03s (sebelumnya34existing
coverage/FOV/geometry tests lulus3.67s). Tambahan random8batch,expiry,recentering,
occupied wins,prior dan FOV/delta menyamai dictionary; sequential complete
RollingMapper dengan backend on/off menyamai state,seen,assumptions,blocked,
blocked_actual,distance,free_distance dan delta. Tes lookup melarang get()
dictionary per voxel pada index path. Geometry-cache default off diverifikasi.

Benchmark frozen history run25,18repeat buang2, restore+FOV lookup:
dictionary167.052947ms ->blocks9.550283ms (17.49x). Percobaan awal181.533370
->9.679242ms (18.75x).119blocks,array bytes5361664 (belum termasuk Python
overhead). Update canonical12000voxels median15.233817ms ->33.726891ms
TERMASUK pemeliharaan index; tambahan18.49ms ini lebih kecil daripada
penghematan lookup pada benchmark tersebut. Tetap ONLYoffline, bukan
keberhasilan timing/flight. Frozen masks+seen+distances array_equal dengan
backend dictionary pada now35.05/500 dan dua window. Artifact script/hasil
di reports/fixed_block_index_20261026. Build tiga package berhasil;
full506 tests lulus281.46s,0error/failure/skipped. Indeks blok belum
flight validated.
