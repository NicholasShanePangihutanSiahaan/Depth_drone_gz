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
