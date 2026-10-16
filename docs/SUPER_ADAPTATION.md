# Adaptasi ide SUPER untuk fixed-lidar — 15 Oktober 2026

Worktree `/home/abin/polinasi_lidar_fixed`, branch `fixed-lidar`. Ini adaptasi
engineering ke planner voxel/PVA ArduPilot yang sudah ada, bukan port lengkap
SUPER, MINCO, CIRI, atau controller on-manifold MPC mereka.

## Lima pekerjaan dan bukti

| Pekerjaan | Implementasi/evaluasi | Batas validasi |
| --- | --- | --- |
| Validasi lintasan kontinu | Handoff t=32.7 s pada run d3f578b terbukti; replay baru menyertakan PID, envelope, async handoff, dan backup | Run lama gagal t=33.85 s; flight kode baru dicatat di jurnal |
| Profil kecepatan | Envelope feedback + ketidakpastian kecepatan; segment limits dari braking; forward/backward knot speeds; kurvatur dan adaptive phase tetap | Skenario uncertainty longitudinal .03 m/s, bukan jaminan semua gangguan 3D |
| Optimasi durasi | Densifikasi ruas lurus agar ada knot cruise; bounded coordinate search durasi dengan PVA sambungan tetap | Quintic C2, bukan joint spatial-time MINCO atau minimum waktu global |
| Backup berhenti eksplisit | Kurva stop materialized dari final PID PVA, turunan/hull/voxel diperiksa, cached dan diperiksa ulang saat abort; pencarian tiga batas handoff dengan future stop | Runtime memakai kandidat stop terikat; optimizer endpoint/durasi alternatif tersedia untuk evaluasi, tidak diaktifkan pada loop PID |
| CIRI/controller | Baseline koridor convex box voxel dipakai mempercepat pengecekan kandidat durasi; kode CIRI dan syarat integrasi diaudit; model MPC lama diperiksa | CIRI tidak diport; MPC tidak diaktifkan karena envelope model tidak cocok |

## Penyebab masalah baseline dan perbaikan

Baseline d3f578b memerintahkan reference hampir .4 m/s, sama dengan batas
speed fisik konfigurasi. Pada t=33.85 s, velocity baru .398997 m/s dan output
PID sebelumnya menyebabkan prediksi Brake mencapai .401589 m/s. Gate live
sekarang mencatat subreason dan input percepatan pada route_failure_diagnostics,
sehingga predicted_speed_limit tidak lagi tersamar sebagai obstacle clearance.

Reference cap baru dihitung melalui model Brake dengan ekstra koreksi PID
.15 m/s² dan uncertainty longitudinal .03 m/s. Dengan konfigurasi saat ini,
cap hasilnya sekitar .34897 m/s. Ada cadangan numerik pada pencarian profil,
kemudian seluruh kurva tetap disertifikasi braking pada grid snapshot dan
command akhir diperiksa pada peta live. Batas speed .4, acceleration .5,
jerk 1.0, clearance, tracking limit, dan health timeout tidak dinaikkan.
Ketidakpastian lebih besar dari skenario tersebut tetap dapat memicu stop.

Profil per ruas mengecek sampel koridor menuju depan; hasilnya adalah
pre-profile, bukan sertifikat seluruh kurva. Ruas terakhir memakai taper menuju
kecepatan nol; titik berhenti tidak memerlukan ruang jelajah di luar tujuan. Batas kurvatur dan propagasi
forward/backward membuat drone melambat sebelum ruas sulit. Pencarian waktu
mempertahankan PVA setiap sambungan, memeriksa bound hull turunan dan geometri,
lalu planner bersama memeriksa braking .05 s sepanjang hasil akhir. Buffer
canopy pada ascent/scan juga diterapkan ke explicit backup.

## Semantik backup dan unknown

Backup adalah objek kurva yang dapat dieksekusi lewat jalur BRAKE yang sama,
bukan publisher kontrol tambahan. Sebelum memakai cache saat fault, PVA awal
harus sama dengan command terakhir dan kurva dicek kembali pada peta live.
Jika cache tidak cocok/terhalang, cache ditolak dan jalur emergency lama tetap
melakukan pemeriksaan sendiri. Kegagalan cache tidak berarti emergency aman.

Handoff mencari tiga batas waktu masa depan dalam rentang lead .75–.85 s dan
memilih skor kemajuan dengan stop yang layak. Jika worker terlambat/gagal,
lintasan lama yang berakhir berhenti dipertahankan. Ini bukan optimisasi
switching time kontinu bersama lintasan seperti paper SUPER.

Kebijakan FOV pengguna dipertahankan: unknown bawah -7° diasumsikan aman;
unknown atas +52° mengikuti profil batas dan ceiling virtual; measured
occupied tetap obstacle; asumsi tidak menyegarkan timestamp dan tidak
menonaktifkan health gate saat dropout. Backup yang menggunakan prior FOV
diberi label `conditional_on_FOV_assumptions=true`. Jangan menyebutnya
jaminan known-free terukur seperti interpretasi ideal paper SUPER.

## Hasil evaluasi offline

Perintah reproduksi tanpa Gazebo/SITL:

```bash
cd /home/abin/polinasi_lidar_fixed
python3 tools/generate_fixed_lidar.py
source tools/ros_environment.sh
/usr/bin/python3 tools/evaluate_super_adaptation.py
```

Output `reports/super_adaptation_evaluation.json`. Pada grid sintetis bebas,
batas dan uncertainty sama untuk kedua metode: ruas lurus 4 m 21.4918 →
15.0202 s; rute dua belokan 32.2378 → 23.9330 s; detour vertikal yang sudah
ditentukan 32.2378 → 25.5071 s. Pengurangan reference 20.9–30.1%, bukan waktu
misi atau hasil flight. Ini juga bukan bukti pencarian detour dari obstacle:
path benchmark sudah diberikan, sedangkan obstacle/unknown diuji terpisah.
Box corridor memerlukan sekitar 1.5–2.4 ms pada host uji; angka ini bukan
perbandingan runtime terhadap CIRI atau benchmark Jetson.

CIRI bekerja dari point obstacles dan model bola. Mengganti voxel safety
memerlukan pemodelan unknown, prior FOV, extrinsic, visibility dan envelope
badan kita secara eksplisit. Karena hal tersebut belum memiliki sertifikat,
voxel backend dipertahankan. Convex box di atas voxel inflated tidak mengubah
occupied/free/unknown dan selalu dapat fallback ke recursive curve checker.

Model MPC yang tersedia gagal `adaptive seed envelope mismatch: command_speed`
ketika diperiksa terhadap preset fixed saat ini (.4 vs envelope identifikasi
.3). Tidak mengubah label envelope agar diterima. PID dengan reference yang
memiliki cadangan diuji lebih dulu; keputusan mengganti controller harus
berdasarkan tracking/latensi flight dan identifikasi ulang yang cocok.

## Sumber primer

- [SUPER, Science Robotics](https://doi.org/10.1126/scirobotics.ado6187)
- [Optimizer backup resmi](https://github.com/hku-mars/SUPER/blob/master/super_planner/src/traj_opt/backup_traj_optimizer_s4.cpp)
- [Kode CIRI resmi](https://github.com/hku-mars/SUPER/blob/master/super_planner/src/super_core/ciri.cpp)
- [MINCO/GCOPTER](https://github.com/ZJU-FAST-Lab/GCOPTER)
- [Status controller asli dan reproduksi pihak lain](https://github.com/hku-mars/SUPER/issues/57)

Tidak menyalin implementasi rotating-lidar atau memasukkan GT ke mode sensor.
Untuk build, launch headless, stop berdasarkan session, dan lokasi cloud/map,
lihat [FIXED_LIDAR.md](FIXED_LIDAR.md). Jangan menjalankan suite tes berat saat
flight mengukur timing.


## Flight pertama dan perbaikan offline berikutnya

Run `reports/fixed_lidar_validation_20261015_211854` memakai 3408675,
headless. Takeoff mencapai sekitar 2 m, kemudian fault `survey_planner_error`
pada t=22.8 s saat menyiapkan APPROACH. Auditor menghentikan run; session.json
menandai stopped=true. Backup eksplisit tercatat valid dan executed dengan
live_rechecked=true. Misi dua pohon/return/landing/disarm belum tercapai.

Rekonstruksi memakai ekspor peta t=22.811 s (observasi terakhir 22.6 s),
bukan snapshot worker persis. Profil lama gagal di ruas terakhir karena
meminta kecepatan maju bahkan pada titik tujuan yang harus berhenti.
Taper terminal memperbaiki reproduksi tersebut; kurva lolos geometri dan
sertifikasi braking .05 s setelah dua upaya retiming. Exception worker asli
tidak menyimpan detail, sehingga kesamaan dengan penyebab run asli masih
inferensi. Sekarang type/message exception worker dicatat. Tidak ada restart
simulasi setelah fault; perbaikan terminal baru diuji offline.


## Riset penolakan braking pada awal SCAN (15 Oktober 2026)

Run c684d4c: approach 20.65→59.40 s = 38.75 s, ASCEND→SCAN pada
70.95 s, survey_planner_error pada 71.35 s. Log worker membuktikan
ValueError: no robust route braking envelope. Belum ada witness titik/ruas
untuk membuktikan occupied, unknown, canopy guard, atau batas dinamis sebagai
penyebab spesifik. Tidak memulai ulang simulasi untuk riset ini.

Temuan kode terkonfirmasi:

- route_speed_limits hanya mencoba faktor 1,.85,.7,.5,.3. Batas scan .3 m/s
  berarti kandidat terendah .09 m/s; penolakan bukan bukti semua speed infeasible.
- Skenario feedback_brake_safe menambahkan uncertainty velocity .03 m/s dan
  PID acceleration .15 m/s² searah gerak. Memperlambat nominal bukan berarti
  stopping envelope menyusut ke nol. Pada nominal tepat nol ada cabang khusus
  stationary; itu bukan bukti transisi dari gerak ke diam aman.
- Dengan implementasi Brake saat ini, reaction .35 s: nominal .09 membutuhkan
  sekitar .132 m searah gerak pada skenario tambahan tersebut; nominal .01
  membutuhkan .04307 m; nominal 1e-6 membutuhkan .03332 m. Ini jarak model
  sebelum inflasi badan/clearance/voxel, bukan stopping distance hardware.
- Pre-profile memakai arah chord setiap ruas dan nominal acceleration nol,
  bukan PVA aktual quintic hasil smoothing. Penolakan chord belum membuktikan
  kurva akhir atau seluruh lintasan alternatif infeasible.
- Exception muncul sebelum loop fallback smoothing dan sertifikasi kurva;
  kegagalan pencarian kandidat biasa berubah menjadi fatal survey_planner_error.

Sumber primer dan implikasi (hasil paper dibedakan dari usulan lokal):

1. SUPER (Science Robotics 2025): exploration trajectory disertai backup pada
   known-free space. Kode BackupTrajOpt mengoptimalkan time allocation,
   waypoint dan split time; endpoint mempunyai turunan nol. Ini mendukung
   optimasi backup/switch, bukan klaim speed minimum selalu aman. Implementasi
   lokal belum joint MINCO; prior FOV pengguna tetap conditional, bukan
   known-free hasil pengukuran. Sumber:
   https://doi.org/10.1126/scirobotics.ado6187
   https://github.com/hku-mars/SUPER/blob/master/super_planner/src/traj_opt/backup_traj_optimizer_s4.cpp
2. TOPP-RA (Pham & Pham, T-RO 2018): backward controllable sets dari kondisi
   akhir, kemudian forward pass memilih kontrol tercepat yang masih mencapai
   kondisi akhir. Menawarkan pemeriksaan kelayakan sepanjang path, bukan
   lima faktor. Jaminan berlaku pada model/constraints/discretization paper;
   vanilla TOPPRA bukan sertifikat langsung untuk jerk dan collision-aware
   braking/PID kita yang bergantung pada state/lingkungan.
   https://arxiv.org/abs/1707.07239
   https://hungpham2511.github.io/toppra/index.html
3. Ruckig (Berscheid & Kroeger, RSS 2021): trajectory generation dari PVA ke
   target PVA dengan batas velocity/acceleration/jerk. Kandidat pembanding
   backup stop, bukan obstacle planner; keluaran multi-axis perlu cek norm
   3D, peta/canopy, PVA dan timing sebelum digunakan.
   https://arxiv.org/abs/2105.04830
   https://docs.ruckig.com/md_pages_2example__01.html
4. TOPPQuad (Mao et al.): retiming quadrotor dengan dinamika rigid-body dan
   batas thrust masing-masing motor. Kandidat jangka lanjut setelah parameter
   massa/inersia/motor/controller sesuai; bukan perbaikan pertama fault ini.
   https://arxiv.org/abs/2309.11637

Rekomendasi rekayasa lokal, BELUM diimplementasikan/diuji flight:

1. Simpan segment/sample, waktu/PVA, speed percobaan, alasan dan witness voxel,
   source occupied/unknown/inflation/guard pada kegagalan. Replay snapshot yang
   benar-benar dipakai worker, bukan hanya ekspor peta sesudahnya.
2. Pisahkan geometry/body obstruction dari dynamic braking failure. Bangun
   kandidat kurva PVA; retime dan sertifikasi kurva sebenarnya. Pre-profile
   chord tidak boleh menjadi satu-satunya bukti infeasibility global.
3. Pencarian speed/durasi adaptif bounded, mencakup nilai di bawah .09 m/s,
   dengan PVA awal dan terminal dipertahankan. Tidak mengasumsikan safe(speed)
   selalu monoton untuk semua state/kurva; bisection hanya pada bracket yang
   mendukung asumsi tersebut, selainnya bounded refinement. Tidak memaksa
   speed nol saat masih bergerak atau menghapus uncertainty agar lolos.
4. Jika kandidat gagal, pertahankan prefix/backup yang masih lolos live map
   dan freshness. Hold hanya bila stationary envelope aman; replan lokal
   atau detour 3D yang tersertifikasi dalam batas misi. Jika tidak ada backup
   aman, atau sensor dropout/bahaya aktual, fail/abort tetap tersedia.
   Retry punya budget waktu/jumlah dan timeout lama tidak dilonggarkan.
5. Regresi: feasible hanya di bawah .09; stationary-safe tetapi semua gerak
   tidak layak; corner/chord berbeda; unknown/occupied/canopy; moving boundary
   acceleration; dropout; planner deadline; invalidasi backup. Jangan
   menganggap penurunan speed monoton atau slew limiter sebagai bukti aman.

Kriteria memilih solusi: durasi misi simulasi dan waktu CPU dicatat terpisah,
jumlah hold/replan, tracking error, clearance, backup validity dan alasan
penolakan. RTF .15 bukan speed fisik drone. Perbandingan approach 38.75 vs
51.20 s lama hanya indikatif karena tree ID/route berbeda.


## Implementasi iterasi pemulihan planner

Setelah otorisasi loop, rekomendasi awal diimplementasikan: faktor speed
ditambah sampai .01 cap; kegagalan chord membawa witness dan dapat berlanjut
ke sertifikasi kurva PVA; gagal sertifikasi menjadi hold/retry dengan timeout
lama dan stop check, bukan langsung exception worker fatal. Full braking
memakai canopy guard dan mencatat PVA/titik/subreason. Exact snapshot numerik
worker dibatasi delapan per run pada map_3d/planner_failures/*.npz (baca dengan
numpy.load(..., allow_pickle=False)); metadata JSON disimpan dalam field
metadata. Snapshot tidak memakai ground truth. Build dan 409 tes lulus;
hasil flight berikut dicatat di jurnal. Belum TOPP-RA/Ruckig, belum pencarian
durasi kontinu baru, dan belum jaminan semua kegagalan scanning selesai.


## Retiming braking lokal — 16 Oktober 2026

Flight9b63761 berhasil memulai SCAN tetapi global retiming memperpanjang
42.8142→164.4750 s. Ditambahkan usulan retiming ruas gagal dan tetangganya,
dengan PVA knot konsisten, bounds dan geometry diperiksa lagi. Whole-curve
braking tetap wajib; moving boundary memakai fallback C2 lama. Cache braking
per-segmen hanya dipakai untuk coefficients/duration dan grid/config identik;
hash mask juga menginvalidasi perubahan tanpa version bump. Snapshot awal
sebelum retiming kini tersimpan. 412 tes lulus; belum flight saat implementasi
ini ditulis. Detail run/perbaikan ada pada jurnal. Ini usulan engineering
berdasarkan prinsip alokasi waktu lokal, bukan klaim implementasi TOPP-RA.


## Endpoint prefix harus menyisakan ruang berhenti

Run301b206 mengungkap terminal statically safe tetapi robust stop keluar
window. Guidance kini memilih prefix endpoint yang lolos moving-side
near-zero uncertainty scenario; ini bukan commanded minimum speed dan
bukan sertifikat seluruh kurva. Pemeriksaan terminal awal menghindari
retiming panjang pada endpoint infeasible. Timeout lama tetap berjalan
saat worker pending. Prinsip backup feasible didukung
[FASTER](https://arxiv.org/abs/2001.04420); implementasi lokal ini bukan port
FASTER dan prior FOV tetap conditional. Replay snapshot exact berhasil
setelah endpoint mundur sekitar10 cm; flight berikut belum tervalidasi.


## Cadangan perencanaan terhadap pembaruan peta

Run57c105d gagal pada PID final backup saat headroom hasil pengamatan baru
tidak lagi memenuhi guard, meskipun body brake lolos. Fixed-LiDAR feedback
mode kini menyisakan satu voxel tambahan di tahap planning ascent/scan;
execution observation guard, clearance, health dan unknown tetap sama.
Area clear tetap menarget stem-top geometry karena kolom dicari sampai
target+reserve; pada roof, preferred height diturunkan dalam observed corridor.
Ini margin engineering, bukan TEB formal [FaSTrack](https://arxiv.org/abs/2102.07039)
atau jaminan terhadap perubahan lingkungan. Critical PID snapshot dan witness
guard diperluas untuk audit. Detail angka/replay dan hasil tes ada di jurnal.


## Status flight dan transisi return

Run879d89e menyelesaikan dua pohon (12 sektor valid) tetapi gagal saat return
karena metadata anchor scan dipakai pada densifikasi route baru. Lifecycle
metadata diperbaiki per-route/per-phase dan regresi async/PID diperpanjang
sampai home/DESCEND. Ini belum berarti full flight landing/disarm berhasil;
run lengkap berikut tetap diperlukan. Tidak mengubah kebijakan keselamatan.
