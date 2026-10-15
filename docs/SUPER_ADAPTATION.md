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
pre-profile, bukan sertifikat seluruh kurva. Batas kurvatur dan propagasi
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
