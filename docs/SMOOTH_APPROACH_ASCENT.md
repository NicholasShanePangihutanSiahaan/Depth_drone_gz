# Transisi approach ke ascent

Opt-in pada fixture rotating sensor inspection: `smooth_approach_ascent`.
Bagian akhir approach level yang direct dan dekat target diganti kurva
quintic Bezier, bukan garis maju lalu berhenti di sudut 90 derajat.
Awal kurva memiliki tangent horizontal, akhir tangent vertikal dengan
kecepatan naik bukan nol. Posisi, kecepatan dan percepatan diteruskan ke
ContinuousAscent (kontinuitas C2). Tick yang melintasi batas waktu kurva
tetap mengintegrasikan posisi vertikal, bukan membekukan posisi sambil
mengirim kecepatan naik.

Kurva hanya diterima bila:

- ada permukaan overhead terukur dekat target; bukan tebakan tinggi pohon;
- tetap di luar half-plane radius stand-off dan di bawah cap kanopi;
- seluruh kurva serta buffer pengamatan berada pada peta observed-free;
- batas kecepatan, percepatan, jerk dan pengereman terpenuhi;
- map live, actual-body braking, nominal-reference braking dan output PID
  tetap lolos pemeriksaan saat dieksekusi.

Pengaturan awal: jarak leg akhir maksimal 1,8 m, kenaikan sampai 0,6 m,
kecepatan vertikal terminal 0,08 m/s. Bila kandidat 0,6 m tidak aman, coba
0,4/0,2 m. Jika tetap tidak aman, gunakan approach/ascent aman sebelumnya
dan catat alasan penolakan; tidak memaksa rounding melalui daun/unknown.
Ini tidak menghilangkan stop antar chunk lain atau jeda sensor/perencanaan.

Log `/navigation/status`: `approach_ascent_transition` memuat cap kanopi,
kandidat diterima/ditolak, durasi, batas turunan, dan kondisi handoff.
Algoritma mengikuti kontrol-hull Bezier dan sertifikasi trajectory yang
sudah digunakan repository; bukan implementasi NMPC baru.

Urutan validasi yang diminta pengguna:

1. Headless, map lama `palm_farm`, sensor localisation, dua pohon dan landing.
2. Setelah COMPLETE/disarmed serta 6 sektor valid untuk tiap pohon,
   jalankan map `palm_farm_flowers`, tanpa mengubah map lama.

Belum boleh mengklaim langkah kedua lulus dari hasil langkah pertama.
