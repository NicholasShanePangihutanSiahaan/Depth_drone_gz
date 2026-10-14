# Transisi approach ke ascent

Opt-in pada fixture rotating sensor inspection: `smooth_approach_ascent`.
Bagian akhir approach level yang direct dan dekat target diganti kurva
quintic Bezier, bukan garis maju lalu berhenti di sudut 90 derajat.
Pada approach level, awal kurva memiliki tangent horizontal; approach yang
masih sedikit menurun dapat melengkung turun-lalu-naik. Akhir tangent vertikal dengan
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

Pengaturan awal: jarak leg akhir maksimal 2 m, offset turun maksimal 0,4 m,
kenaikan sampai 0,6 m dari tinggi sudut approach rendah,
kecepatan vertikal terminal 0,08 m/s. Bila kandidat 0,6 m tidak aman, coba
0,4/0,2 m. Jika tetap tidak aman, gunakan approach/ascent aman sebelumnya
dan catat alasan penolakan; tidak memaksa rounding melalui daun/unknown.
Ini tidak menghilangkan stop antar chunk lain atau jeda sensor/perencanaan.
Seperti rounding lainnya, kurva tidak harus mengunjungi tepat titik sudut
rendah semula. Seluruh geometri tetap diperiksa, bukan hanya endpoint.

Log `/navigation/status`: `approach_ascent_transition` memuat cap kanopi,
kandidat diterima/ditolak, durasi, batas turunan, dan kondisi handoff.
Algoritma mengikuti kontrol-hull Bezier dan sertifikasi trajectory yang
sudah digunakan repository; bukan implementasi NMPC baru.

Urutan validasi yang diminta pengguna:

1. Headless, map lama `palm_farm`, sensor localisation, dua pohon dan landing.
2. Setelah COMPLETE/disarmed serta 6 sektor valid untuk tiap pohon,
   jalankan map `palm_farm_flowers`, tanpa mengubah map lama.

Belum boleh mengklaim langkah kedua lulus dari hasil langkah pertama.

Gate read-only sebelum world kedua:

```bash
python3 tools/check_sensor_mission_completion.py reports/<folder-validasi>/accuracy.summary.json
```

Gate memerlukan sensor-mode, tidak memakai truth untuk estimator, COMPLETE,
disarmed, dua tree ID dan minimal enam sektor unik valid per pohon. Operator
stop, sekadar LAND, view duplikat atau failure tidak dianggap berhasil.
Launcher sensor menerima argumen world kedua, default tetap `palm_farm`;
world bunga baru dijalankan setelah gate map lama lulus.
