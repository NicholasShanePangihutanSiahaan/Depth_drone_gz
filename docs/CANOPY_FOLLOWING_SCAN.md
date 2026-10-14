# Scan mengikuti ruang bawah kanopi

Fixture sensor rotating mengaktifkan `scan_adaptive_height`. Sasaran kamera
adalah `observed_stem_top` dari deteksi batang, bukan deteksi bunga dan bukan
koordinat ground truth. Tinggi kendaraan memperhitungkan offset Z mount
kamera (aproksimasi level-body); arah kamera memakai rotasi badan/mount pada
bridge yang sudah ada. Batas tinggi prototype 5,5 m tetap berlaku.

XY anchor tetap lingkaran. Tiap chunk enam anchor direncanakan ulang pada
snapshot peta terbaru; ada kolom teramati bebas dari bawah hingga kandidat
tinggi + buffer kanopi. Occupied/unknown memutus kolom, sehingga tidak bisa
melompat menembus daun ke udara bebas di atasnya. Cap tiap anchor digabung
dengan batas kemiringan 0,45 dan hysteresis 0,08 m. Backward pass menurunkan
profil sebelum atap yang lebih rendah. Bila ruang tersedia di bagian
berikutnya, profil boleh naik kembali menuju tinggi perkiraan pucuk batang.
Kandidat radial tetap fallback terbatas, bukan penghapusan sektor.

Kurva quintic 3D dan timing bersama menghasilkan gerakan mendatar + vertikal
bersamaan. Tidak ada penambahan Z setelah smoothing atau publisher baru.
Seluruh curve, buffer vertikal, limit v/a/j, braking dan PID final diperiksa.
Buffer juga masuk planner-only blocked mask untuk A* dan smoothing; raw map
dan actual-body mask tidak diubah. Kolom tinggi memilih chord langsung yang
aman bila tersedia, bukan memaksakan endpoint setinggi mungkin di tepi daun.
Buffer vertikal diperiksa dengan curve salinan yang digeser maksimum setengah
voxel per langkah: setiap interval blocked Z (minimal satu voxel) tersampel;
ini bukan downsampling sensor atau pelabelan ulang unknown.

Saat bergerak, map live tetap memeriksa suffix kurva dan buffer pada setiap
versi map baru, actual-body dan stopping pada tick/setpoint. Jika validasi
gagal, rem dahulu dan replan di keadaan diam. Headroom baru dimanfaatkan pada
chunk berikutnya, BUKAN modifikasi vertikal instan di tengah chunk. Boundary
chunk masih stop-to-stop; belum moving replanning/zero-stop orbit. Enam
sektor bukan enam perintah hover, tetapi masih mungkin ada jeda chunk atau
braking keselamatan. Belum validasi hardware atau kemampuan mengenali bunga.

Landasan: pisahkan geometri, kelayakan dinamika, dan replanning, seperti
[Fast-Planner, Zhou et al.](https://github.com/HKUST-Aerial-Robotics/Fast-Planner).
Implementasi ini bukan port Fast-Planner/B-spline/MPC: envelope lokal +
A* dan quintic bersertifikat yang sudah dipakai proyek.

Audit: `scan_height_profile` mencatat cap observed, target batang/kamera,
anchor, tinggi pilihan, map/stamp dan kebijakan update. Rekaman sektor
menambahkan vertical speed, observed stem top dan flag adaptive. Gap/clearance
tetap berdasarkan peta sensor dan bukan ground truth clearance geometris.

Tes: `PYTHONPATH=polinasi_nav python3 -m pytest polinasi_nav/test/test_canopy_scan.py -q`.
Run: `bash tools/run_sensor_localisation_validation.sh inspection` (headless).
