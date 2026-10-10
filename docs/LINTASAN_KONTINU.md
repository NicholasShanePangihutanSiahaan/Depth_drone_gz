# Lintasan kontinu untuk misi mapping

Tiga perubahan diterapkan khusus misi mapping; bukan NMPC. Pengendali sikap
dan motor tetap milik ArduPilot. Mekanisme takeoff tidak diganti.

1. **Lintasan tersambung.** A* mencari jalur pada voxel hasil LiDAR. Sampai
   empat target yang sudah aman digabung menjadi kurva polinomial quintic
   (polinomial orde lima). Posisi, kecepatan, dan percepatan tersambung tanpa
   loncatan. Drone bisa melewati titik tengah tanpa berhenti; satu hasil
   integrasi LiDAR baru tetap diperlukan sesudah masuk radius titik tersebut.
2. **Perintah lengkap.** Satu publisher mengirim posisi, kecepatan, dan
   percepatan ke `/mavros/setpoint_raw/local`. Kecepatan/percepatan membantu
   ArduPilot mengikuti gerak rencana, bukan hanya mengejar posisi.
   Publisher posisi lama tidak aktif pada misi mapping. Misi inspeksi tetap
   memakai antarmuka posisi sebelumnya.
3. **Kecepatan adaptif.** Waktu lintasan diperlambat secara halus jika drone
   tertinggal, dekat rintangan/ruang belum diketahui, atau melalui tikungan.
   Turunan posisi dihitung kembali sehingga kecepatan dan percepatan yang
   dikirim benar-benar sesuai lintasan yang sedang dijalankan.

## Pengamanan

Seluruh kurva diperiksa terhadap voxel rintangan yang sudah diperbesar sesuai
dimensi drone dan margin keselamatan. Ruang belum diketahui tetap diblokir.
Pembulatan yang tidak terbukti aman dikurangi atau dibatalkan. Batas
kecepatan, percepatan, dan jerk (laju perubahan percepatan) tetap diperiksa.
Perhitungan lintasan menyediakan cadangan percepatan/jerk untuk perubahan
kecepatan adaptif. Pemeriksaan pengereman tetap berjalan sebelum tiap perintah.

Drone tetap berhenti pada ujung kelompok lintasan, pembalikan arah tajam,
perencanaan ulang saat diam, atau ruang depan belum teramati. Jadi ini mengurangi
berhenti di titik tengah, **bukan** menjamin seluruh survei terbang tanpa berhenti.
Sensor basi, galat mengikuti target, dan kehilangan lokalisasi tetap menghentikan
misi; kecepatan adaptif tidak menggantikan pengamanan tersebut.

Transformasi `map` ke koordinat lokal FCU benar-benar menghitung rotasi dan
translasi posisi. Kecepatan/percepatan hanya dirotasi. Pesan ROS berisi ENU;
MAVROS mengubahnya sekali menjadi NED untuk ArduPilot. Mengganti label frame
saja tidak dipakai sebagai perbaikan koordinat.

## Konfigurasi dan pengujian

`mapping_check.json` dan `mapping_tour.json` membatasi perintah pada 0,3 m/s;
`mapping_smoke.json` tetap 0,2 m/s. Ini batas maksimum, bukan janji kecepatan
rata-rata atau rekomendasi perangkat nyata. Peta navigasi bergeser berukuran
10 × 10 × 8 m; voxel tetap 20 cm dan arsip global tidak dipangkas.

Parameter penting: `continuous_survey`, `survey_batch_points`,
`survey_pass_radius`, `command_speed`, `adaptive_min_scale`,
`adaptive_clearance_slow`, `adaptive_clearance_full`, dan
`adaptive_turn_acceleration`. Generator konfigurasi dapat menimpa perubahan
manual: `python3 tools/generate_mapping_configs.py`.

Regresi meliputi sambungan lintasan, batas turunan, rintangan/unknown di antara
titik, perlambatan adaptif, kehilangan sensor, integrasi LiDAR baru, rute
pergi–pulang dengan koordinat berulang, dan transformasi perintah MAVROS.
Tes dengan peta diketahui dan gerak titik sederhana bukan bukti fisika
ArduPilot atau lokalisasi LiDAR. Hasil penerbangan nyata simulator dicatat
terpisah di [VALIDATION.md](VALIDATION.md).

Cara build dan launch: [SURVEI_KEBUN.md](SURVEI_KEBUN.md). Status navigasi kini
menampilkan `speed_scale`, `adaptive_reason`, `continuous_passes`,
`commanded_speed_mps`, dan `commanded_acceleration_mps2`.
