# Hasil pemetaan koridor — 10 Oktober 2026

Implementasi: voxel di koridor rute + pengamatan dekat 360°, perluasan lokal
di sekitar obstacle, pemotongan sinar sebelum sampling. Pantulan di luar ROI
tetap memberi bukti kosong pada bagian sinar di dalam ROI. Tidak ada clearing
setelah pantulan, pada data invalid, atau saat dropout. Point cloud permukaan
tetap lengkap; peta voxel bukan lagi seluruh volume jangkauan sensor.

## Loop perbaikan

1. `corridor_run1`: dua EXPLORE pada target pertama. Log menunjukkan voxel
   rendah belum diketahui, bukan obstacle. ROI sebelum takeoff hanya mencakup
   sekitar launch/home, sehingga bukti pada koridor survei awal terbuang.
2. Perbaikan: masukkan lintasan naik dan koridor tujuan survei sejak preflight.
   Sudut bawah MID-360 terbatas; bukti rendah perlu dipertahankan selama naik.
3. `corridor_run2`: takeoff, empat tujuan asli survei, RETURN bertahap,
   LAND, COMPLETE, disarmed. Tidak ada EXPLORE atau failure.

Pemeriksaan keselarasan MPC juga membandingkan posisi pada waktu pengukuran
yang sama (interpolasi; ekstrapolasi ke depan maksimal 30 ms). Transformasi
numerik tetap dikalibrasi sekali saat diam/disarmed; ambang galat MPC 1 cm
tidak dinaikkan dan drift tidak diserap dengan menggeser frame.

## Bukti penerbangan normal

- Posisi: **ground_truth**, bukan lokalisasi LiDAR yang tervalidasi.
- Tujuan: 4/4; jarak keluar nominal 7,5 m; kembali dan landing selesai.
- Kecepatan perintah maksimum: 0,3000 m/s.
- Galat navigasi maksimum dari status: 0,01086 m, fase SURVEY/EXPLORE/RETURN.
- Kecepatan tidak dipenalti oleh pengereman pada uji ini; 165 sampel log
  menunjukkan penalti lama 25% dihilangkan dengan stopping corridor aman.
- Sampel yang dibuat rata-rata 16,92% dari jumlah penelusuran penuh per scan
  pada log yang disampling: pengurangan sekitar 83%, bukan klaim runtime 83%.
- Satu publisher final MAVROS, landing terlihat, motor mati.
- Clearance terhadap voxel occupied yang tersimpan minimum 6,40 m. **Angka
  ini bukan clearance terhadap semua geometri dunia**, karena objek di luar
  ROI mungkin tidak di-voxel-kan. Unknown tetap diperiksa terpisah oleh
  pemeriksaan tubuh, lintasan, dan pengereman; jangan memakai angka ini untuk
  menaikkan kecepatan atau mengklaim clearance fisik seluruh kebun.

Bukti: [laporan penerbangan](corridor_run2_flight.json),
[audit keputusan](corridor_run2_audit.json),
[peta 3D](ros_ground_truth_palm_farm_20261010_155247_23669_mapping/map_3d/map.html).

## Dropout yang disengaja

Simulator/world/config sama. Setelah setup lulus, node simulator diberi
`drop_after=25.0` sebelum start: dropout dihitung dari pesan start misi.
LiDAR terakhir pada detik simulasi 51,8; BRAKE pada 52,3; HOLD_ABORT pada 53,7
dengan alasan `stale_lidar`. Misi memang tidak selesai dan tidak meminta LAND.
Perintah kecepatan akhir mendekati nol. Rentang perubahan posisi terukur pada
83 sampel setelah detik 54: X 2,8 mm, Y 7,6 mm, Z 0,3 mm.
Waktu pengamatan peta dan waktu voxel terbaru tetap 51,8: sensor yang hilang
tidak menghasilkan voxel free baru. Simulator dihentikan setelah pemeriksaan.

Bukti: [laporan dropout](corridor_dropout_flight.json),
[audit dropout](corridor_dropout_audit.json).

## Pengujian dan batas

- Build tiga paket berhasil; **185 tes lulus**, nol error/failure/skipped.
- Peta uji teramati: ekspansi membuka detour; dinding penuh tetap ditolak.
  Ini tes planner/ROI, bukan penerbangan ArduPilot melalui obstacle.
- Tes decoder/dropout, timestamp, bentuk ROI, kesetaraan sampling, interval
  terpisah, obstacle sticky, clipping, dan ROI preflight lulus.
- Benchmark sintetis 5 pengulangan: median whole-range 145,8 ms; clipped ROI
  52,9 ms. Sampel 432.123 menjadi 61.578. Beban laptop saat tes memengaruhi
  angka; bukan pengukuran seluruh stack, hardware Jetson, atau lokalisasi.
  [Data benchmark](corridor_benchmark.json).
- Rute penuh kebun 326 tujuan, detour berobstacle dengan fisika terbang, dan
  lokalisasi berbasis sensor tetap belum divalidasi oleh pengujian ini.
- TTL panjang pada konfigurasi ini hanya untuk kebun simulasi statis. Ini
  tidak membuktikan keamanan foliage bergerak atau penggunaan hardware.

Semua simulasi milik pengujian ini telah dihentikan. Tidak ada publikasi remote
atau operasi perangkat keras. Panduan: [koridor](../docs/CORRIDOR_MAPPING.md),
[perintah simulasi](../docs/SURVEI_KEBUN.md).
