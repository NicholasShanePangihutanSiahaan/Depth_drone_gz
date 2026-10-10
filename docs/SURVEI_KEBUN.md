# Survei kebun dan peta 3D

Peta tersimpan kini mencakup batas kebun 92 × 82 m. Voxel adalah sel 3D kecil;
ukurannya 20 cm. Peta navigasi 10 × 10 × 8 m bergerak bersama drone, sedangkan
hasil pengamatan sebelumnya tetap disimpan dalam peta global.
Batas besar bukan berarti semua area sudah dipindai: area belum terlihat tetap
unknown (belum diketahui), bukan dianggap kosong.

## Menjalankan

Hentikan simulasi lama terlebih dahulu. Jika kode berubah, bangun ulang:

```bash
cd /home/abin/polinasi_lidar
bash tools/build_ros.sh
```

Terminal 1, survei kebun:

```bash
cd /home/abin/polinasi_lidar
bash tools/start_mapping.sh tour ground_truth
```

Untuk memakai **MPC aktif** pada rute zigzag yang sama, gunakan launcher ini
sebagai pengganti perintah di atas (jangan menjalankan keduanya):

```bash
bash tools/start_predictive.sh tour
```

Langkah terminal berikutnya tetap sama. MPC menggunakan model respons hasil
identifikasi; adaptasi online nonaktif dan batas kecepatan tetap 0,3 m/s.
Ini mode ground-truth simulasi. Penerbangan penuh kebun dengan MPC belum
divalidasi; lihat [pengendali prediktif](PREDICTIVE_CONTROL.md).

Uji awal empat tujuan asli survei, lalu kembali dan landing, tersedia dengan
`bash tools/start_predictive.sh tour_check`. Ini bukan survei seluruh kebun.
Penanganan sinar Gazebo tanpa pantulan, pengurangan pemindaian berulang,
kecepatan berbasis pengereman, dan log debug dijelaskan pada
[audit keputusan navigasi](NAVIGATION_DECISIONS.md).
Pemetaan kini dapat memotong sinar ke [koridor adaptif](CORRIDOR_MAPPING.md)
sebelum membuat sampel voxel. Point cloud log tetap lengkap, voxel di luar
wilayah yang diproses tidak otomatis dibuat atau dianggap kosong.

Launcher mapping kembali meminta faktor waktu **0,2**:
target 1 detik simulasi membutuhkan sekitar 5 detik nyata.
Faktor 0,3 lulus loop kecil tetapi gagal pada run `tour`; karena itu bukan lagi
bawaan. Planner dan pembuatan pesan visual kini dipisahkan dari loop kontrol;
lihat [perbaikan runtime](RUNTIME_ISOLATION.md). Beban komputer tetap membatasi
kemampuan aktual; jangan melonggarkan pengamanan jika ada keterlambatan.
Jangan gunakan `smoke` jika ingin survei kebun; rutenya hanya kotak sisi 25 cm.

Jika muncul sensor basi atau `timing_gap`, hentikan run dan ulangi secara lebih
konservatif tanpa mematikan pengamanan:

```bash
POLINASI_SIM_RTF=0.2 bash tools/start_mapping.sh tour ground_truth
```

Ganti `tour` dengan `check` untuk uji pergi 4 m lalu kembali, atau `smoke`
untuk loop sangat kecil. `tour` memakai 326 target pendek dalam jalur zigzag,
sekitar 569 m perjalanan nominal. Area survei mengikuti batas luar sebaran
29 pohon ditambah margin tepi 4,5 m, bukan seluruh lantai kosong. Posisi pohon
berasal dari konfigurasi dunia simulasi, bukan deteksi otomatis; semua jalur
terbang tetap harus lolos pemeriksaan peta LiDAR.

Mapping kini memakai [lintasan kontinu dan kecepatan adaptif](LINTASAN_KONTINU.md).
Titik tengah yang aman bisa dilewati tanpa berhenti; ujung kelompok lintasan
tetap berhenti untuk perencanaan ulang yang aman. Batas perintah `check`/`tour`
0,3 m/s, `smoke` 0,2 m/s; kecepatan sebenarnya dapat lebih rendah.

Terminal 2, konfirmasi titik awal dan kesiapan sebelum memulai:

```bash
cd /home/abin/polinasi_lidar
source tools/ros_environment.sh
.venv/bin/python tools/set_origin.py --timeout 120
python3 tools/check_ros_simulation.py --output reports/mapping_setup.json
```

Lanjutkan hanya jika pemeriksaan `passed: true`. Jangan memulai misi jika
pemeriksaan gagal. Jika terminal 1 melaporkan port masih dipakai, hentikan
simulasi lama dahulu; jangan jalankan dua instance bersamaan.

Terminal 3, tampilan Gazebo:

```bash
cd /home/abin/polinasi_lidar
bash tools/view_simulation.sh
```

Atau RViz untuk point cloud (titik hasil LiDAR), voxel, rencana, dan jalur aktual:

```bash
source tools/ros_environment.sh
rviz2 -d polinasi_nav/config/navigation.rviz --ros-args -p use_sim_time:=true
```

Terminal lain, tampilkan perubahan state dan simpan laporan penerbangan,
**sebelum** memulai misi:

```bash
source tools/ros_environment.sh
python3 tools/watch_mapping.py --seconds 36000 --output reports/mapping_tour_live.json
```

Kembali ke terminal 2 untuk memulai:

```bash
python3 tools/start_mission.py
```

Pantau `mission`, `survey_waypoint`, `failure`, dan `health_reason` di terminal
pengamat. `SURVEY` adalah survei, `EXPLORE` mencari posisi pengamatan tambahan,
`RETURN` kembali, dan `LAND` mendarat. Jumlah target `tour` adalah 326, bukan 4.
Jika ingin status lengkap, gunakan terminal tambahan:

```bash
source tools/ros_environment.sh
ros2 topic echo /navigation/status --field data
```

Tunggu `mission: COMPLETE` dan `armed: false`, kemudian Ctrl+C di terminal 1.
Lokasi `reports/..._mapping/map_3d/map.html` dicetak saat mulai. Buka file itu
di browser untuk memutar/zoom peta; tidak perlu rekaman video atau rosbag.
`map.json` menyimpan titik, voxel, waktu, dan status. Refresh halaman untuk
melihat snapshot terbaru. Snapshot diperbarui tiap 10 detik simulasi.

Pada log baru tersedia checkbox **rute rencana** (kuning), **lintasan perintah
terakhir** (biru), dan **jalur aktual** (hijau). Centang untuk menampilkan,
hapus centang untuk menyembunyikan. Rute rencana belum tentu aman; jalur aktual
berasal dari posisi hasil lokalisasi, bukan dari target. Log lama belum menyimpan
jalur aktual, sehingga checkbox itu dinonaktifkan—perlu simulasi baru untuk
menyimpannya. [Viewer log lama yang diperbarui](../reports/ros_ground_truth_palm_farm_20261009_204135_64865_mapping/map_3d/map_paths.html)
mempertahankan data lama dan hanya menambahkan rute dari konfigurasi run tersebut.

## Batas yang penting

- Faktor waktu baru 0,3 sudah lulus uji `smoke`: takeoff, empat titik, kembali,
  landing otomatis dan motor mati; galat maksimum 1,18 cm. Bukti:
  `reports/factor03_smoke_flight.json`. Faktor 0,5 sempat memicu jeda callback
  melewati batas aman dan hold; karena itu tidak dijadikan bawaan.
  Hasil loop kecil ini tidak membuktikan seluruh `tour` akan selesai.

- Uji baru lintasan kontinu `smoke` berhasil lengkap: empat titik, kembali,
  landing otomatis dan motor mati. Tiga titik tengah dilewati tanpa dwell;
  galat mengikuti target maksimum 1,01 cm. Bukti:
  `reports/continuous_smoke_flight.json`.
  [Peta uji baru](../reports/ros_ground_truth_palm_farm_20261009_214919_77507_mapping/map_3d/map.html)
  menyimpan point cloud, voxel, dan jalur aktual. Kecepatan maksimum perintah
  terukur 0,138 m/s; dekat ruang unknown, faktor kecepatan turun sampai 0,25.
  `check`/`tour` baru dengan batas 0,3 m/s belum diuji penerbangan lengkap.

- Loop kecil sudah berhasil takeoff → empat titik → kembali → landing otomatis.
  Tinggi maksimum 2,005 m, galat mengikuti target maksimum 7,7 cm.
  Bukti: `reports/coverage_smoke_slow_flight.json` (sensor simulasi lama 12 m).
- Uji sebelumnya (antarmuka posisi saja) `check` juga berhasil lengkap, mencapai 4 m dari home dan kembali.
  Galat maksimum 9,8 cm; tinggi maksimum 2,305 m termasuk pengamatan tambahan.
  Bukti: `reports/coverage_check_conservative_flight.json`.
  Peta akhirnya menyimpan 268.169 titik dan 63.226 voxel berisi rintangan.
  Buka [peta hasil uji](../reports/ros_ground_truth_palm_farm_20261009_204135_64865_mapping/map_3d/map.html).
- Konfigurasi baru memakai jangkauan LiDAR 20 m agar sudut bawah −7° dapat
  menangkap tanah dari ketinggian sensor sekitar 2,2 m. Sudut tidak dipalsukan.
- Run lama memakai faktor 0,2 (sekitar 5 detik nyata per detik simulasi).
  Launcher mapping kini kembali meminta 0,2; kemampuan komputer membatasi hasil aktual.
  Survei penuh tetap bisa membutuhkan beberapa jam.
  Jalur sekitar 1 km dengan banyak berhenti bisa sangat lama; batas misi
  18.000 detik simulasi tetap dapat menghentikannya jika banyak rute terhalang.
- Drone tetap berhenti jika sensor basi, lokalisasi hilang, atau rute tidak aman.
  Penyimpanan ruang kosong lama hanya berlaku untuk **kebun simulasi statis**,
  bukan izin memakai kebijakan itu pada lingkungan nyata/dinamis.
- Model autopilot memakai satu IMU simulasi. Konfigurasi sekarang menyalakan
  hanya IMU itu dan menghapus kalibrasi bawaan IMU kedua yang tidak tersedia;
  pemeriksaan arming tetap aktif.
- Survei seluruh kebun belum dibuktikan selesai. Daun dapat menutupi bagian
  pohon; jalur zigzag tidak menjamin setiap permukaan terlihat.
- `ground_truth` memakai posisi simulator untuk menguji navigasi. Itu **bukan**
  validasi lokalisasi LiDAR. Mode `sensor` masih belum layak untuk penerbangan.
## Waktu pemrosesan

Rincian log LiDAR → voxel → setpoint serta tahap perencanaan ada di
[PROCESSING_TIME.md](PROCESSING_TIME.md). Nilai baru tersedia pada run berikutnya;
angka lama tidak direkonstruksi atau ditimpa.

