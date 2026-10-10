# Pemetaan koridor adaptif

Pemetaan navigasi berfokus pada bagian rute yang akan dilalui, bukan seluruh
volume dalam jangkauan LiDAR. Ini mengurangi pekerjaan integrasi voxel;
pembacaan sensor, transformasi awal, dan point cloud log tetap memiliki biaya.

Konfigurasi mapping dan MPC mapping mengaktifkan:

- `mapping_corridor_radius`: 1,5 m dari garis rute, dalam 3D.
- `mapping_near_radius`: 2 m, pengamatan 360° dekat drone.
- `mapping_corridor_lookahead`: 20 m panjang rute, dibatasi jangkauan LiDAR.
- Perluasan di sekitar voxel obstacle yang dilaporkan planner: 1 m per langkah,
  jeda minimal 0,8 detik pengukuran peta, maksimum radius 4 m.

ROI adalah region of interest, atau wilayah yang perlu diproses. Pemilihan
ROI **tidak** menghasilkan ruang kosong. Hanya sinar valid yang benar-benar
melewati wilayah tersebut dapat memperbaruinya. Perluasan tetap menunggu
pengamatan; area yang belum terlihat tetap unknown, termasuk di belakang daun.
Koridor survei pertama sudah diproses sebelum dan selama takeoff. Ini penting
karena cakupan bawah MID-360 hanya -7 derajat: setelah naik, voxel rendah di
bawah tujuan dekat belum tentu dapat terlihat dari posisi hover.

## Pemotongan sinar

Sinar menuju pohon 15 m di luar ROI tetap dapat memberikan bukti ruang kosong
di dekat drone. Program mempertahankan bagian sinar yang berpotongan dengan
ROI, bukan sekadar membuang titik pantulannya. Sampel dibuat hanya dalam
interval perpotongan; interval terpisah tidak dihubungkan sebagai ruang kosong.
Tidak ada sampel setelah pantulan. Sinar tanpa pantulan Gazebo yang valid
tetap dibatasi jarak maksimum konfigurasi, bukan jarak tak hingga.

Kotak pembatas mempercepat pencarian interval; pemeriksaan akhir memakai
bentuk ROI yang sebenarnya (bola dan koridor). Sudut kotak pembatas tidak
otomatis menjadi kosong. Grid keselamatan tetap memakai ukuran drone,
inflasi, unknown, dan pemeriksaan pengereman/lintasan perintah yang sama.

Koridor mengikuti tujuan survei dan lintasan detour yang sedang direncanakan.
Saat RETURN, koridor mengarah ke home. Ruang perluasan yang sudah dipilih tidak
langsung dibuang ketika detour ditemukan. A* dan pemeriksaan lintasan tetap
aktif; jika tidak ada jalur aman, program menahan/mengerem, bukan menembus unknown.

Prinsip membatasi pekerjaan pada informasi relevan lintasan dibahas oleh
[EGO-Planner](https://arxiv.org/abs/2008.08835). Implementasi ini menggunakan
ROI dan A* yang sudah ada, **bukan** implementasi EGO-Planner/optimisasinya.

## Peta dan log

Point cloud permukaan tetap disimpan dari seluruh hit valid. Voxel baru hanya
dibangun pada wilayah yang diproses: karena itu titik di luar ROI dapat terlihat
tanpa voxel yang bersesuaian. Voxel historis tetap disimpan dengan kebijakan TTL
yang sudah dikonfigurasi. Ini bukan peta voxel lengkap seluruh kebun.

`navigation_debug.jsonl` dan `/navigation/status` memuat `mapping_roi`:
wilayah/fokus, radius perluasan, jumlah ekspansi, jumlah sampel jika seluruh
sinar ditelusuri, sampel yang benar-benar dibuat, dan sampel lolos ROI.
Penghematan jumlah sampel bukan jaminan penghematan waktu dengan rasio sama.

```bash
bash tools/build_ros.sh
bash tools/start_predictive.sh tour_check
```

Selanjutnya konfirmasi origin, cek setup, jalankan pengamat, dan mulai misi
sesuai [panduan survei](SURVEI_KEBUN.md). `tour_check` hanya empat tujuan asli
survei dan kembali/landing; `tour` seluruh rute. Keduanya memakai ground-truth
untuk posisi, bukan validasi lokalisasi LiDAR atau keselamatan hardware.

Tes regresi mencakup kesetaraan dengan penyaringan setelah sampling, sinar
paralel, interval terpisah, pantulan di luar ROI, larangan clear di belakang
pantulan, wilayah dekat 360°, dan ekspansi yang dibatasi pengamatan/radius.

Benchmark sintetis tanpa fisika terbang dapat diulang dengan:

```bash
source tools/ros_environment.sh
/usr/bin/python3 tools/benchmark_corridor_mapping.py
```

Hasil disimpan di `reports/corridor_benchmark.json`. Bandingkan waktu median
integrasi dan jumlah sampel; jangan menyamakannya dengan percepatan seluruh
simulasi atau performa Jetson.
