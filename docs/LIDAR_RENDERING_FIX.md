# Perbaikan voxel palsu saat LiDAR tilt

## Penyebab dan perubahan

Laptop memakai `libgz-rendering8` / Ogre2 8.2.3. Material GPU LiDAR yang
terpasang tidak mengatur clamp pada tujuh sampler pass kedua. Pada batas
gambar cubemap, UV=1 dapat melingkar ke sisi lain dan mengambil jarak yang
salah. Itu menjelaskan pola pada sinar sekitar azimut 45°, terutama saat tilt.
Dasar perbaikan: [gz-rendering PR #1319](https://github.com/gazebosim/gz-rendering/pull/1319),
merged 17 Agustus 2026, serta uji endpoint upstream. Sumber itu menjadi hipotesis;
hasil replay lokal sebelum/sesudah diperlukan untuk mengonfirmasi kasus ini.

`tools/prepare_lidar_rendering.py` menyalin resource Gazebo ke
`.dependencies/<run>/rendering_resources`, lalu memasang `tex_address_mode clamp`
untuk sample-table dan keenam sisi cubemap. Launcher mengatur
`GZ_RENDERING_RESOURCE_PATH` hanya untuk proses simulasinya. `/usr/share`, library
Gazebo, source ROS asli, koordinat, susunan/ring sinar, FOV, resolusi voxel,
collision dan fisika tidak diubah. Tidak ada sinar/occupied sektor 45° yang dihapus.

Metadata `lidar_sampler_fix.json` menyimpan hash sebelum/sesudah dan sumber patch.
Tes menolak struktur sampler yang tidak dikenali. Jika instalasi sudah memiliki
clamp, perubahan idempotent. Untuk baseline diagnostik tanpa patch saja, gunakan
`POLINASI_LIDAR_RENDER_FIX=0`; jangan gunakan baseline rusak untuk flight.

## Reproduksi

```bash
cd /home/abin/polinasi_lidar
source tools/ros_environment.sh
/usr/bin/python3 tools/generate_empty_mount_configs.py
bash tools/start_empty_mount_test.sh hidden
```

Terminal kedua (drone disarmed, **jangan** mulai mission):

```bash
source tools/ros_environment.sh
/usr/bin/python3 tools/check_empty_mounts.py --config polinasi_nav/config/empty_mount_hidden.json --output reports/empty_mount_hidden_sampler_fixed.json
/usr/bin/python3 tools/stop_simulation.py --run RUN
```

Ulangi dengan `visible` / `empty_mount_visible_sampler_fixed.json`.
RViz dan viewer kamera dibuka, Gazebo GUI tidak dibuka; server/fisika tetap aktif.
Lantai bukan voxel palsu. Uji menghitung voxel non-lantai (centre z>0,15 m), titik
non-lantai (z>0,1 m), dekat dan jauh dari badan, selama transisi serta dwell.

Validasi obstacle nyata, terpisah dari arena kosong:

```bash
bash tools/start_empty_mount_test.sh fixture
# Terminal kedua:
source tools/ros_environment.sh
/usr/bin/python3 tools/check_empty_mounts.py --config polinasi_nav/config/empty_mount_fixture.json --output reports/lidar_sampler_fixture.json --lidar-only
```

Fixture menambahkan kubus static 1 m pada [3,4,3] m hanya di salinan world privat.
Posisi diketahui hanya untuk audit akurasi, bukan masukan navigasi. Uji memeriksa
titik permukaan dan voxel occupied pada kubus saat tilt; nol semua hit bukan lulus.

## Hasil replay 10 Oktober 2026

Kedua replay memakai urutan mount yang sama dengan baseline masing-masing;
jumlah scan berbeda karena pengukuran berdasarkan waktu simulasi.

| Uji | Scan sesudah patch | Titik palsu lolos self-filter | Voxel palsu non-lantai |
| --- | ---: | ---: | ---: |
| Visual badan normal | 167 | 0 (sebelumnya 274 pengamatan) | 0 |
| Visual badan disembunyikan | 130 | 0 (sebelumnya 205 pengamatan) | 0 |

Lantai tetap terbaca. Dengan visual badan normal, kubus uji menghasilkan 5022
pengamatan titik permukaan dari 50 scan dan 90 voxel occupied. Audit seluruh
map fixture menemukan 90 voxel kubus, 5342 voxel lantai dan **0 voxel non-lantai
di luar kubus**. Jadi perbaikan tidak sekadar menghapus seluruh obstacle.
Default konfigurasi tilt sekarang kembali memakai visual badan normal.

Semua 221 tes paket lulus; 48 tes terarah LiDAR/ray/voxel dan dua tes patch
sampler juga lulus. Ringkasan terukur: `reports/lidar_sampler_validation.json`.
Nilai baseline yang belum diukur ditulis `null`, bukan nol.

Untuk membuat ulang ringkasan setelah menjalankan ketiga replay:

```bash
/usr/bin/python3 tools/report_lidar_validation.py --fixture-map reports/RUN_FIXTURE/map_3d/map.json
```

Ganti `RUN_FIXTURE` dengan nama run fixture yang dicetak launcher.

## Batas

Perbaikan ini untuk generator sensor Gazebo, bukan driver Livox nyata atau validasi
lokalisasi LiDAR–IMU. Kamera pitch masih mempunyai error transient tersendiri,
tidak diselesaikan dalam audit LiDAR. Flight penuh/inspeksi seluruh kebun harus
divalidasi terpisah. Semua replay ini memakai ground truth dan drone disarmed;
tidak membuktikan lokalisasi LiDAR, keselamatan terbang, atau sensor hardware.
