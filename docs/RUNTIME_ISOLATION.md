# Isolasi pekerjaan berat dari kontrol

Run `223429` dan `225640` berhenti dengan `clock_or_scheduler_gap`, bukan
takeoff gagal. Run kedua mencatat jeda 204 ms saat batasnya 200 ms **waktu
simulasi**. Log lama belum mengukur biaya tiap callback, sehingga tidak cukup
untuk menyebut satu fungsi sebagai penyebab pasti.

## Perubahan

### Pembaruan 10 Oktober: proses ROS khusus pemetaan

Run `040712` masih berhenti pada jeda 222/245 ms. Karena penerapan hasil peta
dan pengumpulan data visual masih berada bersama kontrol, worker terpisah saja
belum cukup. Launch mapping sekarang menjalankan `polinasi_mapping_io` dalam
proses ROS tersendiri. Proses ini memiliki point cloud, riwayat global, penerapan
delta voxel, pencatatan jalur aktual, visualisasi dan ekspor HTML/JSON. Ia **tidak
memiliki publisher setpoint ataupun perintah arm/takeoff/land**.

Kontrol menerima hanya window lokal yang sudah diinflasi melalui
`/mapping/navigation_snapshot` (antrean satu, format array numerik tanpa pickle).
Frame tetap map, origin numerik ikut bergerak bersama window; stamp pengukuran
tetap asli. Snapshot lama/duplikat atau di luar batas ditolak. Perubahan peta
tidak melewati pemeriksaan tabrakan/pengereman pada setiap perintah. Kontrol
tidak menunggu hasil mapper: jika umur peta melampaui 1 s, pengamanan tetap
berlaku. Status membedakan `callback_wall_max_ms` kontrol dari
`mapping_io_callback_wall_max_ms`. Pesan visual dan log tidak lagi diterbitkan
dari proses kontrol. Versi lama di bawah adalah tahap sebelumnya.

Rute `tour` juga dipersempit ke batas luar sebaran pohon dengan margin tepi
4,5 m: 326 target, sekitar 569 m, menggantikan 586 target/1.085 m. Ini wilayah
misi yang dikonfigurasi, bukan bukti semua wilayah telah terlihat/aman.

124 tes regresi lulus setelah perubahan ini, termasuk kepemilikan publisher,
transport window dan penolakan snapshot kedaluwarsa, serta batas footprint
rute dan kedekatan rute dengan setiap pohon. Uji penerbangan dicatat terpisah
di `VALIDATION.md`; ini belum sertifikasi penerbangan Jetson atau LiDAR–IMU.

### Tahap worker sebelumnya (9 Oktober)

- Planner survei memakai proses `spawn`, bukan thread. Perhitungan A*,
  pengamatan celah dan sertifikasi lintasan tidak lagi berbagi pengunci Python
  dengan loop kontrol. Perencanaan tetap saat diam, pada snapshot; hasilnya
  tetap diperiksa terhadap peta terbaru sebelum digunakan.
- Pesan voxel, cloud global, rute dan jalur RViz dibangun serta diserialisasi
  dalam proses visual terpisah. Hanya node navigasi yang menerbitkan hasil.
  Worker tidak membuat node ROS, tidak menerbitkan setpoint, dan tidak mengubah
  peta navigasi. Antrean dibatasi satu pekerjaan lokal dan satu global;
  visualisasi yang sibuk melewatkan pembaruan, bukan menumpuk pekerjaan.
- Status menambahkan `callback_wall_max_ms`, `planner_worker_wall_max_ms`,
  dan `timing_gap_previous_callback`. Saat jeda terpicu, ROS log mencetak
  interval serta ringkasan waktunya. `wall` berarti waktu nyata komputer,
  bukan waktu simulasi. Callback terakhir hanya petunjuk, bukan bukti otomatis
  bahwa callback itu menyebabkan jeda.
- Launcher memakai grup proses privat. Ctrl+C/SIGHUP menghentikan Gazebo,
  SITL, node ROS dan worker milik run tersebut, bukan hanya wrapper launch.
  Preflight menolak node navigasi lama yang masih berjalan. Log dipertahankan.
- Bawaan faktor waktu dikembalikan ke **0,2**. Keberhasilan `smoke` pada 0,3
  sebelumnya tidak membuktikan kestabilan `tour`. Jangan menaikkan faktor
  sebelum beban rute sebenarnya diuji.

Batas jeda 200 ms, umur LiDAR/lokalisasi/peta, galat tracking, pemeriksaan
tabrakan/pengereman, resolusi voxel dan batas gerak tidak dilonggarkan.
Proses terpisah mengurangi pemblokiran Python, tetapi tetap berbagi CPU/RAM;
ini bukan jaminan kontrol waktu-nyata keras pada komputer yang kelebihan beban.

## Pemeriksaan

119 regresi lulus, mencakup planner proses spawn, transformasi kontrol yang tetap benar,
serialisasi/deserialisasi voxel dan lintasan beserta frame/waktu, serta bukti
bahwa interval 204 ms masih memicu pengamanan. Hasil uji awal rute `tour`
dicatat terpisah di [VALIDATION.md](VALIDATION.md), bukan dianggap seluruh
kebun selesai atau lokalisasi LiDAR tervalidasi.

```bash
cd /home/abin/polinasi_lidar
bash tools/build_ros.sh
POLINASI_SIM_RTF=0.2 bash tools/start_mapping.sh tour ground_truth
```

Langkah origin, pemeriksaan PASS, start dan pemantauan:
[SURVEI_KEBUN.md](SURVEI_KEBUN.md).
