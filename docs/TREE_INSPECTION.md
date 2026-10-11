# Inspeksi pohon dari LiDAR — prototipe simulasi

Misi terpisah: deteksi kandidat batang → takeoff → approach bertahap →
delapan viewpoint menghadap pohon → pohon berikutnya → kembali → landing.
Tidak mengaktifkan bunga atau buzzer. Kontrol memakai PID dan pengecekan
lintasan/braking pada voxel ROI yang sama dengan survei.

Detektor memakai point cloud terukur, bukan koordinat pohon Gazebo. Batang
diperkirakan dari kelompok vertikal dan kecocokan penampang lingkaran.
Kandidat harus teramati berulang sebelum dipilih. Ini belum klasifikasi
semantik: tiang juga dapat terdeteksi sebagai kandidat pohon. Tinggi bagian
batang yang terlihat bukan bukti bahwa pucuk sebenarnya sudah ditemukan.

## Jalankan

Terminal 1:

```bash
cd /home/abin/polinasi_lidar
source tools/ros_environment.sh
colcon --log-base log_ros build --symlink-install --build-base build_ros --install-base install_ros --base-paths polinasi_nav beehive_drone uav_interfaces --packages-select polinasi_nav
bash tools/start_tree_inspection.sh
```

Gazebo dan RViz dibuka bersama. Launcher mencetak direktori run dan peta HTML.
Terminal 2:

```bash
cd /home/abin/polinasi_lidar
source tools/ros_environment.sh
.venv/bin/python tools/set_origin.py --timeout 120
/usr/bin/python3 tools/check_ros_simulation.py --output reports/tree_inspection_setup.json
/usr/bin/python3 tools/start_mission.py
```

Jangan mulai jika setup gagal atau terdapat dua publisher kontrol.
Misi menunggu kandidat pohon terkonfirmasi sebelum takeoff.

Pemantau otomatis, terminal 3 (ganti RUN dengan direktori absolut yang dicetak):

```bash
source tools/ros_environment.sh
/usr/bin/python3 tools/watch_tree_inspection.py --run RUN --output reports/tree_inspection_watchdog.jsonl --seconds 7200
```

Pemantau mencatat pergantian state dan menghentikan run beserta kedua GUI jika
terjadi kegagalan. Untuk menghentikan manual:

```bash
/usr/bin/python3 tools/stop_simulation.py --run RUN
```

## Batas cakupan

- Default ground truth: posisi berasal dari simulator, bukan hasil lokalisasi
  LiDAR. Tidak boleh dilaporkan sebagai validasi lokalisasi LiDAR.
- `tree_demo_limit=0` berarti tidak ada batas jumlah kandidat; gunakan 2 untuk
  pengujian singkat. Run pengujian awal memakai batas dua pohon. Daftar kandidat hanya
  mewakili pohon yang terlihat; belum ada eksplorasi untuk membuktikan semua
  pohon di seluruh kebun sudah selesai. `entire_farm_inspected` selalu false.
- Viewpoint dipilih berdasarkan ruang bebas dan biaya perjalanan. Belum ada
  penilaian mutu gambar/visibilitas bunga; kamera tetap tanpa pitch aktif.
- Radius kandidat 3,5–5,5 m, bukan minimum fisik yang sudah dioptimalkan.
- Voxel unknown tidak dianggap kosong. Tujuan belum teramati dapat memicu
  pemindaian atau hold. Data persepsi basi menyebabkan berhenti aman.
- Real-time factor default 0,15: waktu simulasi berjalan lebih lambat daripada
  jam nyata untuk pengujian beban komputasi; ini bukan klaim kinerja Jetson.
- Log memuat kandidat/ID pohon, fase, viewpoint, waktu detektor, scan terintegrasi,
  alasan berhenti, dan diagnostik kontrol/pemetaan yang sudah tersedia.

Tes unit geometri dan supervisor tidak menggantikan pengujian penerbangan SITL.

Dasar pemilihan pendekatan geometris: [Automatic extraction and measurement
of individual trees from mobile laser scanning point clouds of forests](https://academic.oup.com/aob/article/128/6/787/6316746)
membahas deteksi batang dan fitting penampang. Implementasi ini hanya prototipe
clustering/fitting sederhana, bukan reproduksi metode maupun hasil paper tersebut;
penentuan pucuk dan penolakan daun/tiang masih perlu validasi khusus sawit.
