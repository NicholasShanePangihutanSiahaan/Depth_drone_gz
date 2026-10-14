# Tujuan tetap, panduan riwayat peta, validasi lokal

Fixture sensor rotating mengaktifkan `history_route_enabled` untuk RETURN
dan APPROACH. Tujuan RETURN adalah [home.x,home.y,takeoff_altitude], bukan
titik geometris 2 m. Tujuan APPROACH tetap pose stand-off pohon dari deteksi
LiDAR; bukan pusat batang atau koordinat pohon simulator.

Navigation memilih tujuan, lalu mengirim start/reference dan tujuan melalui
`/navigation/status.history_route_request`. Mapping IO menyalin riwayat
SparseHistory (hanya memori pengukuran/prior pad terdeklarasi) ke worker
terpisah. Worker membuat peta .2 m pada kotak start–goal + margin XY 4 m,
Z 1.5 m, maksimum 800000 voxel. Semua unknown dan free kedaluwarsa blocked;
ukuran kendaraan, clearance, tracking margin sama dengan navigasi lokal.
Transfer tidak naik di atas endpoint yang lebih tinggi (toleransi voxel).

A* 26 tetangga dengan pemeriksaan swept-edge/corner mencari jalur menuju
tujuan akhir. Komponen observed-free dipakai untuk membedakan jalur penuh
dari tujuan belum dapat dijangkau. Bila baru sebagian diketahui, jalur
parsial berakhir pada bagian komponen terjangkau yang paling dekat tujuan.
`partial=true` bukan mission success. Tidak ada kemajuan atau budget terlampaui
berarti hold dan timeout, bukan memaksa terbang ke unknown. Ini bounded global
guidance, bukan pencarian tanpa batas di seluruh farm atau full exploration
planner: jika terhenti di frontier yang perlu sudut pandang baru, automatic
exploration lengkap belum disambungkan pada gate ini.

Hasil `/mapping/history_route` hanya panduan, bukan setpoint. Navigation
mengambil prefix maksimal 2 m SEPANJANG JALUR, bukan sepanjang garis ke home.
Endpoint prefix wajib aman pada rolling map terbaru. A* lokal, quintic,
braking, actual-body dan PID final tetap memeriksa lintasan; obstacle baru
memicu rem sebelum replan. Umur panduan maksimal 8 s simulasi, pembaruan
worker tiap 2 s, tunggu tanpa rute maksimal 30 s. Goal mismatch, stale path,
jarak dari jalur >.75 m, dan jalur lokal blocked ditolak. Tepat satu publisher
MAVROS tetap; worker riwayat tidak mempunyai ROS node/publisher flight.

Asynchronous: proses kontrol tidak mengirim peta global besar atau menjalankan
A* global. Mapping IO masih melakukan shallow-copy dictionary history ketika
submit (biaya ini bukan nol); worker restore/inflate/search terpisah. Status
`history_route` mencatat jalur, tujuan, partial, request id, map/stamp, jumlah
ekspansi dan wall time; `history_route_diagnostics` mencatat pemilihan prefix
atau alasan menunggu. Jalur nominal bukan bukti bebas obstacle.

Referensi arsitektur hierarkis: [FUEL, Zhou et al., RA-L 2021](https://github.com/HKUST-Aerial-Robotics/FUEL).
Implementasi ini bukan FUEL, bukan frontier-gain/TSP optimization dan tidak
mengklaim performa paper: memakai SparseHistory + komponen free + A* proyek.

Tes: `PYTHONPATH=polinasi_nav python3 -m pytest polinasi_nav/test/test_global_route.py -q`.
Simulasi headless: `bash tools/run_sensor_localisation_validation.sh inspection`.
Belum validasi hardware; estimator tetap eksperimen scan-to-map ICP + IMU.
