# Loop simulasi dengan RViz dan kamera

```bash
cd /home/abin/polinasi_lidar
bash tools/run_inspection_loop.sh
```

Skrip membangun paket, menjalankan Gazebo server + ArduPilot SITL + ROS,
mengatur origin, memeriksa setup, memulai inspeksi, memantau kegagalan,
lalu menutup proses miliknya dan menyimpan ringkasan audit. RViz dan jendela
kamera dibuka pada setiap run dan ditutup saat run dihentikan. GUI Gazebo tidak
dibuka; server Gazebo tetap menjalankan fisika dan sensor. Desktop display
harus tersedia; kegagalan membuka display tidak boleh diklaim sebagai GUI aktif.
Lokalisasi memakai ground truth, bukan validasi lokalisasi LiDAR.

Log: `reports/inspection_loop_<waktu>/`. `run_1_audit.json` menunjukkan
penyelesaian, kegagalan, dan lokasi peta HTML. Kecepatan/error dalam ringkasan
hanya sampel pergantian fase; log lengkap tetap diperlukan untuk audit detail.
Konfigurasi memakai batas jumlah pohon demo, bukan janji seluruh kebun selesai.

`Ctrl+C` menghentikan run milik skrip. Simulasi lain yang sudah aktif tidak
dimatikan; skrip menolak mulai untuk mencegah dua publisher kontrol.

Untuk hingga tiga pengulangan yang **berhasil**:

```bash
bash tools/run_inspection_loop.sh --runs 3 --wall-timeout 7200
```

Saat gagal atau timeout, loop berhenti. Agen membaca log, memperbaiki kode,
dan menjalankan skrip lagi; skrip tidak memperbaiki kode secara otomatis.
Jika dialog izin menawarkan aturan yang diingat, pilih persetujuan untuk
prefix sempit `bash /home/abin/polinasi_lidar/tools/run_inspection_loop.sh`.
Ketersediaannya bergantung kebijakan aplikasi. Skrip tidak melewati sandbox,
tidak mengoperasikan hardware, dan tidak melakukan push/download.
