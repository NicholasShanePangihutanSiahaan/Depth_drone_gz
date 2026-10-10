# Kecepatan lokal pada survei

Perubahan ini tidak menaikkan batas 0,30 m/s dan tidak menonaktifkan MPC,
collision checking, pemantauan sensor atau pemeriksaan pengereman.

- `local_speed_profile`: kecepatan di titik sambungan dibatasi sudut belokan,
  panjang bagian di sekitarnya dan kemampuan percepatan/pengereman dengan
  lintasan maju-mundur. Durasi polinomial quintic diperpanjang **per bagian**
  sampai batas turunan kecepatan/percepatan/jerk konservatif terpenuhi, dengan
  cadangan untuk pengaturan kecepatan adaptif. Posisi, kecepatan, percepatan
  tetap bersambung. Bentuk kurva hasilnya tetap diperiksa terhadap peta.
- `trajectory_geometry_cache`: kotak-kotak voxel yang membuktikan kurva aman
  disimpan. Setiap pemakaian ulang memeriksa semua kotak tersebut pada peta
  terbaru. Perubahan origin/resolusi/ukuran memaksa sertifikasi ulang; occupied
  atau unknown baru di dalam kotak tidak boleh diloloskan. Cache ini hanya
  untuk geometri, bukan untuk pengereman, tracking, atau kesegaran sensor.
- `survey_prefetch` / `survey_prefetch_seconds=3`: worker menyiapkan kelompok
  berikutnya menjelang akhir kelompok saat ini. Hasil hanya dipakai jika
  state, indeks dan titik awal cocok, scan baru mengonfirmasi target sebelumnya,
  dan kurva masih aman pada peta terbaru. Hasil spekulatif yang gagal dibuang.

**Batas penting:** kelompok berikutnya masih dimulai setelah berhenti.
Sambungan bergerak antarkelompok belum diaktifkan; diperlukan sambungan
posisi/kecepatan/percepatan dan sertifikasi pengereman yang cocok. Prefetch
mengurangi penantian komputasi, bukan menghilangkan berhenti yang disengaja.
Titik di dalam satu kelompok dapat dilewati tanpa berhenti. Pembatasan global
darurat saat sertifikasi pengereman gagal masih ada sebagai fallback.

Ini pendekatan lokal dengan batas turunan analitis, bukan implementasi TOPPRA
atau optimasi waktu global. Dasar pemisahan jalur dan waktu dapat dibaca pada
[paper TOPPRA](https://arxiv.org/abs/1707.07239). Pengujian turunan dan safety
tetap diperlukan pada perintah PVA yang benar-benar dipublikasikan.

Log `/navigation/status` / `navigation_debug.jsonl` menambahkan
`trajectory_timing`: durasi bagian, kecepatan sambungan, hit/miss cache, dan
prefetch dipakai/ditolak. `planner_job_id` membedakan tugas untuk analisis
waktu, dan `measured_speed_mps` membantu membandingkan kecepatan aktual.

```bash
source tools/ros_environment.sh
/usr/bin/python3 tools/compare_speed_profiles.py
bash tools/start_predictive.sh tour_check
```

Launcher sekarang membuka Gazebo dan RViz otomatis pada desktop. Ctrl+C pada
launcher menghentikan run dan kedua GUI miliknya. `POLINASI_SIM_GUI=0` adalah
opsi eksplisit untuk mesin tanpa desktop, bukan default saat pengujian ini.
Jangan menjalankan GUI tambahan melalui `view_simulation.sh` jika sudah terbuka.

Untuk menghentikan run yang diserahkan dalam keadaan berjalan:

```bash
cd /home/abin/polinasi_lidar
python3 tools/stop_simulation.py
```

Perintah memvalidasi PID, waktu mulai proses dan launcher proyek sebelum
mengirim SIGINT. Launcher menutup GUI dan backend miliknya, bukan proses lain.

Bandingkan rute, batas gerak, sensor dan safety gate yang sama: waktu simulasi
SURVEY, kecepatan rata-rata, waktu berhenti, tracking, kegagalan, dan waktu
planner/pipeline. Waktu nyata laptop juga dipengaruhi real-time factor dan GUI;
perbandingan waktu terbang memakai jam simulasi. Uji referensi bukan bukti
flight fisik atau lokalisasi LiDAR.
