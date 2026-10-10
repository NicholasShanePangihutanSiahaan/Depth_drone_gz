# Status snapshot lidar360-mpc

Snapshot ini berasal dari proyek lokal polinasi_lidar, dengan basis upstream
151a6b3. Repo asli tidak diubah. Branch default repo tujuan tidak diganti.

- 192 tes lulus pada 10 Oktober 2026.
- Run sebelumnya dengan ground truth menyelesaikan empat target survei,
  pulang, landing; bukan bukti seluruh kebun atau lokalisasi LiDAR.
- Profil kecepatan lokal, cache geometri yang memeriksa peta terbaru,
  prefetch lintasan berikutnya, dan log waktu pipeline sudah diimplementasikan.
  Uji numerik menunjukkan manfaat; validasi flight perubahan ini belum selesai.
- Run GUI terbaru dengan RTF 0,4 berhenti sebelum target pertama: MPC beberapa
  kali memperoleh solusi berumur >0,25 detik, kemudian pencocokan timestamp
  FCU/odometri tidak tersedia dan menghasilkan HOLD_ABORT. Run, Gazebo, RViz
  sudah dihentikan. Threshold safety tidak dilonggarkan.
- Peralihan bergerak antarkelompok belum aktif. Prefetch menerima lintasan
  baru setelah berhenti dan lolos pemeriksaan peta terbaru.

MPC ini model respons PVA orde rendah, bukan NMPC model rotor. Simulasi memakai
ground truth; sensor-based localisation dan flight hardware belum tervalidasi.

Dependensi, build dan log/cloud lengkap tidak dimasukkan ke Git. Ringkasan
benchmark/hasil ada di reports yang dipilih dan docs; log mentah tetap lokal.
Lihat LOCAL_SETUP.md, SURVEI_KEBUN.md, LOCAL_SPEED_PROFILE.md, PROCESSING_TIME.md.
