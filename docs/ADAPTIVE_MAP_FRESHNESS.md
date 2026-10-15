# Batas umur peta adaptif — eksperimen simulasi

Aktif melalui `adaptive_map_age` pada fixture sensor rotating-lidar.
Ini bukan implementasi Dynablox/DYNUS dan bukan bukti lingkungan statis.

## Apa yang berubah

Semua fase navigasi ROS memakai penilaian peta yang sama: sebelum takeoff,
GUIDED/ARM/TAKEOFF, approach, scan, transfer dan return. Pemeriksaan ascent
memakai penilaian yang sama dengan batas khususnya. LAND ArduPilot tidak
diganti dengan algoritma landing baru. Semua pemeriksaan pose/cloud/IMU,
unknown, inflasi, lintasan, tracking dan pengereman tetap berlaku.

Default fixture: batas biasa 1 s bisa menjadi maksimum 1,5 s; khusus ascent
0,4 s bisa menjadi maksimum 0,8 s. Tidak pernah memperbarui timestamp lama.
Jika syarat tidak terpenuhi, batas biasa tetap berlaku. Penanganan lewat
batas tetap mengikuti fase: ascent mengerem/menunggu, gate global tetap
`stale_integrated_map`. Fitur ini tidak membuat semua failure auto-resume.

## Bukti yang diwajibkan

- Bit `observed` berisi voxel free/hit yang diperbarui sinar LiDAR pada
  integrasi terakhir, bukan history restore, launch prior atau depth camera.
- Kepercayaan hanya disimpan dalam kubus 4 m mengikuti drone, memakai kunci
  koordinat dunia tetap. Snapshot duplikat tidak menambah bukti.
- Setiap voxel pada kotak konservatif lintasan pengereman harus FREE,
  mendapat >=3 pengamatan berjarak >=0,2 s, rentang konsisten >=4 s,
  dan terakhir diamati <=2,5 s. Gap >2,5 s mengulang akumulasi bukti.
- Kotak mencakup actual/reference, convex hull rem, footprint/margin,
  quantisation satu voxel, cadangan jarak v*age + |a|*age²/2 dan guard atas
  untuk ascent. Ini konservatif; bukan prediksi gerak obstacle.
- Seluruh voxel harus lolos, bukan persentase mayoritas yang menyembunyikan
  lubang pengamatan. Unknown/occupied langsung menolak pelonggaran.
- Referensi maupun kecepatan terukur <=0,3 m/s. Health sensor wajib lolos.
- Covariance odometry diagonal harus positif, finite, posisi <=0,01 m²
  dan orientasi <=0,01 rad². Lompatan posisi terhadap prediksi >0,15 m
  menghapus bukti. Covariance estimator eksperimen belum dikalibrasi;
  syarat ini hanya sanity gate, bukan confidence statistik tervalidasi.

## Noise dan gerakan

Perubahan raw state mengulang bukti; tidak langsung diberi label bergerak.
Occupied sticky tetap obstacle: tidak menghapus daun untuk memperoleh
status stabil. Tidak mengabaikan perubahan voxel yang berpotensi bahaya.
Karena itu noise dapat membuat metode konservatif/sering menolak extension.
Belum ada klasifikasi/kecepatan objek dinamis atau penghilangan ghost obstacle.
Kebijakan ini tidak menjamin objek diam akan tetap diam, terutama daun angin.

`/navigation/status.map_freshness`: alasan, umur peta, batas biasa/efektif,
extension, jumlah unknown/occupied, stable_fraction, kotak dan waktu assess.
Log mapping worker/processing pipeline sebelumnya tetap tersedia.

## Uji

`PYTHONPATH=polinasi_nav python3 -m pytest polinasi_nav/test/test_map_trust.py -q`

Tes mencakup evidence, hard cap, dropout, noisy state flips, unknown,
occupied, speed, perpindahan origin, transport, launch prior, serta policy
bersama controller. Ini bukan validasi daun bergerak di Gazebo atau hardware.
Validasi flight terpisah memakai runner sensor; hentikan pada masalah.

## Rujukan desain

- [Dynablox](https://arxiv.org/abs/2304.10049): bukti ruang kosong yang
  memperhitungkan keterbatasan sensor/estimasi/pemetaan.
- [FASTER](https://arxiv.org/abs/2001.04420): lintasan cadangan berhenti
  dalam ruang yang diketahui. Tidak mengadopsi izin eksekusi ke unknown.
- [OctoMap](https://octomap.github.io/): penggabungan bukti sensor;
  metode saat ini bukan probabilistic OctoMap.
- [ROG-Map](https://arxiv.org/abs/2302.14819): pembaruan lokal efisien;
  incremental inflation C++ belum diintegrasikan oleh perubahan ini.
