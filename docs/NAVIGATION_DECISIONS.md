# Pemindaian dan kecepatan: audit keputusan

## Perubahan

- Planner tetap mencoba jalur aman terlebih dahulu. Bila gagal, log membedakan
  voxel belum diketahui, obstacle, batas peta lokal, dan batas pencarian A*.
- Sebelum EXPLORE, tunggu minimal 0,8 detik simulasi dan dua pemindaian yang
  benar-benar diintegrasikan. Pilih posisi pengamatan yang diprediksi membuka
  voxel belum diketahui pada koridor tujuan, bukan sekadar menambah peta di
  tempat lain. Prediksi ini bukan jaminan voxel akan berhasil teramati.
- Pengurangan kecepatan berdasarkan ruang untuk pengereman searah lintasan,
  bukan jarak ke ruang belum diketahui di segala arah. Batas kecepatan MPC
  tetap 0,3 m/s; pemeriksaan tubuh drone, margin, tracking, dan pengereman
  aktual tidak dihapus.
- Perjalanan RETURN dibagi menjadi tujuan antara maksimal 2 m agar home yang
  jauh tidak langsung dipaksakan ke luar jendela peta lokal. Tiap perjalanan
  diperiksa ulang; drone tidak mulai turun sebelum mencapai tujuan home.
- Pemulihan peta hanya membaca voxel dalam jendela lokal. Pembaruan riwayat
  memakai batch, dengan penghapusan duplikasi antar sinar satu scan. TTL,
  ukuran voxel, dan sifat obstacle yang tetap tersimpan tidak dilonggarkan.

## Sinar Gazebo tanpa pantulan

Gazebo GPU LiDAR menulis jarak tanpa pantulan sebagai +infinity. XYZ-nya
memiliki tanda sesuai arah sinar. Pembaca baru memverifikasi ukuran scan,
ring, urutan, dan sudut dari titik valid, kemudian mengganti sinar tanpa
pantulan yang terverifikasi dengan ujung pada batas jangkauan konfigurasi.
Sepanjang sinar tersebut dapat ditandai observed-free; setelah batasnya
tetap unknown. Ujung virtual bukan obstacle dan tidak ditambahkan ke log
point cloud permukaan. Obstacle yang sudah diamati tidak dihapus.

Data mentah `/livox/lidar` tetap tidak berubah. Khusus konfigurasi simulasi
yang mengaktifkan `simulation_no_return_rays`, pemetaan menerima
`/mapping/lidar_rays`. NaN, sinar jarak negatif, scan dengan geometri salah,
dan sensor dropout tidak menghasilkan bukti ruang kosong. Pembacaan ini
**bukan decoder tanpa pantulan Livox asli**.

Untuk membatasi beban, sinar permukaan diproses dengan stride 4 dan sinar
tanpa pantulan dengan stride 12: satu dari 4/12 sinar dipakai tiap pembaruan,
dengan fase bergilir. Semua kelompok tetap mendapat giliran. Ini tidak
membesarkan voxel atau menganggap celah antar sinar kosong; voxel yang belum
teramati tetap unknown. Log point cloud permukaan menyimpan semua hit yang
lolos filter, bukan hanya subset pemetaan. `mapping_ray_counts` mencatat
jumlah sinar yang benar-benar diproses.

Semantik dan susunan scan diperiksa pada
[kode resmi Gazebo GpuLidarSensor](https://github.com/gazebosim/gz-sensors/blob/gz-sensors8/src/GpuLidarSensor.cc).
Prinsip tetap menyediakan lintasan aman di ruang teramati juga dibahas pada
[FASTER](https://arxiv.org/abs/2001.04420); implementasi ini bukan algoritma
FASTER atau replika optimisasinya.

## Log dan pengujian

`reports/<run>/map_3d/navigation_debug.jsonl` berisi keputusan per 0,5 detik
simulasi dan setiap perubahan penting: jumlah/contoh voxel penghalang,
posisi pengamatan pilihan, faktor kecepatan lama dan baru, umur pengukuran,
statistik sinar, serta waktu komputasi. Rekaman ini ditulis proses pemetaan,
bukan proses pengiriman perintah terbang.

```bash
python3 tools/analyse_navigation_debug.py \
  reports/<run>/map_3d/navigation_debug.jsonl --output reports/decision_audit.json
```

Untuk uji awal empat tujuan asli survei, lalu kembali dan landing:

```bash
bash tools/start_predictive.sh tour_check
```

Konfirmasi origin, periksa kesiapan, jalankan pengamat, lalu mulai misi sesuai
[panduan survei](SURVEI_KEBUN.md). Uji empat tujuan bukan validasi rute kebun
penuh 326 tujuan. Semua penerbangan ini menggunakan posisi ground-truth,
bukan pembuktian lokalisasi LiDAR.
