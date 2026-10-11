# Jurnal masalah dan solusi

Dokumen ini merangkum masalah yang sudah tercatat dalam dokumentasi dan log
lokal. Disusun pada 11 Oktober 2026; bukan rekonstruksi lengkap seluruh
riwayat proyek. Tanggal mengikuti dokumentasi atau nama direktori simulasi
lokal (Asia/Jakarta), bukan waktu simulasi. Solusi yang diterapkan tidak
otomatis berarti masalah selesai; hasil pengujian ditulis terpisah.

Semua penerbangan yang dirangkum di sini memakai posisi ground truth
(posisi sebenarnya dari simulator), dengan navigasi GPS dinonaktifkan.
Hasilnya bukan validasi lokalisasi LiDAR–IMU atau keselamatan hardware.

## 10 Oktober 2026 — Voxel palsu saat LiDAR dimiringkan

- **Masalah:** muncul voxel non-lantai di arena kosong, termasuk ketika
  visual badan drone disembunyikan. Tidak cukup dijelaskan oleh pantulan badan.
- **Penyebab:** sampler GPU LiDAR Ogre2 pada instalasi Gazebo tidak memakai
  clamp di batas tekstur; sampel dapat mengambil jarak dari sisi yang salah.
- **Solusi diterapkan:** patch resource rendering pada salinan privat setiap
  simulasi. Tidak menghapus sektor sinar atau mengubah instalasi sistem.
- **Hasil:** replay arena kosong menghasilkan nol voxel palsu non-lantai.
  Kubus uji tetap menghasilkan 90 voxel occupied; obstacle nyata tidak dibuang.
- **Bukti:** [audit LiDAR](LIDAR_RENDERING_FIX.md),
  `reports/lidar_sampler_validation.json` (lokal, tidak diunggah).
- **Status:** teruji untuk kasus simulator tersebut; bukan uji sensor Livox nyata.

## 10 Oktober 2026 — Rute nominal dan perluasan ROI terlalu besar

- **Masalah:** rute awal memiliki bypass berdasarkan layout pohon, sehingga
  tidak murni lawnmower. Perluasan region of interest (ROI: area pemetaan
  relevan untuk navigasi) akibat pantulan lantai menambah beban pemetaan.
  Run `20261010_202639_44553` berhenti pada `stale_integrated_map`
  setelah tujuh target.
- **Solusi diterapkan:** rute nominal dibuat lawnmower murni; detour memakai
  voxel pengukuran. Perluasan permukaan dibatasi pada band ketinggian
  tubuh/rute serta jumlah pusat perluasan. Target yang terbukti occupied
  boleh dilewati dengan catatan celah cakupan; unknown tidak dianggap kosong.
- **Hasil:** tes terarah dan build ROS lulus menurut dokumentasi. Run gagal
  tersebut adalah bukti masalah, bukan bukti seluruh kebun selesai.
- **Bukti:** [perubahan obstacle avoidance](OBSTACLE_AVOIDANCE_UPDATE.md).
- **Status:** implementasi tersedia; cakupan penuh dan revisit celah belum tervalidasi.

## 11 Oktober 2026 — Origin diminta sebelum SITL siap

- **Masalah:** loop `054607` gagal sebelum misi, dengan
  `ConnectionRefusedError` saat menjalankan `set_origin.py`.
- **Solusi diterapkan:** tunggu koneksi sampai timeout, ulangi koneksi,
  periksa heartbeat FCU sistem 1 dan keadaan disarmed sebelum mengatur origin.
- **Hasil:** loop-loop berikutnya berhasil melewati pengaturan origin.
- **Bukti:** `reports/inspection_loop_20261011_054607/run_1_origin.log`
  dan `run_1_audit.json`; implementasi `tools/set_origin.py`.
- **Status:** startup berikutnya teruji; tidak membuktikan misi penerbangan selesai.

## 11 Oktober 2026 — Naik kontinu dan daun setelah normalisasi LiDAR

- **Masalah:** referensi naik sempat terpengaruh koreksi PID horizontal.
  Setelah LiDAR kembali horizontal, daun yang baru terlihat dapat menyebabkan
  pemeriksaan keselamatan menolak perintah. Loop `054821` gagal dengan
  `emergency_unavoidable:pid_final_safety`.
- **Solusi diterapkan:** pisahkan percepatan nominal dari koreksi PID;
  uji jalur naik dan pengereman setiap tick; tambahkan buffer vertikal
  teramati 0,5 m di atas lintasan berhenti. Unknown tetap ditolak.
- **Hasil:** loop berikutnya dapat melakukan ascent, normalisasi LiDAR,
  dan scanning. Ascent masih dapat berhenti lebih awal ketika map terlambat;
  tidak membuktikan pucuk batang atau ketinggian aman maksimum ditemukan.
- **Bukti:** [alur ascent](TILT_ASCENT.md),
  `reports/inspection_loop_20261011_054821/run_1_audit.json`.
- **Status:** perbaikan parsial teruji; misi penuh belum selesai.

## 11 Oktober 2026 — Ruang bawah jalur belum terbukti bebas

- **Masalah:** loop `060227` berhenti `blocked_route` saat scanning;
  batas depth 8 m membuang pengamatan tanah jauh yang diperlukan untuk
  memeriksa ruang di bawah jalur.
- **Solusi diterapkan:** samakan clip sensor simulasi dan batas proyeksi
  depth menjadi 20 m. Sebelum planning, tunggu badan/kamera stabil dan
  pengukuran depth baru yang sudah terintegrasi. Eksplorasi scanning
  dibatasi pada ketinggian yang sama, tanpa fallback turun diam-diam.
- **Hasil:** loop `062312` menyelesaikan enam viewpoint pohon pertama.
  Loop itu kemudian gagal pada approach pohon berikutnya, bukan selesai penuh.
- **Bukti:** [depth kamera](DEPTH_CAMERA_MAPPING.md),
  `reports/inspection_loop_20261011_060227/run_1_audit.json` dan
  `reports/inspection_loop_20261011_062312/run_1_audit.json`.
- **Status:** hasil positif terbatas; RGB-D ideal Gazebo bukan emulasi stereo ZED.

## 11 Oktober 2026 — Titik bantu menuju pohon berada dalam daun

- **Masalah:** setelah pohon pertama selesai pada loop `062312`, titik
  bantu approach sejauh maksimum 2 m masuk inflasi obstacle. A* menolak
  endpoint tersebut sebelum dapat mencari detour.
- **Solusi diterapkan:** cari titik bantu alternatif yang mendekati target
  akhir, sudah teramati bebas, dan memiliki segmen penghubung aman.
  Target viewpoint akhir tidak dipindah atau dihitung selesai.
- **Hasil:** replay peta tersimpan menemukan alternatif aman. Pada loop
  `104225`, detour awal menuju pohon kedua berhasil dijalankan;
  penerbangan kemudian berhenti karena depth terlambat.
- **Bukti:** bagian subgoal pada [alur inspeksi](TILT_ASCENT.md),
  `tools/replay_inspection_subgoal.py`, audit loop `062312` dan `104225`.
- **Status:** detour awal teruji; approach/scan pohon kedua belum selesai.

## 11 Oktober 2026 — Hasil depth terlambat ketika drone hampir diam

- **Masalah:** loop `104225` menyelesaikan enam viewpoint pohon ID 4,
  kemudian gagal `stale_depth_camera` pada approach berikutnya.
  Pengukuran depth valid masih tersedia, tetapi hasil integrasi map terlambat.
- **Solusi diterapkan:** toleransi floating point pada batas umur 1 detik;
  jika drone sudah diam pada hold aman dan depth valid masih segar,
  boleh menunggu integrasi maksimum 2 detik tanpa trajectory baru.
  Kehilangan kamera, hold tidak aman, atau timeout tetap gagal.
- **Hasil:** kebijakan dan tes ditambahkan, tetapi simulasi berikutnya
  masih gagal saat drone bergerak; solusi ini hanya menangani kasus diam.
- **Bukti:** [kebijakan depth](DEPTH_CAMERA_MAPPING.md),
  `polinasi_nav/polinasi_nav/depth_health.py`, audit loop `104225`.
- **Status:** belum menyelesaikan akar keterlambatan pipeline pemetaan.

## 11 Oktober 2026 — Depth terintegrasi terlambat saat bergerak

- **Masalah:** loop `180145` berhenti pada APPROACH, waktu simulasi
  27,25 detik, dengan `stale_depth_camera`. Kecepatan terukur sekitar
  0,248 m/s. Umur depth valid sekitar 0,25 detik, tetapi hasil terintegrasi
  sekitar 1,05 detik, melewati batas 1 detik.
- **Diagnosis:** pengukuran masih datang; bukti ini menunjukkan hasil
  pemetaan terlambat, bukan membuktikan kamera mati. Kebijakan menunggu
  ketika diam tidak berlaku saat drone bergerak. Penyebab komputasi
  spesifik dan pengaruh GUI masih perlu diukur, bukan diasumsikan.
- **Solusi:** belum diterapkan. Tindak lanjut adalah audit antrean/integrasi
  dan biaya publikasi/visualisasi; evaluasi rem–hold pemulihan yang tetap
  memeriksa ruang teramati dan memiliki timeout. Jangan sekadar menganggap
  unknown kosong atau menaikkan timeout tanpa pengujian.
- **Hasil:** belum ada pohon selesai pada run ini; landing tidak terkonfirmasi.
- **Bukti:** `reports/inspection_loop_20261011_180145/run_1_audit.json`,
  `run_1_observer.log`, dan `run_1_watchdog.jsonl`.
- **Status:** terbuka. Dua pohon × enam viewpoint lalu landing/disarm
  belum berhasil divalidasi.

## Cara memperbarui jurnal

Tambahkan entri setiap masalah atau perbaikan bermakna dengan format:
tanggal → masalah → diagnosis → solusi → hasil uji → bukti → status.
Jika diagnosis belum pasti, tulis sebagai hipotesis. Jika tes belum dijalankan,
tulis demikian. Jangan menimpa kegagalan lama dengan klaim keberhasilan baru.
Direktori `reports/` adalah bukti lokal yang umumnya diabaikan Git; jurnal
dan dokumentasi ini disimpan di repository, sedangkan file log tidak otomatis
tersedia di GitHub.
