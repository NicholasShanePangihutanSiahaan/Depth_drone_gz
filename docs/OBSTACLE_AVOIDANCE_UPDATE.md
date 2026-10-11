# Navigasi lawnmower — perubahan lokal 10 Oktober 2026

Rute nominal kuning sekarang lawnmower tanpa bypass berdasarkan batang.
Layout pohon hanya memilih batas kebun, bukan obstacle/free-space evidence.
Detour A* memakai voxel pengukuran, pemeriksaan kurva halus dan pengereman.
Kecepatan perintah tetap 0,3 m/s.

## Pemilihan voxel

Sebelum filter endpoint, pantulan terukur diperiksa terhadap halo dekat koridor
dan drone. Lebar halo berdasarkan ukuran tubuh, clearance, tracking margin,
dan resolusi voxel. Pantulan relevan di luar ROI menambahkan area pengamatan
di sekitar titik itu. Setelah audit run awal, pantulan tambahan dibatasi pada
band ketinggian tubuh/rute: lantai jauh tidak lagi memperluas ROI secara massal.
Pusat pantulan dibatasi 16, paling banyak empat pusat baru per scan.
Feedback penghalang dari planner tetap dapat memperbesar radius hingga
batas konfigurasi. Perluasan tidak menciptakan free: hanya interval sinar
terukur membersihkan voxel; endpoint hit tetap occupied. Bagian pohon jauh
di luar volume relevan tidak wajib menjadi voxel.

Log: `mapping_roi.region.surface_halo_candidates`,
`surface_expansion_centres_added`, `expansions`, `expansion_centres`.

## Tujuan terhalang

Target yang overlap obstacle teramati dengan kernel tubuh + margin dilewati.
Unknown atau target di luar window tidak menjadi alasan skip. Keputusan
worker diperiksa lagi pada map live sebelum diadopsi. Target berikutnya tetap
membutuhkan jalur aman. `survey_skipped_targets` memuat indeks, posisi,
timestamp, alasan dan `coverage_complete=false`. Indeks waypoint menunjukkan
kemajuan daftar, bukan jumlah lokasi yang diamati. Belum ada revisit otomatis
untuk celah cakupan.

## PID dan pengereman

Koreksi diperiksa pada keadaan terukur dan referensi. Bila koreksi penuh
ditolak, coba 50% lalu 0%, setiap kandidat dibatasi acceleration/jerk dan
diperiksa ulang. Jika semuanya ditolak, tetap BRAKE/HOLD_ABORT. Integrator
tidak bertambah saat koreksi dikurangi. `pid.safety_checks` mencatat kandidat
dan alasan: `reaction_segment_blocked`, `predicted_speed_limit`,
`braking_outside_window`, `braking_box_blocked`. Kotak yang ditolak dan raw
occupied/unknown dicatat; counts bukan klasifikasi lengkap karena inflasi
bisa berasal dari voxel di sebelah kotak. Logging posisi/kecepatan aktual,
referensi dan percepatan nominal ditambahkan untuk run berikutnya.

Tes terarah mencakup ROI prefilter, endpoint, ray occlusion, perluasan map,
occupied versus unknown goal, PID dan pengurangan koreksi aman, detour
sintetis, transport map dan navigasi. Tes sintetis bukan validasi penerbangan
LiDAR localisation. Build ROS lulus.

Run awal: `ros_ground_truth_palm_farm_20261010_202639_44553_mapping`, kedua GUI,
GPS-disabled dengan ground truth. Semua setup checks lulus. Hasil penerbangan
berhenti pada `stale_integrated_map` setelah tujuh target. Banyak perluasan
lantai menyebabkan pekerjaan map terlalu besar; pembatasan band ketinggian
dan jumlah pusat di atas merupakan perbaikan setelah audit tersebut.
Belum ada klaim seluruh kebun selesai.

Prinsip pemisahan cakupan dan jalur penghubung bebas obstacle dibahas oleh
[Choset & Pignon](https://publications.ri.cmu.edu/coverage-path-planning-the-boustrophedon-decomposition).
Ini bukan implementasi penuh cellular decomposition pada paper tersebut.
