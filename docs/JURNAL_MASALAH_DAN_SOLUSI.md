# Jurnal masalah dan solusi

Dokumen ini merangkum masalah yang sudah tercatat dalam dokumentasi dan log
lokal. Disusun pada 11 Oktober 2026; bukan rekonstruksi lengkap seluruh
riwayat proyek. Tanggal mengikuti dokumentasi atau nama direktori simulasi
lokal (Asia/Jakarta), bukan waktu simulasi. Solusi yang diterapkan tidak
otomatis berarti masalah selesai; hasil pengujian ditulis terpisah.

Semua penerbangan yang dirangkum di sini memakai posisi ground truth
(posisi sebenarnya dari simulator), dengan navigasi GPS dinonaktifkan.
Hasilnya bukan validasi lokalisasi LiDAR–IMU atau keselamatan hardware.

Bagian **Kronologi penemuan** menceritakan urutan kejadian berdasarkan log
dan dokumentasi yang tersedia, bukan pengamatan visual baru. Angka detik
di dalam cerita adalah waktu simulasi, bukan lama menunggu di laptop.
`BRAKE` berarti program masuk fase pengereman; tidak dengan sendirinya
membuktikan drone sudah diam atau sudah landing. Enam viewpoint selesai
berarti enam posisi pengamatan memenuhi syarat program, bukan bukti seluruh
permukaan pohon terlihat atau bunga terdeteksi.

## 10 Oktober 2026 — Voxel palsu saat LiDAR dimiringkan

**Kronologi penemuan:** saat memeriksa hasil LiDAR yang dimiringkan, terlihat
susunan voxel aneh yang terus muncul pada bagian tertentu. Awalnya perlu
dibedakan apakah itu daun, tanah, pantulan badan drone, atau kesalahan sensor
simulasi. Karena itu pengujian dipindahkan ke arena kosong, tanpa pohon,
dengan drone tidak di-arm. Lantai tetap merupakan obstacle yang sah;
yang dicari adalah voxel tambahan di atas lantai. Voxel aneh masih muncul
bahkan ketika visual badan drone disembunyikan. Temuan ini membuat penjelasan
“semua voxel tersebut adalah pantulan badan” tidak cukup. Audit dilanjutkan
ke rendering GPU LiDAR. Setelah patch sampler, pengujian arena kosong tidak
lagi menghasilkan voxel palsu non-lantai. Kubus nyata kemudian ditambahkan
untuk memastikan perbaikannya bukan sekadar menghapus semua obstacle.

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

**Kronologi penemuan:** ketika melihat peta hasil survey, jalur nominal
tampak sudah berbelok mengitari pohon. Ini menimbulkan pertanyaan apakah
belokan dibuat oleh obstacle avoidance atau sudah dimasukkan ke rute awal.
Audit menunjukkan rute awal memang memiliki bypass berdasarkan layout
batang. Rute kemudian diubah menjadi lawnmower murni agar jalur penghindaran
benar-benar ditentukan dari hasil sensor. Pada pengujian berikutnya drone
berhasil melewati tujuh target, tetapi kemudian berhenti dengan
`stale_integrated_map`. Pemeriksaan log pemetaan menunjukkan banyak
perluasan ROI berasal dari pantulan lantai, bukan hanya obstacle relevan di
sekitar jalur terbang. Jadi masalah berikutnya bukan semata bentuk rute:
pekerjaan pemetaan ikut membesar sampai hasil map terlambat. Perluasan ROI
kemudian dibatasi, tanpa mengubah unknown menjadi ruang bebas.

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

**Kronologi penemuan:** loop pertama mulai menyiapkan simulator dan hendak
mengatur titik acuan lokal sebelum mengirim perintah misi. Namun proses
`set_origin.py` langsung berhenti dengan `ConnectionRefusedError`. Audit
tidak memiliki event penerbangan: masalah muncul sebelum takeoff, bukan
drone gagal approach atau scanning. Skrip mencoba terhubung ketika endpoint
SITL belum siap menerima koneksi. Startup diperbaiki agar menunggu dan
mencoba lagi sampai timeout, kemudian memverifikasi identitas FCU. Pada
loop-loop berikutnya tahapan origin berhasil dilewati dan drone bisa takeoff.

- **Masalah:** loop `054607` gagal sebelum misi, dengan
  `ConnectionRefusedError` saat menjalankan `set_origin.py`.
- **Solusi diterapkan:** tunggu koneksi sampai timeout, ulangi koneksi,
  periksa heartbeat FCU sistem 1 dan keadaan disarmed sebelum mengatur origin.
- **Hasil:** loop-loop berikutnya berhasil melewati pengaturan origin.
- **Bukti:** `reports/inspection_loop_20261011_054607/run_1_origin.log`
  dan `run_1_audit.json`; implementasi `tools/set_origin.py`.
- **Status:** startup berikutnya teruji; tidak membuktikan misi penerbangan selesai.

## 11 Oktober 2026 — Naik kontinu dan daun setelah normalisasi LiDAR

**Kronologi penemuan:** saat pengujian naik kontinu, referensi yang seharusnya
hanya naik vertikal sempat ikut dipengaruhi koreksi PID horizontal. Ini
menunjukkan bahwa perintah nominal dan koreksi pengendali belum cukup
terpisah. Setelah pemisahan itu diperbaiki, masalah lain muncul pada loop
`054821`: drone sudah approach, mengamati atas, dan naik; tetapi ketika
LiDAR kembali horizontal untuk scanning, daun yang baru teramati membuat
pemeriksaan akhir PID menolak perintah. Log melaporkan
`emergency_unavoidable:pid_final_safety`, bukan bukti tabrakan fisik.
Artinya, lolos pemeriksaan saat LiDAR menghadap atas belum cukup untuk
menjamin posisi tersebut tetap lolos ketika arah pengamatan berubah.
Buffer vertikal teramati 0,5 m kemudian ditambahkan agar kenaikan berhenti
lebih konservatif sebelum normalisasi LiDAR. Perubahan ini membantu run
berikutnya mencapai scanning, tetapi belum menyelesaikan semua masalah misi.

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

**Kronologi penemuan:** pada loop `060227`, drone berhasil takeoff, approach
pohon ID 1, memiringkan LiDAR, naik, lalu mulai scanning sekitar ketinggian
2,94 m. Pada detik 113,55 log sudah menunjukkan indeks viewpoint 1.
Ketika hendak melanjutkan perpindahan scanning, drone justru masuk EXPLORE
pada detik 118, kembali ke SURVEY pada 128,90, lalu berhenti dengan
`blocked_route` pada 161,45. Dari tampilan point cloud jalur dapat terlihat
lapang, tetapi pemeriksaan voxel membutuhkan seluruh ruang tubuh dan margin
di bawah jalur terbukti bebas, bukan hanya garis lintasannya. Audit menemukan
pengamatan tanah jauh yang diperlukan dibuang oleh batas depth 8 m.
Batas simulasi dan proyeksinya kemudian diperluas menjadi 20 m. Run berikutnya
berhasil menyelesaikan enam viewpoint pohon pertama; keberhasilan itu
terbatas pada run dan model RGB-D ideal tersebut, bukan jaminan sensor nyata.

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

**Kronologi penemuan:** pada loop `062312`, drone sudah berhasil scanning
pohon pertama, ID 4. Setelah naik hingga sekitar 3,02 m dan menyelesaikan
enam viewpoint, program masuk SEARCH pada detik 241,35 lalu memilih pohon
ID 7 pada 241,40. Seharusnya drone mulai perjalanan ke pohon berikutnya,
tetapi pada detik 259,25 program masuk BRAKE dengan `blocked_route`.
Audit tidak berhenti pada kesimpulan “pohon kedua tidak bisa dicapai”.
Pemeriksaan planner dan peta tersimpan menunjukkan bahwa titik bantu pendek
yang dipilih menuju pohon kedua justru berada di dalam obstacle setelah
ditambah ukuran drone dan margin keselamatan. Karena endpoint itu tidak
aman, A* menolaknya sebelum dapat mencari jalan mengitari daun.
Pemilih titik bantu diperbaiki untuk mencari alternatif yang sudah teramati
bebas. Replay menemukan alternatif aman, dan pada run berikutnya drone
benar-benar menjalankan detour awal menuju pohon kedua. Namun perjalanan
itu kemudian gagal karena masalah depth yang berbeda, dijelaskan di bawah.

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

**Kronologi penemuan:** pada loop `104225`, drone kembali menyelesaikan
scanning pohon ID 4, kali ini sekitar ketinggian 2,87 m. Pada detik 237,00
enam viewpoint selesai, lalu pohon ID 7 dipilih pada 237,05. Berbeda dari
percobaan sebelumnya, titik bantu alternatif memungkinkan drone menjalankan
detour awal; ketinggiannya kemudian sekitar 2,73 m saat approach.
Tetapi pada detik 255,40 program masuk BRAKE dengan `stale_depth_camera`.
Saat kegagalan tercatat, kecepatan terukur hanya sekitar 0,007 m/s,
sehingga drone hampir diam. Pemeriksaan timestamp menunjukkan data depth
valid masih datang, tetapi hasil yang sudah diintegrasikan ke map tertinggal
sekitar satu detik. Dengan demikian, nama error tidak boleh langsung
ditafsirkan sebagai kamera mati. Aturan kontrol diperbaiki agar batas satu
detik tidak salah ditolak akibat floating point dan agar drone yang sudah
diam pada posisi aman boleh menunggu integrasi secara terbatas. Solusi itu
belum mengatasi keterlambatan ketika drone sedang bergerak.

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

**Kronologi penemuan:** setelah aturan menunggu ketika diam ditambahkan,
simulasi diulang dari awal pada loop `180145`, dengan RViz dan viewer kamera
dibuka sejak startup. Drone berhasil takeoff sampai sekitar 2 m dan mulai
approach pohon ID 4. Kali ini masalah muncul jauh lebih awal: pada detik
27,25, sebelum scanning pohon pertama, program masuk BRAKE. Kecepatannya
sekitar 0,248 m/s, jadi kondisi menunggu “drone sudah diam” tidak terpenuhi.
Log menunjukkan pengukuran depth valid berumur sekitar 0,25 detik, tetapi
hasil integrasinya berumur sekitar 1,05 detik. Ini mengungkap batas solusi
sebelumnya: aturan hold membantu kasus diam, tetapi belum membuat pipeline
pemetaan cukup cepat saat bergerak. Run dihentikan oleh watchdog untuk audit;
bukan selesai dua pohon atau landing normal. Belum ada pengukuran yang
memisahkan biaya integrasi, antrean, dan pengaruh GUI secara meyakinkan,
sehingga penyebab komputasi spesifik masih terbuka.

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

## 11 Oktober 2026 — Audit dan pengurangan duplikasi input pemetaan

**Kronologi penemuan:** setelah run `180145` gagal ketika baru approach,
audit dilanjutkan dengan membandingkan umur frame kamera dan umur hasil
map. Kamera masih mengirim pengukuran; hasil integrasi yang tertinggal.
Penelusuran kode menemukan grid lengkap disalin dan dikirim ke worker setiap
scan, padahal worker rolling-map membangun kembali window dari history
miliknya dan hanya membutuhkan versi serta timestamp depth dari grid lama.

- **Solusi diterapkan:** kirim metadata kecil sebagai input rolling-map;
  pertahankan salinan lengkap untuk mapper non-rolling. Antrean subscription
  depth dibatasi satu frame terbaru. Tambahkan waktu integrasi LiDAR, depth,
  rekonstruksi/inflasi/delta serta timestamp pekerjaan depth berjalan/selesai.
- **Pengujian:** tes membandingkan seluruh array keselamatan dan delta hasil
  input lengkap versus metadata. Kesalahan pertama terjadi pada assertion
  perbandingan tuple berisi array NumPy; assertion diperbaiki menjadi
  perbandingan array satu per satu, tanpa mengubah hasil mapper.
- **Status:** optimasi input dan instrumentasi, bukan bukti akar seluruh
  keterlambatan selesai. Simulasi ulang masih diperlukan. Timeout sensor,
  inflasi obstacle dan penolakan unknown tidak dilonggarkan.

## 11 Oktober 2026 — Map berubah saat persiapan uji perbaikan

**Kronologi penemuan:** saat loop `182547` dimulai untuk menguji optimasi
pemetaan, agen lain sedang menambahkan bunga ke kebun. Pemeriksaan world
privat run `ros_ground_truth_palm_farm_20261011_182549_63407_mapping`
menemukan link bunga beserta collision. Jadi run ini sudah memuat lingkungan
berbunga, walaupun tujuannya masih inspeksi tanpa deteksi bunga.
Map lama kemudian dikembalikan dan map baru dipisahkan oleh agen lain;
perubahan file sumber itu tidak mengganti world yang sudah dimuat simulator.

- **Masalah:** perubahan lingkungan dan perbaikan navigasi berlangsung
  bersamaan, sehingga hasil run ini tidak dapat dibandingkan langsung
  dengan baseline lama untuk mengisolasi efek optimasi kode.
- **Keputusan:** sesuai arahan pengguna, lanjutkan run yang masih aktif.
  Permintaan stop sempat dibuat tetapi dibatalkan sebelum dijalankan;
  session masih aktif ketika diperiksa. Simulasi berikutnya wajib memakai
  `palm_farm.sdf` (map lama), bukan `palm_farm_flowers.sdf`.
- **Hasil sementara:** pada detik simulasi 50,95 drone masih APPROACH
  pohon ID 1, altitude sekitar 2 m, tanpa failure; umur depth terintegrasi
  0,35 detik. Ini belum membuktikan dua pohon selesai atau landing.
- **Pengujian optimasi:** 19 tes terarah dan 22 tes adapter ROS lulus.
- **Status:** run berlangsung; hasil akhir harus dicatat setelah berhenti.
  Pekerjaan agen map tidak diubah atau dicampurkan ke commit navigasi.

## 11 Oktober 2026 — Status navigasi tidak diterima setelah approach

**Kronologi penemuan:** loop `182547` sempat melewati kegagalan depth run
sebelumnya dan drone bergerak sekitar 0,27 m/s. Pada sampel terakhir,
detik 68,45, drone masih APPROACH pohon pertama, hampir diam, dan umur
depth terintegrasi 0,25 detik. Watchdog kemudian tidak menerima status
navigasi selama 15 detik waktu nyata dan menghentikan run beserta viewers.
Ini berbeda dari `stale_depth_camera`: belum ada failure sensor pada
status terakhir. Traceback di log ROS berasal dari SIGINT saat shutdown,
bukan bukti bahwa node crash sebelum watchdog bertindak.

- **Diagnosis:** penyebab hilangnya status belum terbukti. Log lama tidak
  mencatat umur `/clock` ketika status hilang, sehingga belum dapat
  membedakan clock/simulator tertahan dari callback navigasi tertahan.
- **Solusi diagnostik:** watchdog menyimpan checkpoint penuh setiap 5 detik
  waktu nyata, termasuk umur penerimaan status dan `/clock`, selain event
  perubahan fase. Batas penghentian 15 detik tidak diperlonggar.
- **Kontrol lingkungan:** runner berikutnya memeriksa world privat dan
  menolak memulai misi jika ada model/link bunga. Hash world dan mesh
  map lama disimpan dalam `run_1_environment.json` untuk audit lokal.
- **Hasil:** belum ada pohon selesai atau landing terkonfirmasi pada run
  ini. Simulasi ulang map lama diperlukan; tidak mengklaim optimasi kode
  menyelesaikan seluruh keterlambatan.
- **Bukti:** `reports/inspection_loop_20261011_182547/run_1_audit.json`,
  `run_1_watchdog.jsonl`, dan log ROS direktori run terkait.
- **Status:** terbuka; instrumentasi tambahan bukan perbaikan akar masalah.

## 11 Oktober 2026 — Map terlambat mengakhiri ascent terlalu dini

**Kronologi penemuan:** pada loop `183635`, world lama tanpa bunga telah
diverifikasi. Drone berhasil takeoff dan approach pohon ID 1; clock/status
tetap berjalan melewati titik kegagalan run sebelumnya. Pada detik 76,50
LiDAR mulai observasi ke atas, lalu ASCEND dimulai pada 81,80. Namun hanya
0,40 detik kemudian program masuk NORMALIZE dengan `ascent_map_too_old`.
Pada detik 87,50 program sudah SCAN, padahal altitude tetap sekitar 2 m.
Tidak ada abort sensor saat itu, tetapi perilaku ini tidak memenuhi maksud
naik sampai batas aman. Run dihentikan oleh agen untuk perbaikan;
error watchdog status hilang sesudah penghentian bukan kegagalan penerbangan
independen pada run ini.

- **Penyebab kode:** map terlambat memicu Brake, tetapi setelah berhenti
  program memperlakukan alasan itu sebagai akhir ascent dan menormalisasi
  LiDAR, sama seperti ketika koridor benar-benar terhalang.
- **Solusi diterapkan:** rem dan hover, pertahankan tilt, tunggu map baru
  segar setelah jeda dimulai, lalu periksa lagi segmen, pengereman dan guard
  sebelum melanjutkan naik. Batas umur map tetap 0,35 detik. Timeout pemulihan
  memakai `tree_mount_timeout` (20 detik default); tidak pulih berarti
  `ascent_mapping_recovery_timeout`, bukan scan berhasil.
- **Perbaikan terkait:** rem nominal memakai percepatan nominal ascent,
  bukan mengintegrasikan koreksi PID horizontal kembali ke referensi naik.
- **Pengujian:** 20 tes terarah ascent/FSM/depth-health lulus. Kasus tambahan
  mencakup menunggu dan resume setelah map baru, timeout menjadi failure,
  serta map baru dengan ceiling unknown tetap menolak naik.
- **Status:** tes terarah lulus; pemulihan dalam penerbangan belum tervalidasi.
  Simulasi berikutnya tetap memakai map lama, RViz dan kamera.

## 11 Oktober 2026 — Scanning terhenti karena biaya koridor ROI rapat

**Kronologi penemuan:** loop `185204` memakai map lama yang terverifikasi.
Perbaikan ascent berhasil: drone tidak langsung scanning ketika map terlambat,
melainkan menjalankan 16 pause/resume dan naik sampai sekitar 3,42 m.
Ascent berakhir dengan `observation_guard_limit`, lalu scanning dimulai.
Tiga viewpoint tercatat selesai pada scan timestamp 129,8; 156,7; 198,4.
Perpindahan ke viewpoint ketiga sempat membutuhkan satu pengamatan tambahan
untuk satu unknown voxel dan berhasil mendapat rute langsung. Perpindahan
berikutnya membutuhkan pengamatan tambahan untuk enam unknown voxel.
Pada detik 216,45, ketika drone mulai bergerak lagi sekitar 0,083 m/s,
run berhenti dengan `stale_depth_camera`. Depth valid berumur 0,65 detik,
tetapi hasil terintegrasi 1,05 detik. Jadi pemulihan ascent membantu satu
fase, bukan menyelesaikan keterlambatan seluruh pipeline.

- **Bukti biaya:** pekerjaan map terakhir sekitar 2166,88 ms waktu nyata:
  LiDAR 1323,17 ms; depth 461,31 ms; restore/inflasi/delta 382,40 ms.
  ROI pada log memuat 77 segmen koridor, termasuk 76 segmen dari titik rapat
  trajectory. Pemeriksaan membership mengulang pekerjaan untuk tiap segmen.
  Biaya callback visual/log juga tidak nol; ini bukan diagnosis tunggal
  semua sumber latensi.
- **Solusi diterapkan:** sederhanakan hanya polyline ROI dengan deviasi
  maksimal 0,05 m (atau seperempat resolusi bila lebih kecil), lalu tambah
  radius koridor sebesar batas deviasi yang dihitung. Koridor baru mencakup
  koridor lama secara konservatif. Tidak mengubah trajectory/perintah
  terbang, inflasi tubuh, timeout sensor, atau menjadikan unknown bebas.
  Log ROI menyimpan jumlah segmen sebelum/sesudah dan tambahan radius.
- **Pengujian:** 28 tes terarah ROI/depth/ascent lulus. Benchmark geometri
  memakai ROI tersimpan dan 1000 sinar sintetis: 77 segmen menjadi 4;
  tambahan radius 0,02863 m; seluruh sampel ROI lama tetap tercakup.
  Median sampling/pemeriksaan ROI 129,16 ms menjadi 36,97 ms (sekitar
  3,49 kali lebih cepat pada benchmark ini). Ini bukan pengukuran seluruh
  pipeline penerbangan atau jaminan kecepatan drone naik 3,49 kali.
- **Bukti lokal:** `reports/inspection_loop_20261011_185204/`,
  `reports/roi_simplification_benchmark.json`.
- **Status:** ascent pulih dalam run ini tetapi sering berhenti; pemulihan
  belum mulus. Optimasi ROI belum diuji dalam penerbangan; dua pohon lalu
  landing tetap belum tervalidasi.

## 11 Oktober 2026 — Penjadwalan pemetaan memperbanyak jeda ascent

**Kronologi penemuan:** loop `192122` kembali memakai map lama. Optimasi
ROI aktif pada approach: contoh 13 segmen menjadi 2 tanpa mengubah trajectory.
Drone berhasil approach dan mulai ascent, tetapi sudah tercatat 19 jeda map
dan 18 resume pada detik 102,90, altitude sekitar 2,68 m. Ini bukan abort,
tetapi belum memenuhi tujuan gerakan mulus. Pada detik 108,15 worker sekitar
414 ms waktu nyata, sementara receive-to-submit sekitar 659 ms; seluruh
pipeline sampel sekitar 1590 ms. Drone tetap bisa naik setelah map pulih,
tetapi biaya menunggu juga perlu dikurangi. Run dihentikan sebelum seluruh
scanning; fase terakhir pada audit adalah `NORMALIZE`. Ini bukan bukti optimasi ROI gagal pada
scanning karena tahap itu belum diuji pada run ini.

- **Temuan kode:** mapper hanya memeriksa antrean dan future setiap
  0,1 detik simulasi. Pada pacing 0,15, satu interval itu sekitar 0,67 detik
  waktu nyata bila pacing tercapai. Ada biaya worker/transport lain juga;
  timer bukan satu-satunya penyebab.
- **Solusi diterapkan:** poll setiap 0,02 detik simulasi. Saat tidak ada
  pekerjaan selesai/map baru, callback segera kembali dan tidak mengirim
  snapshot. Frekuensi sensor dan batas umur map tidak diubah.
- **Pengujian:** ditambahkan tes adapter bahwa polling pada versi map
  tidak berubah tidak mempublikasikan snapshot baru. Tes ini belum
  dijalankan karena pengguna meminta penghentian pekerjaan simulasi.
- **Status:** perubahan disimpan sebagai checkpoint belum diuji, bukan
  perbaikan yang sudah terbukti. Jika pekerjaan dilanjutkan nanti, perlu
  menjalankan tes tersebut dan membandingkan jumlah pause/resume, umur
  map serta waktu pipeline dalam penerbangan.

## 11 Oktober 2026 — Loop dihentikan atas permintaan pengguna

**Kronologi penghentian:** setelah run terakhir memperlihatkan drone bisa
approach dan melanjutkan ascent setelah beberapa jeda, pengguna meminta
pekerjaan dihentikan dan jurnal diselesaikan. Tidak dimulai simulasi atau
pengujian baru. Perubahan polling terakhir disimpan dengan status belum diuji.

- **Bukti penghentian:** sesi
  `.dependencies/ros_ground_truth_palm_farm_20261011_192124_75825_mapping/session.json`
  mencatat `stopped: true`. Runner loop telah keluar. Cleanup launcher
  menutup proses simulasi, RViz dan kamera milik run ini. Penghentian
  proses bukan bukti bahwa drone menyelesaikan landing.
- **Catatan audit:** `navigation_status_missing_wall_15_seconds` muncul
  setelah launcher sengaja dihentikan sementara watchdog masih menunggu.
  Itu artefak penghentian, bukan bukti kegagalan penerbangan baru.
- **Hasil yang benar-benar dicapai pada rangkaian terbaru:** run `185204`
  mencapai sekitar 3,42 m dan menyelesaikan tiga viewpoint pohon pertama,
  tetapi kemudian gagal karena depth terintegrasi terlambat. Run `192122`
  mencapai fase `NORMALIZE`; scanning lengkap dan landing tidak terbukti.
- **Kesimpulan:** target scan dua pohon lalu landing dengan mulus belum
  tercapai. Semua penerbangan ini memakai ground-truth localisation;
  hasilnya tidak memvalidasi lokalisasi LiDAR.
- **Jika dilanjutkan:** uji perubahan polling, gunakan map lama, lalu
  audit latensi sensor sampai setpoint, jeda ascent dan freshness depth.
  Keberhasilan harus dibuktikan oleh dua pohon dengan enam viewpoint
  masing-masing serta landing dan disarm, bukan perkiraan dari awal run.
- **Bukti lokal:** `reports/inspection_loop_20261011_192122/run_1_audit.json`.

## 11 Oktober 2026 — Loop dilanjutkan dan polling lolos tes adapter

**Kronologi:** pengguna meminta loop dimulai lagi setelah penghentian
sebelumnya. Sebelum menerbangkan drone, dijalankan tes adapter ROS pada
domain terisolasi. Seluruh 23 tes lulus, termasuk bahwa polling mapper
tidak mengirim snapshot bila versi map tidak berubah. Ini memvalidasi
regresi adapter, bukan membuktikan berkurangnya latensi penerbangan.

- **Simulasi baru:** `reports/inspection_loop_20261011_225246/`.
- **Lingkungan:** ground-truth, map lama `palm_farm`, misi inspeksi dua
  pohon, RViz dan kamera; bukan validasi lokalisasi LiDAR.
- **Status saat pencatatan:** simulasi terdaftar dan proses persiapan
  origin/kesiapan sistem sedang berjalan. Hasil penerbangan belum ada.
- **Batas klaim:** perubahan polling kini lolos tes adapter, tetapi
  keberhasilan scan dua pohon dan landing masih harus dibuktikan.

## 11 Oktober 2026 — Viewpoint pertama selesai, depth tertinggal saat mulai transit

**Kronologi penemuan:** run `225246` berhasil takeoff, approach, tilt 68°,
naik sampai sekitar 3,42 m, lalu menyelesaikan viewpoint pertama pada scan
timestamp 123,0. Ascent menjalankan empat pause/resume, dibandingkan 16
pada run `185204`; ini perbandingan run, bukan eksperimen terkontrol yang
mengisolasi seluruh beban laptop. Pada detik 127,65 drone baru mulai menuju
viewpoint kedua ketika `stale_depth_camera` memicu penghentian otomatis.
RViz dan kamera ditutup bersama simulasi.

- **Bukti:** depth valid terbaru 127,4 (umur 0,25 detik), tetapi depth
  terintegrasi terakhir 126,6 (umur 1,05 detik). Kecepatan terukur sekitar
  0,014 m/s. Job LiDAR terakhir tidak membawa depth baru; job berikutnya
  membawa depth 127,4 tetapi belum selesai. Ini keterlambatan integrasi,
  bukan bukti kamera kehilangan semua datanya. Kontributor keterlambatan
  meliputi worker, callback ekspor dan jeda frame; penyebab semua jeda
  frame kamera belum dipastikan.
- **Pengukuran pipeline:** median sekitar 944 ms waktu nyata; p95 1774 ms;
  maksimum sampel 2519 ms. Metrik ini dari ROS mapper menerima scan sampai
  setpoint pertama memakai map, bukan total respons fisik drone.
- **Solusi:** tambah pemulihan depth gerak. Pada umur integrasi >60% dari
  timeout, bila frame valid masih segar dan actual/reference braking aman,
  hentikan trajectory lewat pengereman yang sudah diperiksa. Tunggu hingga
  actual drone berhenti serta map depth baru setelah awal pemulihan tersedia,
  lalu rencanakan ulang ke viewpoint yang sama. Tidak menambah scan berhasil
  atau melewati viewpoint. Data sensor basi, koridor pengereman tidak aman,
  atau penantian >2 detik tetap menjadi kegagalan terminal.
- **Keselamatan:** batas freshness sensor tidak dilonggarkan. Failure lain
  tetap mengambil alih pemulihan. Watchdog hanya memperbolehkan status
  sementara `depth_recovering`; timeout/dropout tetap menghentikan run.
- **Log tambahan:** `depth_motion_recovery` mencatat action, jumlah pemulihan,
  waktu mulai, umur depth valid/integrasi, threshold dan timeout.
- **Pengujian:** 21 tes depth/ascent/runner serta 23 tes adapter ROS lulus.
  Pemulihan belum diuji dalam penerbangan pada saat commit ini.
- **Bukti lokal:** `reports/inspection_loop_20261011_225246/`.
- **Status:** satu viewpoint, bukan satu pohon lengkap. Dua pohon lalu
  landing belum tercapai; run berikutnya memakai map lama dan dua viewer.

## 11 Oktober 2026 — Pemulihan depth pertama berhasil dalam penerbangan

**Kronologi:** run lanjutan `231335` memakai map lama dan dua viewer.
Takeoff dan approach berhasil. Ascent mencapai sekitar 3,42 m dengan satu
pause/resume map. Tiga viewpoint selesai. Saat menuju viewpoint keempat,
depth terintegrasi tertinggal dan pemulihan preventif benar-benar aktif.

- **Bukti awal pemulihan:** detik 226,05, kecepatan terukur 0,291 m/s,
  umur depth terintegrasi 0,65 detik dan depth valid 0,25 detik. Koridor
  pengereman diperiksa aman sebelum `depth_recovering` dimulai.
- **Respons:** pengereman selesai, state hold tercatat pada 227,85 dengan
  actual speed sekitar 0,028 m/s. Pada 227,90, depth terintegrasi baru
  berumur 0,50 detik, actual drone sudah cukup lambat, dan misi kembali
  `SURVEY` lewat perencanaan ulang. Pemulihan memerlukan sekitar 1,85
  detik simulasi, masih di bawah batas 2 detik.
- **Batas klaim:** ini satu pemulihan yang berhasil; tidak membuktikan
  sensor dropout dapat diabaikan, semua latensi hilang, atau misi selesai.
  Nama state hold internal masih `HOLD_ABORT`, tetapi failure sementara
  `depth_recovering` berbeda dari kegagalan terminal; log mencatat keduanya.
- **Status saat dicatat:** simulasi masih berjalan, tiga viewpoint selesai,
  drone mendekati viewpoint keempat; dua pohon dan landing belum terbukti.
- **Bukti lokal:** `reports/inspection_loop_20261011_231335/run_1_watchdog.jsonl`.

## 12 Oktober 2026 — Pohon pertama selesai, approach pohon kedua terhenti

**Kronologi penemuan:** run `231335` berhasil menyelesaikan enam viewpoint
pohon pertama dan memilih pohon berikutnya (ID tracker 49). Drone bergerak
dari ketinggian scanning ke approach rendah. Dua belas pemulihan depth
berhasil dilanjutkan, sehingga dropout/timeout depth tidak menjadi failure
terminal run ini; tetapi banyak jeda berarti gerakannya belum mulus.
Pada detik 406,10 drone hampir diam sekitar 2,98 m, kemudian `blocked_route`
menghentikan run dan kedua viewer. Landing belum terjadi.

- **Diagnosis:** target antara 2 m ke depan berada dalam margin daun.
  Log menunjukkan dua occupied voxel sekitar z=3,7 m, tanpa unknown pada
  koridor nominal. Pemilih detour hanya mencoba ketinggian nominal,
  ketinggian saat ini dan ketinggian akhir 2 m; semua kandidat ditolak.
  Guard tidak meminta explore untuk koridor nominal yang hanya occupied,
  sehingga program tetap menunggu target terhalang sampai timeout.
- **Solusi:** saat approach, coba juga turun sebagian 0,4/0,8 m, tetap
  dibatasi panjang target antara 2 m. Detour hanya diterima bila endpoint
  serta segmen yang dilalui sudah aman. Jika tidak ada alternatif aman,
  kandidat yang hanya unknown dan tanpa occupied pada koridornya boleh
  menjadi *hint pengamatan*, bukan perintah terbang. Guard lama tetap
  memerlukan pengamatan dan planner/collision checking sebelum bergerak.
  Scanning tetap pada ketinggian sama; perubahan ini hanya untuk approach.
- **Log tambahan:** jumlah kandidat aman/kandidat pengamatan, penolakan
  karena tidak maju, endpoint occupied, koridor occupied atau unknown,
  serta sampel unknown dan penyebab blockage awal.
- **Uji offline:** rekonstruksi window dari `map.json` saat penghentian
  menemukan satu kandidat aman sekitar (-8,73; 3,44; 2,57) m, serta 42
  kandidat pengamatan unknown. Map tersimpan adalah hasil akhir shutdown,
  bukan salinan identik snapshot tepat sebelum failure. Ini bukti alternatif
  geometri pada map tersebut, bukan bukti penerbangan berhasil.
- **Pengujian:** 26 tes target antara/depth/ascent/runner lulus, termasuk
  partial descent, unknown tetap tidak boleh dilalui, serta fully blocked
  tetap ditolak. Perubahan detour ini belum diuji dalam penerbangan.
- **Bukti lokal:** `reports/inspection_loop_20261011_231335/` dan
  `reports/ros_ground_truth_palm_farm_20261011_231337_101238_mapping/map_3d/`.
- **Status:** pohon pertama lengkap, pohon kedua belum selesai; dua pohon
  lalu landing belum tercapai. Loop berikutnya memakai map lama.

## 12 Oktober 2026 — Mengurangi gerakan mundur saat memilih titik explore

**Kronologi penemuan:** pengguna melihat drone sering menjauh setelah scan
viewpoint. Audit executed path run `231335` menemukan radius dari pusat
pohon meningkat sekitar 2,00→2,26 m setelah viewpoint ke-2,
2,00→2,60 m setelah ke-3, dan 2,00→2,76 m setelah ke-5. Perpindahan itu
bertepatan dengan `EXPLORE`, bukan aturan mundur setelah setiap scan.
Proyeksi lintasan terhadap arah badan saat scan juga negatif, tetapi itu
tidak membuktikan drone terbang mundur terhadap yaw aktual sepanjang gerak.

- **Penyebab:** scorer lama menyeimbangkan gain, jarak dan progress, tetapi
  tidak memprioritaskan arah viewpoint akhir atau memberi penalti kembali
  ke lintasan yang sudah ditempuh. Gain besar dapat memenangkan langkah
  yang menjauh dari tujuan.
- **Solusi:** fitur opt-in pada konfigurasi tilt. Bila ada kandidat aman
  dan informatif searah viewpoint berikutnya, pilih dari kelompok itu.
  Beri penalti progress negatif/gerakan berlawanan arah serta kembali
  dekat titik-titik pada tiga lintasan sebelumnya. Sampel riwayat dibatasi
  64 titik; ini penalti endpoint, bukan pemeriksaan seluruh overlap rute.
  Kandidat samping dan mundur tetap dicadangkan bila kelompok searah
  tidak menghasilkan informasi baru yang cukup. Radius, jumlah kandidat,
  anggaran explore, pemeriksaan map dan braking tidak dilonggarkan.
- **Pengujian menemukan masalah:** tes fallback pertama gagal karena
  kandidat samping mendominasi jatah fallback. Pembagian kandidat diubah
  menjadi kelompok searah/samping/mundur, total tetap maksimal sesuai
  `exploration_candidates`. Tes kemudian lulus.
- **Log:** `route_blockage.exploration_selection` berisi preferred goal,
  jumlah kandidat eligible/evaluated, alasan `forward_informative` atau
  `fallback_only_informative`, titik terpilih, gain, progress, jarak tempuh,
  revisit fraction, penalti dan skor. Tidak adanya kandidat dicatat juga.
- **Pengujian:** 49 tes exploration/route-guard/local-subgoal/depth/ascent/
  runner dan 23 tes adapter ROS lulus. Tes tambahan memeriksa gain besar
  di belakang tidak mengalahkan kandidat searah yang informatif, fallback
  mundur tetap tersedia, dan penalti riwayat mengubah pilihan tanpa
  mengizinkan segmen tidak aman. Hasil penerbangan fitur ini belum ada.
- **Dasar metode:** inspirasi next-best-view berbasis informasi dan biaya
  perjalanan dari [Bircher dkk., 2016](https://www.research-collection.ethz.ch/entities/publication/ae2aefd8-1e86-4601-bb36-fa175be18481).
  Prioritas arah dan bobot penalti di sini adalah heuristik implementasi
  lokal, bukan reproduksi penuh planner RRT pada paper atau parameter
  yang telah dioptimalkan dalam penerbangan.
- **Status:** perbaikan partial descent approach sudah tersimpan dalam
  `2d54eec`; fitur arah explore melengkapinya sebelum loop berikutnya.
  Tidak menjanjikan semua gerakan menjauh hilang: bila itu satu-satunya
  posisi pengamatan aman yang informatif, fallback tetap dapat dipilih.

## 12 Oktober 2026 — Pencarian target approach memblokir callback kontrol

**Kronologi:** run `003126` menyelesaikan enam viewpoint pohon pertama
pada sekitar detik 319, tanpa jeda ascent atau pemulihan depth. Beberapa
transit tetap perlu explore; scorer baru mencatat fallback ketika kandidat
searah tidak menghasilkan gain yang cukup. Saat memilih kandidat pohon
berikutnya (ID tracker 40), program gagal dengan `clock_or_scheduler_gap`
pada detik 320,521, lalu simulasi dan kedua viewer dihentikan otomatis.

- **Bukti timing:** control callback maksimum 6139,88 ms waktu nyata,
  trajectory/safety callback 6136,51 ms; gap tick 0,921 detik simulasi.
  Sampel pipeline terakhir 6725,04 ms, dengan navigation-to-setpoint
  6243,66 ms. Log callback sebelumnya menyebut callback LiDAR pendek;
  maksimum callback kontrol mengungkap pekerjaan berat yang sebenarnya.
- **Penyebab kode:** penyiapan kamera memanggil `_mission_hint`, yang
  mencari ratusan kandidat partial descent/detour saat target nominal
  occupied. Pencarian ini masih berjalan di kontrol live walaupun rute
  utama direncanakan pada worker terpisah.
- **Solusi:** petunjuk kamera live menjadi perhitungan geometris ringan
  saja, tidak menjadi izin terbang. Clone worker membuang goal geometris
  tersebut dan menghitung ulang kandidat detour, A*, trajectory serta
  safety certification. Hasil tetap diperiksa terhadap map live sebelum
  digunakan. Diagnostik pemilihan subgoal worker disalin kembali ke status.
  Batas timing 0,2 detik tidak diubah.
- **Pengujian:** 64 tes exploration/route-guard/local-subgoal/depth/ascent/
  runner/survey dan 23 tes adapter ROS lulus. Tes baru memastikan callback
  live tidak memanggil pencarian detour dan worker menghitung ulang goal.
- **Batas klaim:** run mencapai satu pohon dengan enam scan, belum dua
  pohon dan landing. ID tracker berikutnya adalah kandidat geometri,
  bukan bukti independen bahwa objek itu benar-benar batang pohon.
  Pemisahan pencarian ini belum diuji dalam penerbangan saat dicatat.
- **Bukti lokal:** `reports/inspection_loop_20261012_003126/`.
- **Penyimpanan:** pengguna meminta penghentian commit/push mulai sekarang;
  perubahan berikut dan jurnal disimpan lokal tanpa commit atau push.

## 12 Oktober 2026 — Kontrol tidak lagi tertahan, tetapi kandidat pohon kedua berasal dari daerah daun

- **Kronologi:** run `inspection_loop_20261012_011144` berhasil takeoff,
  approach pohon di sekitar (9, 0), tilt LiDAR, naik, lalu menyelesaikan
  keenam viewpoint. Saat mencari pohon berikutnya, drone memilih ID 29
  dengan pusat (-0,829, 10,004). Audit posisi model dunia lama menemukan
  pusat kandidat ini berjarak 3,806 m dari batang terdekat. Simulasi dan
  viewer dihentikan secara sengaja sebelum menganggap kandidat itu sebagai
  pohon kedua yang berhasil diinspeksi. Posisi model hanya dipakai untuk
  audit, tidak dimasukkan sebagai input detektor/navigasi.
- **Hasil perbaikan sebelumnya:** pencarian worker sekitar 7,98 detik
  waktu nyata tidak lagi menahan kontrol; callback kontrol maksimum
  sekitar 39,15 ms pada transisi tersebut. Belum ada abort timing sebelum
  penghentian manual. Dua pemulihan depth sementara terjadi dan pulih.
  Run belum memenuhi dua pohon dan landing. Pesan watchdog
  `navigation_status_missing_wall_15_seconds` setelah stop adalah akibat
  penghentian manual, bukan kegagalan terbang baru.
- **Bukti objek palsu:** point cloud akhir memuat 871 titik dalam radius
  0,8 m dari kandidat; terdapat titik tanah, tetapi tidak ada titik batang
  antara 0,5 dan 3,5 m. Kelompok atas mulai sekitar 4 m. Kandidat sebelumnya
  hanya didukung 15 titik, residual 0,093 m, dan empat observasi.
- **Diagnosis:** kode lama menghitung ulang persentil tanah dari buffer
  30 detik. Setelah melihat ke atas dan titik tanah lama kedaluwarsa,
  persentil bisa naik ke daun sehingga daun diperlakukan sebagai batang
  rendah. Ini mekanisme yang direproduksi tes; buffer detektor persis pada
  saat pemilihan tidak direkam, sehingga pergeseran persentil pada run
  tersebut belum diukur langsung. Diagnostik baru ditambahkan untuk itu.
- **Solusi:** referensi tanah diukur sekali dari kelompok titik rendah
  yang cukup tersebar pada awal deteksi, lalu dipertahankan saat LiDAR
  melihat ke atas. Kelompok kompak tanpa bukti tanah tidak boleh
  menginisialisasi referensi. Status memuat referensi tanah, persentil
  terkini, rentang tinggi pencarian batang, dan jumlah deteksi.
- **Pengujian:** 46 tes tree-inspection/ascent/local-subgoal/exploration/
  runner lulus. Regresi menguji daun atas setelah titik awal kedaluwarsa,
  penolakan inisialisasi dari kelompok kompak, serta batang rendah baru
  yang tetap terdeteksi setelah titik awal kedaluwarsa.
- **Batas solusi:** ini detektor geometri untuk kebun datar, bukan
  segmentasi pohon semantik ataupun estimasi tanah mengikuti lereng.
  Referensi awal yang keliru masih merupakan risiko. Jangan mengklaim
  validasi medan bergelombang atau lokalisasi LiDAR. Perbaikan referensi
  tanah belum diuji dalam penerbangan pada saat entri dibuat.
- **Bukti:** `reports/inspection_loop_20261012_011144/` dan
  `reports/ros_ground_truth_palm_farm_20261012_011146_117463_mapping/map_3d/`.
- **Penyimpanan:** perubahan dan jurnal hanya lokal; tidak commit/push.

## 13 Oktober 2026 — Uji MID-360 di depan dengan roll tetap 90 derajat

- **Permintaan:** pengguna memberikan video Exyn Nexys/X500 dan meminta
  konfigurasi pembanding beserta simulasi dengan RViz. Frame video lokal
  menunjukkan orientasi sensor berubah; sumbu mekanis/sudut persis belum
  dapat dipastikan. Uji ini bukan salinan hardware Exyn, melainkan kandidat
  MID-360 dengan roll tetap 90 derajat yang telah dibahas. Mekanisme roll
  kontinu belum dibuat maupun diuji.
- **Implementasi:** launcher `tools/start_front_roll_test.sh` membuat
  fixture terpisah dari dunia identification, mempertahankan tanah dan
  menambah empat kotak kalibrasi. Sensor ditempatkan pada XYZ
  (0,65; 0; 0,45) relatif badan dan roll pi/2. Persiapan model mengubah
  pose link LiDAR/IMU di SDF sesuai TF, bukan mengganti nama frame saja.
  Canonical link tetap badan drone. Model sensor asli dan kebun lama
  tidak diubah. Konfigurasi yang dihasilkan ada di
  `reports/front_roll_fixture/config.json`.
- **Prosedur:** simulator Gazebo/SITL tetap digunakan, ground-truth untuk
  transform badan; tidak ada mission-start, arming, takeoff, atau perintah
  aktuator. RViz saja dibuka; kamera dan Gazebo GUI tidak dibuka. Depth ZED
  tidak diintegrasikan pada tes ini agar cakupan dapat diatribusikan ke
  LiDAR. Pantulan badan tidak disembunyikan oleh visibility mask.
- **Hasil uji:** 13 tes front-roll/tree-inspection lulus. Audit pasif 45
  detik waktu nyata menerima 88 cloud yang berhasil ditransformasi, dua
  kegagalan lookup TF, FCU terbaca dan selalu tidak armed. Hit kumulatif:
  depan 3960, depan-atas 1848, depan-bawah 6536, atas 3960, tanah tepat di
  bawah sensor 133312. Hit kumulatif bukan jumlah titik unik atau ukuran
  clearance. Audit coverage `passed=true`.
- **Peta:** snapshot peta waktu simulasi 10 detik menyimpan masing-masing
  9/6/25/9 voxel occupied pada empat kotak dan 28 voxel tanah di bawah
  sensor. Point cloud dan voxel tersimpan di HTML/JSON, terlihat di RViz.
  Ini membuktikan keberadaan objek uji pada peta, bukan kualitas peta
  keseluruhan atau tidak adanya self-return palsu.
- **Batas:** lima arah yang diuji teramati saat diam; ini tidak berarti
  seluruh ruang bola terlihat atau jalur terbang sudah aman. Belum uji
  terbang kebun, occlusion seluruh badan, putaran mekanis, deskew,
  lokalisasi sensor, atau hardware. Pola LiDAR Gazebo berupa scan grid
  ideal, bukan pola non-repetitif/timing per titik MID-360 asli. Posisi
  pemasangan merupakan kandidat simulasi, bukan rekomendasi mekanik final.
- **Bukti:** `reports/front_roll_fixture/coverage.json` dan
  `reports/ros_ground_truth_identification_20261013_162539_12649_mapping/map_3d/`.
- **Cara ulang:** `bash tools/start_front_roll_test.sh`. Di terminal kedua:
  `source tools/ros_environment.sh`, lalu
  `python3 tools/check_front_roll_test.py --config reports/front_roll_fixture/config.json --output reports/front_roll_fixture/coverage.json --seconds 45`.
  Jangan menjalankan `start_mission.py` untuk tes pasif ini. Stop run
  dengan `tools/stop_simulation.py --run` dan direktori run yang tepat.
- **Penyimpanan:** tanpa commit/push sesuai instruksi pengguna.

## 13 Oktober 2026 — Takeoff dan hover dengan LiDAR rolling kontinu: dua penyebab berbeda

- **Kronologi awal:** pengguna meminta takeoff-hover di kebun lama,
  dengan roll LiDAR kontinu dan Gazebo/RViz. Joint velocity controller
  roll ditambahkan pada model privat, kecepatan 0,4 rad/s (satu putaran
  sekitar 15,7 detik simulasi), dan TF mengikuti sudut joint yang terukur.
  IMU LiDAR ikut sensor; IMU badan tetap milik ArduPilot. GT digunakan
  untuk posisi badan; tidak ada klaim lokalisasi LiDAR.
- **Kegagalan pertama:** run `20261013_163321_14252` menerima takeoff,
  tetapi baru terangkat sedikit lalu terjadi mismatch ExternalNav dan
  laporan crash ArduPilot dengan AngErr=175. Model memindahkan massa
  LiDAR 0,265 kg sejauh 0,65 m di depan tanpa kompensasi. Audit plugin
  menunjukkan input ArduPilot berasal dari IMU badan; dugaan sebelumnya
  bahwa sensor roll langsung menjadi IMU FCU tidak terbukti.
- **Perbaikan mekanik simulasi:** model privat chassis dibuat untuk
  menggeser pusat massa inertial base 1,5 kg sebesar -0,11483 m pada X,
  sehingga perubahan momen massa horizontal LiDAR dikompensasi. Massa
  total tidak dikurangi. Ini pendekatan redistribusi payload/baterai pada
  simulasi, bukan rancangan hardware yang telah divalidasi. Visual chassis
  tidak ikut bergeser; inersia/mount/gimbal belum model mekanik lengkap.
  Semua aset dependency asli dipertahankan; tes memastikan hal itu.
- **Baseline terpisah:** run `20261013_164004_15807`, rate roll nol,
  berhasil takeoff dan masuk hover sekitar 2 m tanpa mismatch/crash.
  Body/FCU/sensor direkam terpisah dalam
  `reports/rolling_hover/balanced_static.jsonl`.
- **Kegagalan kedua:** run `20261013_164336_16921` dengan massa seimbang
  dan rolling berhasil hover sekitar 10,5 detik simulasi, tetapi gagal
  `stale_integrated_map` pada t=30,801. Cloud age 0,101 s sedangkan map
  age 1,201 s; map worker sekitar 1,25 detik wall, handoff/snapshot dan
  logging juga mahal. Setup pada t=28,3 sempat lulus seluruh pemeriksaan
  publisher/sensor/FCU, tetapi itu bukan jaminan bebas kegagalan sesudahnya.
  Run beserta kedua viewer dihentikan untuk audit, tidak retry buta.
- **Perbaikan komputasi:** konfigurasi khusus hover mengaktifkan ROI
  dekat 6 m dengan koridor 3 m, memakai surface stride 6 dan no-return
  stride 18. Point cloud hit tetap disimpan di luar ROI; voxel navigasi
  hanya diproses di wilayah relevan dan halo yang ditambahkan mekanisme
  ROI. Luar wilayah yang belum diproses tetap unknown, bukan dianggap
  bebas. Timeout map 1 s tidak dilonggarkan. Konfigurasi misi inspeksi lama
  tidak diganti dengan konfigurasi eksperimen hover ini.
- **Hasil run berikutnya:** `20261013_164841_19160` berhasil takeoff,
  lalu hover pada sampel t=19,8 sampai setidaknya t=55,3. Pada audit
  t=53,3: durasi hover 33,5 detik, 2,133 putaran roll saat hover, tanpa
  failure. Altitude sampel hover 1,999676–2,001219 m; max tracking error
  0,10458 m saat transisi hover dan terakhir 0,00334 m. Ini extrema sampel
  audit sekitar 0,5 s, bukan semua pengukuran atau jaminan terus-menerus.
  Audit akhir 108 sampel, 72 sampel hover, daftar failure kosong.
- **Timing terakhir:** map age 0,4 s; callback kontrol maksimum 94,43 ms
  wall. Pipeline mapping-to-first-setpoint p50 1788,94 ms, p95 2421,67 ms,
  max 3159,83 ms wall. Simulasi diperlambat; angka ini bukan bukti bahwa
  hardware real-time akan sanggup pada laju sama. CPU masih perlu optimasi.
- **Tes:** 23 tes front-roll/mount/survey dan lima tes adapter mount ROS
  lulus. Termasuk TF dari sudut roll aktual, joint velocity controller,
  preservasi FOV, massa seimbang, dan dependency asli tidak dimodifikasi.
- **Bukti:** `reports/rolling_hover/balanced_rolling_roi.jsonl`,
  `balanced_rolling_roi.summary.json`, `balanced_rolling.jsonl`,
  `rolling_setup.json` serta direktori run/peta terkait. `rolling_setup`
  milik run sebelum ROI, bukan pemeriksaan runtime run terakhir.
- **Handoff:** proses pengamatan read-only dihentikan setelah bukti
  dua putaran hover; simulator, Gazebo, RViz tetap menyala untuk pengguna.
  Hover dikonfigurasi 600 detik simulasi sebelum return/landing; rangkaian
  landing tersebut belum divalidasi pada pengujian ini.
- **Cara ulang:** `bash tools/start_rolling_hover_test.sh 0.4`, lalu
  `source tools/ros_environment.sh`,
  `.venv/bin/python tools/set_origin.py --timeout 180`, dan
  `python3 tools/start_mission.py --timeout 180`. Audit opsional:
  `python3 tools/audit_rolling_hover.py --output reports/rolling_hover/new_run.jsonl --seconds 300`.
  Stop hanya run aktif menggunakan `tools/stop_simulation.py --run` dan
  direktori yang dicetak launcher, agar kedua viewer ikut ditutup.
- **Batas:** belum uji lintasan kebun, lokalisasi sensor, putaran kabel,
  occlusion lengkap, atau dinamika mount hardware. Scan Gazebo ideal pada
  timestamp sensor, bukan scan MID-360 nyata yang membutuhkan deskew per
  titik saat berputar. Tidak ada commit atau push.

## 13 Oktober 2026 — Konsep visual drone span 50 cm dan viewer web lokal

- **Permintaan:** pengguna memilih perubahan visual, memberi sketsa,
  mengubah span menjadi 50 cm antara motor berseberangan, mengarahkan ZED
  ke kiri dengan pitch atas/bawah, dan LiDAR di depan dengan roll X.
- **Implementasi:** model terpisah `polinasi_visual_50cm` mencakup badan,
  lengan X, motor/propeller statis, baterai bawah dengan strap, skid/kaki,
  ZED dan dudukan, LiDAR serta sumbu roll. Diagonal pusat motor tepat 0,5 m.
  LiDAR pivot pada (0,205; 0; -0,035) m relatif badan dan kamera pada
  (0; 0; 0,098) m dengan yaw +90 derajat. Ukuran lainnya perkiraan, bukan
  hasil pengukuran hardware. Tidak memakai posisi sensor jauh 0,65 m
  pada eksperimen hover sebelumnya.
- **Aset:** generator menghasilkan scene JSON dan SDF visual dari geometri
  yang sama. Viewer WebGL tanpa library/CDN menyediakan orbit/zoom, roll
  sensor manual atau animasi, pitch kamera, dan sumbu referensi. Endpoint
  hanya melayani HTML dan scene JSON, bind 127.0.0.1:8766; repository dan
  API kontrol penerbangan tidak diekspos. Tidak dipublikasikan remote.
- **Tes:** dua tes geometri lulus: diagonal motor, arah kiri kamera,
  pitch positif melihat atas, sumbu roll, serta kesesuaian primitive/pose
  SDF dengan scene browser. Sintaks JavaScript diperiksa dengan Node.
  Endpoint JSON lokal berhasil diakses. Browser runtime melaporkan tidak
  ada browser tersambung; inspeksi screenshot/interaksi browser belum
  dilakukan dan tidak diklaim lulus.
- **Batas:** model static visual preview, bukan pengganti model fisik
  penerbangan aktif. SDF belum memiliki sensor aktif, motor/controller,
  collision dan inertial baru. Visual dapat menjadi permukaan pantulan
  jika dimasukkan ke dunia Gazebo, tetapi self-return/occlusion model ini
  belum diuji. Self-filter harus disesuaikan saat integrasi, bukan menghapus
  pantulan objek nyata dekat drone. Kamera pada simulasi lama belum diubah
  menjadi menghadap kiri melalui pekerjaan preview ini.
- **Cara lihat:** `http://127.0.0.1:8766/`. Jika server berhenti, jalankan
  `python3 tools/serve_drone_viewer.py --port 8766`. Untuk membangun ulang
  aset: `python3 tools/generate_drone_visual.py`.
- **Penyimpanan:** tanpa commit/push; perubahan agen pembuat kebun dijaga.

## 13 Oktober 2026 — Integrasi branch rotating-lidar, model visual 50 cm, dan koreksi sensor

- **Lingkup:** model visual baru diintegrasikan ke model run-local Iris;
  motor, collision dan inertia lama dipertahankan. Visual asli dependency
  tidak dimodifikasi; salinan privat memakai komponen yang sama dengan
  viewer web. Konfigurasi lama tetap ada. Span visual tidak menjadi
  validasi span/thrust/propeller fisik. Propeller visual masih statis.
- **Sensor:** LiDAR di depan X=0,205 m, roll sumbu X, kamera ZED di atas
  menghadap kiri yaw=pi/2 dan pitch lokal. TF memakai sudut joint aktual.
  Filter endpoint berubah dari kotak besar menjadi primitive badan dan
  envelope sensor bergerak; luar ROI tetap unknown. Model penerbangan
  menyeimbangkan momen massa horizontal payload pada salinan chassis.
- **Masalah startup:** launcher pertama memakai path relatif yang ditolak
  pemeriksaan ownership; diubah ke absolut. Satu restart ditolak karena
  port SITL belum selesai ditutup; tidak membunuh proses sembarang dan
  menunggu cleanup sebelum mengulang.
- **Masalah self-return:** run `20261013_174426_28369` berhasil naik tetapi
  tidak lolos safety handoff: voxel pantulan sendiri menyebabkan clearance
  negatif. Toleransi primitive 8 mm lebih kecil daripada noise jarak
  simulator 10 mm. Toleransi menjadi 50 mm; tes tetap memastikan objek
  eksternal dekat badan di ruang tidak terisi tidak ikut dihapus. Toleransi
  ini khusus simulasi, bukan kalibrasi MID-360 hardware.
- **Hasil berikutnya:** run `20261013_175016_30048` masuk hover rolling dan
  kamera merespons target pitch 0,4 rad dengan feedback sekitar 0,397 rad.
  Tetapi setup menunjukkan rangefinder tidak valid karena visual baterai
  baru berada pada arah pandang pemasangan lama.
- **Rangefinder:** dipindahkan ke (0,115; 0; -0,07) m, di luar tray/baterai.
  Koreksi tinggi memakai proyeksi XYZ mount dan sikap badan, bukan offset
  tetap 5 cm. Rumah visual sensor kemudian diposisikan di belakang optical
  origin, tidak pada jalur sinar bawah. Pemeriksaan langsung mengonfirmasi
  range tersedia. Error daemon `ros2 topic echo` dipisahkan dari kegagalan
  sensor; checker rclpy langsung tetap dapat membaca sensor.
- **TF statis:** saat menambah rangefinder, dua pemanggilan broadcaster
  membuat subscriber yang terlambat bergabung kehilangan map→odom.
  Run `20261013_181249_34139` punya range valid tetapi map_version=0.
  Kode Python broadcaster Humble hanya melatch TFMessage terakhir.
  Semua transform kini dipublikasikan sekaligus; tes regresi ditambahkan.
- **Baseline berhasil:** run `20261013_181657_35513` lulus semua setup
  checks, takeoff READY, hover rolling >34 s simulasi dan >2 putaran tanpa
  failure. Altitude settled sekitar 1,950–1,958 m terhadap target 2 m;
  error sekitar 5 cm tetap merupakan keterbatasan estimasi/alignment,
  bukan dihapus dengan mengganti label frame. Audit dihentikan pada t=77,5
  dengan 95 sampel hover; simulator kemudian ditutup untuk perubahan baru.
- **Perubahan tinggi:** pengguna meminta LiDAR lebih tinggi agar lebih
  mudah melihat atas. Pivot dinaikkan dari Z=-0,035 ke Z=0,085 m (+12 cm),
  beserta bracket/sumbu pada model, TF dan konfigurasi. Viewer web memakai
  scene yang sama. Cakupan atas masih bergantung sudut roll, bukan selalu
  seluruh bola terlihat. Run `20261013_182902_37801` hover >37 s dan
  >2,35 putaran tanpa failure; setup lulus. Itu belum mengesahkan mutu
  penghalangan sensor.
- **Temuan audit gambar:** pengguna mempertanyakan point cloud/voxel di
  belakang drone. Peta gabungan sepanjang putaran memang dapat mencakup
  belakang melalui sinar yang lolos atas/samping/bawah. Tetapi audit
  perpotongan sinar inti badan menemukan masalah tambahan: pada 41 scan
  raw, 32277 finite return melampaui exit inti badan dan 29355 valid return
  tidak tersaring sebagai self. Pemeriksaan awal hanya no-return terlalu
  lemah (tidak ada +inf menembus badan) dan diperketat; hasil strict FAIL.
  Kemungkinan terkait rendering/near clipping; penyebab internal renderer
  belum dikonfirmasi, sehingga tidak diklaim sudah diketahui persis.
- **Solusi persepsi:** sensor gate khusus model baru memakai perpotongan
  ray terhadap geometri badan sebelum publikasi /livox/lidar dan sinar
  mapper. Ray terhalang dijadikan invalid, tidak menjadi ruang bebas di
  belakang badan dan tidak menjadi titik obstacle eksternal. Box exact,
  cylinder memakai box pembungkus konservatif, camera memakai swept
  envelope. Envelope LiDAR pada optical origin tidak dipakai menghalangi
  dirinya. Batang dudukan diperpendek agar tidak melintasi optical origin.
  Scan menunggu TF timestamped singkat dan gagal tertutup jika tidak dapat
  diverifikasi. Konfigurasi lama tidak mengaktifkan guard baru ini.
- **Tes sebelum rerun:** sembilan tes visual/roll dan 27 tes adapter ROS
  lulus; regresi sinar rear terhalang meskipun endpoint jauh, upward ray
  tetap lolos, batching TF dan proyeksi rangefinder. Ini tidak berarti
  seluruh occlusion arm/leg/model telah divalidasi empiris.
- **Bukti:** `reports/visual50_hover/flight_audit*.jsonl`, setup JSON terkait,
  `raised_body_occlusion_strict.json`, serta folder run yang tercatat.
  Run baru `20261013_184111_40095` sedang diuji dengan guard shadow ketika
  entri ini ditulis; hasilnya belum diklaim berhasil.
- **Cara menjalankan:** `bash tools/start_visual50_hover.sh`; terminal lain
  `source tools/ros_environment.sh`, `.venv/bin/python tools/set_origin.py --timeout 180`,
  lalu `python3 tools/start_mission.py --timeout 180`. Untuk audit:
  `python3 tools/audit_rolling_hover.py --output reports/visual50_hover/new_audit.jsonl --seconds 400`.
  Audit penghalangan pada input mapper:
  `python3 tools/audit_body_occlusion.py --topic /livox/lidar --output reports/visual50_hover/new_shadow_audit.json --seconds 90`.
- **Batas misi:** konfigurasi ini khusus takeoff-hover-return-land eksperimen;
  inspeksi pohon penuh dengan kamera kiri dan lokalisasi sensor belum
  diaktifkan/tervalidasi. Terrain-following dan depth navigation dinonaktifkan
  pada uji hover. Konfigurasi/launcher inspeksi lama tetap tersedia.
  Semua posisi GT; belum hardware. Tidak commit/push.

## 13 Oktober 2026 — Hasil akhir raised-LiDAR dengan guard penghalangan badan

- **Run:** `20261013_184111_40095`. Pada sampel t=58,5, drone berada di
  MISSION/SURVEY, armed, tanpa failure; durasi setelah handoff hover
  33,55 detik simulasi dan 2,136 putaran roll. Altitude terakhir 1,95733 m,
  tracking error 0,04349 m terhadap target sekitar 2 m. Galat sekitar
  4–5 cm belum dihilangkan. Ini keberhasilan hover eksperimen, bukan
  validasi misi inspeksi lengkap atau lokalisasi LiDAR.
- **Setup:** `setup_shadow_fixed.json` lulus seluruh pemeriksaan termasuk
  rangefinder, map freshness, alignment FCU, serta publisher kontrol tunggal.
- **Audit occlusion sesudah guard:** pada /livox/lidar, 134 scan, 138047 ray
  terhalang inti badan, semuanya invalid; tidak ada positive no-return,
  finite return menembus exit badan, atau self-return valid lolos filter.
  TF failures nol pada audit ini. `body_occlusion_shadow_fixed.json` PASS.
  Raw /simulation/lidar tetap menggambarkan keterbatasan backend renderer;
  mapper/perception mengonsumsi /livox/lidar yang telah melewati guard.
  Pemeriksaan inti badan tidak membuktikan seluruh arm/leg/moving mount
  sudah divalidasi empiris. Cylinder memakai bounding box konservatif dan
  camera memakai envelope, sehingga sebagian ray bebas dapat ikut ditahan
  secara aman. Guard ini hanya untuk model simulasi khusus, bukan untuk
  driver hardware yang belum diuji.
- **Ruang atas:** snapshot map t=40,008 memuat 128 cell observed-free
  dengan radius XY <0,6 m dari origin dan Z=3,3–4,1 m, di atas prior
  launch-pad yang berhenti pada Z<3,2 m. Ini bukti pengamatan simulasi ke
  atas; tidak menggantikan pemeriksaan footprint/braking lintasan naik.
- **Penjelasan gambar:** occupied oranye mencakup tanah dan objek, bukan
  seluruhnya pantulan drone. Peta adalah akumulasi sepanjang roll. Badan
  hanya menutup subset ray; belakang dapat teramati dari atas/samping/bawah.
  Pada snapshot ini fraction volume teramati hanya 0,012757 dari bounds
  global, sehingga gambar tidak berarti seluruh kebun telah dipetakan.
- **Handoff:** pengamat read-only dihentikan setelah bukti tercapai;
  Gazebo/RViz dan simulator tetap menyala. Hover config 300 s simulasi
  sebelum return/land; landing dan navigasi antar pohon tidak divalidasi
  dalam run ini. Tidak commit atau push.

## 2026-10-13 — Scanning kontinu dan perubahan roll menjadi 180°/detik

- **Kronologi:** setelah hover model baru berhasil, pengguna meminta scanning
  tanpa berhenti di setiap viewpoint. Dibuat controller terpisah yang memakai
  jalur melingkar CCW, 24 anchor dan 6 sektor inspeksi; kamera kiri diarahkan
  ke pohon. Controller lama tetap tersedia. Jalur menggunakan map terukur,
  pengecekan collision dan braking, bukan layout pohon sebagai collision map.
- **Batas implementasi:** chunk tetap berakhir diam jika lookahead belum aman;
  smoothing bisa memperlambat/berhenti pada knot jika diperlukan. Ini bukan
  jaminan orbit tanpa berhenti pada semua kondisi. Konfigurasi percobaan ini
  approach dan orbit pada sekitar 2 m; prosedur naik ke pucuk batang versi
  pitch-LiDAR lama belum dipindahkan ke mode rolling ini.
- **Uji kode:** 48 tes continuous/tree/survey lolos. Setelah perubahan roll,
  14 tes model/front-roll/continuous-tree dan 8 tes mount ROS lolos.
- **Perubahan berikutnya:** pengguna meminta roll 180°/detik. Run
  `ros_ground_truth_palm_farm_20261013_191150_44293_mapping` dihentikan ketika
  masih fase takeoff (belum ada kegagalan tercatat). Generator visual50 kini
  memakai pi rad/s; limit joint mengikuti rate yang diminta. Config menerima
  maksimal pi rad/s hanya untuk mode simulasi opt-in. Ditambahkan pengukuran
  angular rate dari selisih sudut joint bertimestamp, selain commanded rate.
- **Bukti sementara:** pada run
  `ros_ground_truth_palm_farm_20261013_191507_45540_mapping`, t=7,3 s,
  commanded rate=3,14159265359 dan measured rate=3,14159265359 rad/s.
  Log `reports/continuous_inspection/flight_audit_02_roll180.jsonl`.
  Putaran aktual simulator sudah sesuai; keberhasilan dua pohon dan landing
  masih diuji. Simulasi GT, bukan validasi lokalisasi LiDAR atau hardware.
- **Audit ray setelah perubahan rate:** 44 scan pada `/livox/lidar`, 47.970
  ray yang arah geometrinya memotong core badan semuanya invalid, bukan free
  atau occupied. Tidak ada finite hit di belakang badan yang lolos guard.
  Satu scan TF belum tersedia; audit ini hanya core badan, bukan keseluruhan
  geometri. Bukti `reports/continuous_inspection/body_shadow_roll180.json`.
- **Hasil penerbangan sementara:** pada t=155,75 s, pohon ID 4 telah selesai
  (6 sektor). Lima sektor interior tercatat pada speed sekitar 0,125–0,291
  m/s; sektor terakhir di ujung orbit berhenti sebelum pergantian pohon.
  Orbit terbagi dua chunk (anchor 0–8 lalu 9–23); ada jeda aman antar chunk,
  bukan satu putaran tanpa jeda. Drone sudah approach ke pohon terukur ID 5,
  target `[5.032165,7.063950,2.0]`. Snapshot saat itu tidak menunjukkan failure;
  **audit riwayat lengkap berikutnya menemukan failure sementara sebelumnya**.
  `inspection_valid` memeriksa heading, pitch feedback dan fresh map;
  bukan validasi kualitas gambar atau identifikasi bunga. Kamera dibuka
  atas permintaan pengguna, GUI Gazebo ditutup tanpa menghentikan server.
  Dua pohon plus landing masih belum selesai. Tidak commit/push.

### Koreksi audit lengkap dan penambahan naik sebelum orbit

- **Kronologi:** drone selesai scanning ID 4 lalu ID 5 dan masuk RETURN.
  Namun pengamat yang sebelumnya berhenti pada failure pertama ternyata
  mencatat `route_changed` pada t=141,8 s. Navigation debug memuat delapan
  snapshot BRAKE mulai t=141,65 s; kemudian controller memulihkan diri dan
  melanjutkan. Laporan snapshot “tanpa abort” tidak mencakup riwayat ini.
- **Tindakan:** run dihentikan saat RETURN, terakhir t=286,95 s; RViz dan
  kamera ditutup. Landing tidak selesai diuji. Dua scan berhasil, tetapi run
  tidak boleh dilabelkan bebas kegagalan.
- **Hipotesis yang diperiksa:** pemeriksaan seluruh orbit dapat menolak bagian
  yang sudah dilewati ketika rolling window bergeser. Pada kejadian pertama
  window X=[1,8;11,8] sedangkan sisi timur orbit sebelumnya sekitar X=11,0,
  dekat border yang diinflasi. Penyebab asli belum direplay dengan map exact
  saat kejadian; jangan menyatakannya sebagai diagnosis final.
- **Perbaikan:** eksekusi continuous trajectory kini menguji seluruh **sisa**
  kurva dengan subdivision de Casteljau exact, bukan membatalkan lintasan
  karena bagian lampau. Perencanaan/acceptance tetap menguji seluruh rute;
  body clearance dan brake aktual tetap diperiksa setiap tick. Ditambahkan
  log whole-curve vs remaining-curve. Tes memastikan blocker di masa depan
  tetap ditolak, sedangkan blocker pada bagian lampau tidak membatalkan sisa.
- **Masukan pengguna:** approach di bawah tajuk, naik vertikal sebelum orbit.
  Mode rolling kini mengaktifkan ContinuousAscent yang sudah tersedia,
  tanpa tilt/normalization LiDAR. Speed naik 0,12 m/s, reaction reserve 0,7 s,
  map max-age 0,4 s. Gap nominal atas envelope ke obstacle 0,6 m; sebagian
  sudah tercakup clearance/tracking margin, sisanya menjadi observation guard.
  Quantisation voxel dan pengereman dapat menghentikan lebih jauh. Referensi
  XY tetap, map/braking tetap aktif; unknown atau cap tinggi bukan bukti sudah
  mencapai daun. Kamera pitch kiri dan orbit kontinu baru dimulai setelah naik
  selesai. Ini bukan mengimplementasikan penuh paper FASTER, hanya memakai
  prinsip stopping fallback pada ruang known-free.
- **Uji:** 36 tes continuous/tree/ascent lolos. Flight dengan ascent belum
  dijalankan pada saat entri ini. Pengamat sekarang mempunyai opsi `--stop-run`
  agar failure sementara tidak terlewat; stop hanya registered private run.
  Launch berikut membuka RViz dan kamera, bukan GUI Gazebo sesuai permintaan.

## 2026-10-13 — Baseline lokalisasi sensor untuk LiDAR rolling

- **Permintaan:** hentikan simulasi GT, implementasikan lokalisasi sensor dan
  lanjutkan loop. Run GT `ros_ground_truth_palm_farm_20261013_195357_52149_mapping`
  dihentikan beserta RViz/kamera. Tidak mengklaim hasil GT sebagai LIO.
- **Masalah:** estimator lama fixed-extrinsic tidak mendukung LiDAR rolling.
  IMU pada MID ikut berputar; rotasinya tidak boleh dianggap rotasi badan.
- **Implementasi:** sensor IMU terpisah rigid di body origin (simulasi IMU
  Pixhawk), topic `/simulation/body_imu`, serta measured dynamic transform
  base_link→mid360 pada timestamp scan. Estimator ICP–IMU memakai body IMU;
  rotasi mount hanya mengubah point cloud, bukan gravitasi/gyro body.
  Sensor internal MID tetap diterbitkan tetapi bukan input estimator rolling.
- **Eksklusivitas:** sensor-mode localisation tidak subscribe ground-truth.
  Peta/navigation/ExternalNav menerima odometry sensor. Ground truth hanya
  dipakai observer read-only untuk pasangan timestamp dan error; bukan koreksi.
  Posisi awal konfigurasi adalah prior surveyed launch gauge, bukan oracle
  posisi bergerak. Frame ENU/FLU dan konversi MAVROS yang ada dipertahankan.
- **Batas:** ini ICP–IMU eksperimental, bukan FAST-LIO/SLAM ber-loop-closure.
  Gazebo menghasilkan scan instant raster, bukan MID-360 nonrepetitive sweep
  dengan per-point timing. Kalibrasi/noise IMU hardware belum divalidasi.
- **Tes:** 30 navigation/tree/front-roll, 28 ROS adapter/mount, dan 3 tes
  rolling-localisation lolos. Tes badan diam/LiDAR berputar tidak menghasilkan
  gerak badan; body gravity tidak ikut diputar extrinsic LiDAR.
- **Loop bertahap:** fixture terpisah `hover` (takeoff/hover 20 s/landing)
  terlebih dahulu; `inspection` tetap tersedia untuk tahap berikut. Dua mode
  GT/sensor terpisah; estimator mengeluarkan `/localisation/debug` (RMSE,
  reason, scan/IMU stamp, reference count, mount transform readiness).
  Script audit sensor menulis error posisi/orientasi pada measurement-time.
  Uji penerbangan baru dimulai; belum dinyatakan lolos.
- **Git:** mulai paket perubahan ini commit/push kembali diaktifkan atas
  permintaan pengguna ke remote `depth_drone_gz`, branch `rotating-lidar`.
  File kredensial dan asset map milik agen lain tidak dimasukkan.

### Loop lokalisasi 01 — Gagal sebelum takeoff

- **Run:** `ros_sensor_palm_farm_20261013_203210_57758_mapping`.
  Setup menunjukkan sumber lokalisasi eksklusif benar (LiDAR + body IMU,
  tanpa subscription ground truth), tetapi healthy=false dan pose/ExternalNav
  berhenti. Misi tidak di-arm atau dimulai; run dan jendela dihentikan.
- **Gejala:** bootstrap memakai satu scan dengan 2.805 titik; setelah FOV
  berubah saat roll, `registration_rejected` lalu `poor_overlap`. Pada t=18 s
  p estimator `[0.746,-0.041,-1.274]` meskipun drone masih di pad.
- **Audit:** sampel body IMU frame base_link, gyro sekitar 0,001 rad/s dan
  gravitasi Z=9,8066; IMU quaternion orientation tidak dipakai estimator.
  Hipotesis utama: sparse raster correspondence/partial FOV pada reference
  satu scan memiringkan estimasi dan kemudian integrasi gravitasi membuat
  drift. Belum dinyatakan terbukti hanya dari satu sampel IMU.
- **Perbaikan:** stationary-disarmed bootstrap mengumpulkan 2,2 s (lebih
  satu putaran) sebelum healthy. Rolling estimator memakai point-to-plane
  robust residual dengan IMU orientation prior; fixed-mount mode lama tetap
  point-to-point. Ground truth tidak dipakai untuk bootstrap/correction.
  Debug ditambah orientation_rpy dan velocity. Uji ulang masih diperlukan.
- **Git:** paket awal sudah commit/push `03bc194` pada rotating-lidar.

### Loop lokalisasi 02 — Posisi stabil, velocity drift saat disarmed

- **Run:** `ros_sensor_palm_farm_20261013_204026_59440_mapping`.
  Setelah full-roll bootstrap, t=15,3 s posisi error terukur 0,007322 m,
  orientation error 0,000539 rad, sehat dan tracking. Namun debug velocity
  sekitar `[-0.045,0.019,-0.119]` m/s meskipun badan diam di pad.
- **Masalah:** IMU propagation memakai gravitasi tetap 9,81, sedangkan
  simulasi nominalnya sekitar 9,8; correction ICP memperbaiki pose tetapi
  velocity lama tidak dikoreksi. Ini membuat twist ExternalNav keliru.
- **Solusi:** kalibrasi stationary gravity magnitude dan gyro bias pada body
  IMU; accepted scan innovation juga mengoreksi velocity dengan gain bounded.
  Angular twist memakai gyro body terkalibrasi, bukan selalu nol.
- **Tindakan:** run dihentikan sebelum arm. Tes baru memastikan IMU diam
  dengan gravitasi 9,8 dan bias gyro tidak menghasilkan gerak. Kalibrasi ini
  masih prototype stationary, bukan estimator bias EKF FAST-LIO penuh.
  Commit sebelumnya `dd10c0f` sudah dipush. Uji ulang setelah koreksi diperlukan.

### Loop lokalisasi 03 — Estimator sehat, preflight tertahan beda timestamp

- **Run:** `ros_sensor_palm_farm_20261013_204702_60922_mapping`.
  T=23,6 s, pose error 0,005803 m, orientation error 0,000850 rad,
  velocity error 0,009456 m/s. Gravity hasil init 9,799425 m/s².
  Healthy/source-exclusive/fresh/ownership/ExternalNav checks lolos.
- **Penghambat:** fixed FCU alignment belum tersedia. Capture lama mensyaratkan
  FCU latest stamp dengan sensor stamp berbeda <0,05 s, sementara ICP pose
  mempunyai processing age sekitar 0,15 s. Ini bukan alasan mengubah label
  frame atau memakai posisi truth.
- **Perbaikan:** simpan history pose FCU; interpolasikan position dan wrapped
  yaw pada measurement time estimator sebelum numerical FCULocalAlignment.
  Tidak extrapolate, tidak mengubah batas pose age atau discrepancy .35 m.
  Capture tetap disarmed dan stationary. Tes yaw ±179° dan missing history
  memastikan pairing tidak menciptakan pose masa depan.
- **Tindakan:** run dihentikan sebelum arm, ulangi setelah commit/push.
  Commit sebelumnya `a5cba0b` sudah dipush. Landing sensor belum diuji.

### Loop lokalisasi 04 — Takeoff berhasil, drift horizontal saat hover

- **Run:** `ros_sensor_palm_farm_20261013_220240_67910_mapping`.
  Seluruh setup check lolos, kemudian takeoff sensor-only berhasil. T=34,3 s
  posisi error 0,011369 m; t=46,1 s membesar menjadi 0,064052 m, kemudian
  t=50,1 s 0,211526 m. Observer mengukur 350 pasangan measurement-time,
  RMSE posisi 0,040181 m. Navigasi gagal `tracking_error`; watcher otomatis
  menghentikan run dan owned viewers. Hover/landing tidak dinyatakan lolos.
- **Audit:** posisi, orientasi dan velocity bukan ground truth input. Error
  pose masih dapat bertambah meski RMSE registration terlihat rendah; itu
  ukuran kecocokan ke map estimator, bukan error terhadap posisi sebenarnya.
- **Hipotesis:** scan-to-map update setiap scan membentuk feedback drift;
  `voxel_downsample(...)[-40000:]` juga memilih ekor voxel terurut, bukan
  spatially balanced history. Normal/korespondensi dapat menjadi tidak merata.
- **Perbaikan:** frozen measured bootstrap anchor dan keyframes tiap .5 m
  perpindahan, bukan insert setiap roll/yaw/hover. Bounded map memakai sampling
  merata pada seluruh indeks, bukan memotong satu sisi. Debug menambah jumlah
  keyframe dan eigenvalues informasi translasi. Tes stationary rolling
  memastikan reference tidak berubah ketika badan tidak bergerak.
- **Status:** perbaikan perlu flight ulang. Tidak memakai ground truth untuk
  membekukan/correct map; tidak menambah toleransi error agar test lolos.
  Commit sebelumnya `47dbcbb` sudah dipush.

### Loop lokalisasi 05 — Drift berkurang, reject memakai metrik yang salah

- **Run:** `ros_sensor_palm_farm_20261013_221409_70014_mapping`.
  629 pasangan timestamp, RMSE posisi 0,012115 m, max 0,041641 m.
  Takeoff dan transisi menuju hover tercapai; pada t=77,3 s registration
  ditolak dan navigasi melaporkan localisation_loss. Watcher menghentikan run.
- **Audit:** error pose terakhir 0,008676 m dan error velocity 0,037722 m/s,
  tetapi Euclidean nearest-point RMSE=0,182531 melewati threshold .18.
  Estimator sudah memakai point-to-plane untuk solve, namun acceptance masih
  memakai jarak ke titik sampel terdekat. Raster roll mengubah lokasi sampel
  pada permukaan yang sama; jarak tersebut bukan residual permukaan.
- **Perbaikan:** body-IMU mode memakai point-to-plane residual RMSE .08,
  dengan overlap, minimal 80 plane constraints, information eigenvalue .003,
  translation/rotation innovation gates tetap berlaku. Euclidean RMSE tetap
  dilog, tidak dihilangkan. Tes resampling permukaan sama menguji perbedaan
  jarak sampel dan error permukaan.
- **Freshness:** rejected scan tidak diterbitkan. Healthy hanya dapat memakai
  pose accepted sebelumnya dalam batas pose_timeout yang sudah ada (.3 s)
  serta IMU live .1 s; tidak menerbitkan prediction dengan timestamp palsu.
  Satu rejected scan tidak langsung membatalkan accepted pose yang masih fresh.
  Dropout berlanjut tetap membuat pose stale dan menghentikan navigasi.
- **Status:** flight ulang diperlukan; landing belum lolos. Commit sebelumnya
  `eb32e43` telah dipush. Ini bukan melonggarkan error pembanding agar test lolos.

### 2026-10-13 — Loop sensor tanpa jendela GUI

- **Permintaan:** simulasi berikutnya tidak membuka window apa pun.
- **Perubahan:** launcher sensor mematikan RViz, Gazebo GUI, dan camera viewer;
  server fisika, sensor, navigasi, serta audit tetap berjalan.
- **Uji:** konfigurasi launcher diperiksa dengan `bash -n`.
  Run hover 06 yang sudah berjalan tetap diaudit; run berikutnya headless.
- **Uji perbaikan Loop 05:** 25 tes estimator/navigasi dan 29 tes adapter ROS
  lulus. Tes ROS memerlukan jaringan lokal di luar sandbox; kegagalan init
  sandbox bukan kegagalan algoritma.

### 2026-10-13 — Loop sensor 06: hover lolos, impuls landing merusak velocity

- **Kronologi:** drone berhasil takeoff, hover, kembali ke home, dan turun
  sampai kaki menyentuh tanah. Pada t=100,4 s watcher melihat error velocity
  1,648 m/s, lalu lima sampel berturut-turut melewati batas .15 m/s dan
  menghentikan run sebelum landing/disarm selesai. Tidak diklaim COMPLETE.
- **Bukti:** `reports/sensor_localisation/audit_06_hover.summary.json`,
  run `ros_sensor_palm_farm_20261013_223251_74310_mapping`.
  924 pasangan, RMSE posisi .013064 m, maksimum .045997 m; posisi saat
  benturan tetap akurat. Kecepatan palsu meluruh .8 per scan karena koreksi
  innovation lemah terhadap impuls IMU yang terintegrasi pada kontak tanah.
- **Perbaikan:** setelah percepatan IMU badan >30 m/s², selama .5 s koreksi
  velocity memakai selisih dua posisi scan accepted dengan gain .8.
  Tanpa asumsi ground contact, tanpa pose/velocity GT, tanpa melonggarkan
  batas watcher. Pose tetap harus lolos registration normal. Mode normal
  tetap memakai IMU + innovation. Peak impuls dan sumber velocity dilog.
- **Uji:** 7 tes rolling localisation lulus, termasuk impuls sintetis yang
  sengaja alias dan recovery velocity <.15 m/s mulai scan kedua.
  Tes flight ulang masih diperlukan; baseline berikutnya tanpa GUI.

### 2026-10-13 — Loop sensor 07: impuls kontak terlewat sampel IMU

- **Kronologi:** takeoff, hover, RETURN, dan LAND kembali berjalan. Saat kaki
  menyentuh tanah t=89,5 s, posisi tetap akurat tetapi velocity masih turun
  sekitar .45 m/s. Lima error velocity berturut-turut memicu watcher.
  Run berhenti sebelum disarm, belum COMPLETE.
- **Bukti:** `audit_07_hover.summary.json`: 845 pasangan, RMSE posisi .013955 m,
  max .036963 m. `imu_shock_peak_mps2=0` dan recovery khusus tidak aktif;
  impuls fisika tidak selalu muncul pada sampel sensor 200 Hz.
- **Perbaikan:** dua scan accepted berturut-turut dengan selisih velocity
  scan-displacement terhadap IMU >.3 m/s juga mengaktifkan recovery .5 s.
  Tidak menganggap drone di tanah dan tidak membaca GT; seluruh scan harus
  lolos overlap/residual/information gates. Displacement, disagreement,
  jumlah scan, dan sumber update velocity dilog. Batas audit tetap sama.
- **Uji:** 8 tes estimator lulus, termasuk kontak tanpa sampel IMU besar.
  Landing/disarm harus diuji ulang sebelum melanjutkan dua pohon.

### 2026-10-13 — Loop sensor 08: landing/disarm berhasil, konfirmasi range tertinggal

- **Kronologi:** drone takeoff, hover, RETURN, LAND, menyentuh tanah, lalu
  disarm tanpa failure. Error velocity setelah kontak ~.007 m/s; recovery
  berhasil. Namun FSM tetap LAND, bukan COMPLETE. Run dihentikan saat motor
  sudah disarm untuk audit konfirmasi, bukan dianggap misi COMPLETE.
- **Bukti:** run `ros_sensor_palm_farm_20261013_225830_80585_mapping`,
  `audit_08_hover.jsonl` dan summary. Range aktual di ROS .125 m valid.
- **Diagnosis:** range_cb meminta TF pada waktu pengukuran range; estimator
  LiDAR menerbitkan TF 10 Hz dengan delay, sehingga callback dapat meminta
  waktu yang lebih baru dari TF terakhir dan membuang pengukuran. Ini berbeda
  dari sensor range gagal atau drone belum landing.
- **Perbaikan:** antrekan 12 range valid, retry dari tick saat TF pengukuran
  tersedia. Tidak memakai TF latest atau mengganti timestamp. Range melewati
  range_timeout dibuang; transform mount/attitude tetap dihitung numerik.
  Log range vertical, umur pengukuran, jumlah antrean dan TF waits ditambah.
- **Uji:** 30 tes ROS lulus, termasuk TF terlambat, timestamp tidak berubah,
  range stale tidak dihidupkan kembali, serta proyeksi mount terelokasi.
- **Berikutnya:** langsung uji dua pohon sensor tanpa GUI; juga memverifikasi
  range freshness dan COMPLETE setelah landing. Belum klaim full flight lolos.

## Cara memperbarui jurnal

Tambahkan entri setiap masalah atau perbaikan bermakna dengan format:
tanggal → kronologi penemuan → masalah → diagnosis → solusi → hasil uji →
bukti → status. Dalam kronologi, jelaskan apa yang sudah berhasil dilakukan
drone, apa yang hendak dilakukan berikutnya, gejala yang muncul, dan bagaimana
audit membedakan gejala itu dari penyebabnya.
Jika diagnosis belum pasti, tulis sebagai hipotesis. Jika tes belum dijalankan,
tulis demikian. Jangan menimpa kegagalan lama dengan klaim keberhasilan baru.
Direktori `reports/` adalah bukti lokal yang umumnya diabaikan Git; jurnal
dan dokumentasi ini disimpan di repository, sedangkan file log tidak otomatis
tersedia di GitHub.
