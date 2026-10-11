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
