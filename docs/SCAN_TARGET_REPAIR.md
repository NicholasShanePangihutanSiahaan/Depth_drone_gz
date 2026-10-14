# Penyesuaian target scan yang terkena daun

Opt-in `scan_target_repair` pada fixture sensor rotating. Pada awal chunk
di keadaan diam, target berikutnya dievaluasi ulang. Jika occupied, kandidat
tetap pada sudut pengamatan yang sama, radius awal hingga +1 m dan tinggi
saat ini hingga -0,6 m total dari awal scan (minimum takeoff altitude).
Kandidat harus observed-free dengan footprint/margin lengkap. Kandidat
direct lebih disukai, lalu jarak pergeseran dan penalti turun. Tidak ada
pengurangan radius atau penambahan tinggi. Sisa orbit boleh diturunkan;
ini pengecualian keselamatan, bukan scanning naik-turun yang disengaja.

Jika `scan_adaptive_height` aktif, sisa orbit tidak dikunci turun selamanya.
Lihat CANOPY_FOLLOWING_SCAN.md untuk kebijakan penggantinya.

UNKNOWN tidak mengaktifkan proyeksi target: jalur observasi yang ada tetap
berlaku. Tidak ada kandidat aman berarti hold/blocked timeout, bukan terbang
ke unknown. Endpoint baru tetap melalui A*, smoothing, batas dinamika,
braking dan final PID safety. Tidak menjamin selalu ada rute. Sudut sektor
dan jumlah enam sektor tidak berubah; rekaman kamera/peta tetap wajib.

Target nominal tidak diganti dalam log nominal; waypoint eksekusi diperbarui
bersama hasil worker yang diterima setelah live-map curve check. Log status
`scan_target_diagnostics` dan `scan_target_repairs` membedakan target yang
diusulkan dari validasi penerbangan/inspeksi. Batas `scan_target_max_outward`
dan `scan_target_max_descent` divalidasi konfigurasi.

Landasan konsep: perencanaan ulang berdasarkan pengamatan terbaru dan
pemilihan viewpoint dari kandidat aman, seperti Bircher et al. (ICRA 2016),
[Receding Horizon Next-Best-View Planning](https://github.com/ethz-asl/nbvplanner).
Implementasi ini bukan port NBVPlanner/RRT dan tidak mengoptimalkan gain
informasi: resampling lokal deterministik, dilanjutkan planner A* proyek.

Tes: `PYTHONPATH=polinasi_nav python3 -m pytest polinasi_nav/test/test_continuous_tree.py -q`.
Lihat jurnal untuk run sumber masalah dan hasil penerbangan berikutnya.
