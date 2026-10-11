# Audit kegagalan pengereman

`navigation_debug.jsonl` dan `/navigation/status` memuat
`speed_diagnostics.stopping_checks`. Untuk setiap faktor kecepatan yang dicoba,
log menyimpan posisi, kecepatan, percepatan uji, alasan lulus/gagal, segmen
reaksi, serta interval dan kotak lintasan pengereman yang ditolak.

`blockage` menjelaskan satu sel yang ditolak setelah penambahan ukuran badan
dan margin keselamatan. `causes` bisa memuat lebih dari satu penyebab:

- `occupied`: obstacle terukur di daerah badan/margin.
- `unknown`: ruang belum teramati di daerah badan/margin.
- `outside_window`: badan/margin keluar batas peta lokal.
- `inflated_mask_without_raw_blocker`: mask dan sel mentah tidak konsisten;
  perlu audit terpisah, bukan dianggap bebas.

Koordinat sel penyebab, hitungan, contoh maksimum delapan occupied dan delapan
unknown, versi peta dan timestamp observasi disimpan. Contoh dibatasi untuk
menghindari log besar. Sel pusat lintasan bisa FREE tetapi tetangganya membuat
badan drone belum aman. Hitungan `raw_*` dalam kotak lintasan bukan hitungan
seluruh daerah badan; gunakan `blockage.footprint_*` untuk melihat tetangga.

`witness_policy` membedakan supercover garis dan kotak konservatif kurva
Bezier. Kotak kurva yang menyentuh sel blocked **bukan bukti** bahwa kurva atau
drone benar-benar menabrak; itu bukti sertifikasi konservatif gagal.

Keputusan gagal kini menimpa rincian tick sebelumnya sebelum status BRAKE,
sehingga `braking_scale=0` dan semua percobaan gagal tetap terekam. Pemeriksaan
keselamatan, kecepatan dan margin tidak dilonggarkan. Log lama tidak dapat
dilengkapi retroaktif: perlu simulasi ulang untuk mengetahui saksi kegagalannya.

Tes sintetis mencakup occupied/unknown pada tetangga yang terinflasi, batas
peta, kotak kurva konservatif, tidak memutasi occupancy, dan empat percobaan
kecepatan gagal yang tersimpan. Ini bukan validasi penerbangan baru.
