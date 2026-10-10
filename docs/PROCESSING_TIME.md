# Log waktu pemrosesan

`/navigation/status` dan `map_3d/navigation_debug.jsonl` sekarang memiliki
`processing_pipeline`. Terminal mencetak `Processing timing` setiap 5 detik
simulasi. `last.total_wall_ms` mengukur satu scan yang berkorelasi: diterima
callback ROS pemetaan → masuk worker → integrasi voxel dan inflation → hasil
worker → pengiriman/decode snapshot → publikasi pertama setpoint final yang
memakai peta tersebut untuk pemeriksaan keselamatan. Tahap-tahap tercatat dalam
`last.stages_wall_ms`. P50/P95 memakai maksimal 2048 sampel terakhir; maksimum
dan jumlah sampel mencakup seluruh run. Satu versi peta dihitung sekali.

Ini waktu nyata monotonic Linux pada host yang sama, bukan waktu simulasi.
`measurement_to_publish_sim_seconds` terpisah: umur timestamp sensor menurut
jam ROS. Jangan mencampur atau menjumlahkan kedua jenis waktu. Instrumentasi
dimulai pada callback ROS pemetaan, bukan saat Gazebo memancarkan sinar.
Tidak mengukur pengiriman MAVROS ke FCU atau respons pesawat. Referensi jalur
dan solusi MPC bisa berasal dari snapshot sebelumnya; log ini mengukur
pemakaian peta terbaru untuk keselamatan, bukan mengklaim setiap scan memicu
perencanaan baru. Saat tidak ada setpoint final, tidak ada sampel total baru.
Scan yang diganti antrean latest-only juga tidak dihitung sebagai selesai.

`planner_stages_last_wall_ms` memisahkan pemilihan goal/pencarian rute (langsung
atau A* dan batch), pencarian sudut observasi jika perlu, pembuatan lintasan
halus, collision checking, serta sertifikasi pengereman. Dua entri
`braking_sample_checks` dan `braking_attempts` adalah jumlah, bukan milidetik.
`worker_total` adalah kerja satu tugas planner; `submit_to_result` juga
mencakup antrean, serialisasi, transfer hasil dan waktu menunggu polling.

Planner dijalankan ketika lintasan baru diperlukan, bukan setiap setpoint.
Sertifikasi memeriksa lintasan setiap 0,05 detik waktu lintasan dan mencoba
hingga 12 kali jika perlu memperlambat lintasan. Lintasan lambat memiliki durasi
panjang sehingga jumlah sampelnya dapat besar. Angka maksimum lama 783 ms
mencakup seluruh tugas ini, bukan A* saja; tanpa breakdown lama, tahap yang
paling mahal pada run lama tidak bisa dipastikan. Tidak ada safety gate,
kecepatan, atau deadline yang dilonggarkan oleh perubahan logging ini.

```bash
source tools/ros_environment.sh
ros2 topic echo /navigation/status
```

Log baru tersedia setelah simulasi berikutnya; log historis tidak diubah.
