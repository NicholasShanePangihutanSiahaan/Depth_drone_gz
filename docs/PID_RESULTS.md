# Hasil sementara PID — 10 Oktober 2026

Snapshot MPC telah dipush ke `lidar360-mpc` (b68c83e). Controller PID dan
presetnya berada di `lidar360-PID`; default mapping tidak menjalankan MPC.
Branch default remote dan proyek asli tidak diubah.

## Perubahan dan pengujian

- PID menambahkan koreksi percepatan berbasis error posisi, integral, dan
  kecepatan. Batas percepatan/jerk, pemeriksaan pengereman, peta unknown,
  pemeriksaan sensor basi, dan satu publisher MAVROS tetap dipertahankan.
  Tes PID: tujuh kasus, termasuk saturasi, anti-windup, unknown dan input invalid.
- Run pertama berhasil takeoff, lalu node mati ketika status PID memuat
  `numpy.bool_` yang tidak dapat diserialisasi ke JSON. Boolean sekarang
  dikonversi menjadi tipe Python; tes memeriksa JSON pada keluaran normal
  dan tersaturasi. Run tersebut dihentikan sebelum pengujian ulang.
- ROS launch kini meminta shutdown jika node navigasi keluar, sehingga
  launcher menutup proses dan GUI miliknya. Konstruksi launch dan event
  handler telah diperiksa. Penghentian akibat crash belum diuji dengan
  sengaja pada run kedua; perubahan berlaku pada peluncuran berikutnya.
- Analisis log lama tidak lagi menganggap kecepatan terukur yang tidak
  tersedia sebagai nol.

Pengujian terakhir: 173 kasus paket navigasi lulus tanpa environment ROS,
empat dilewati; suite adapter dengan ROS lulus 17 kasus, termasuk empat
kasus yang tadi dilewati (13 lainnya tumpang tindih). Total 177 kasus unik.
Percobaan tes tambahan legacy beehive terhambat karena
`landing_command_due` tidak tersedia pada modul mission_state_machine;
hal ini belum diperbaiki dan tidak boleh disebut suite legacy lulus.

## Run kedua: penerbangan nyata dalam SITL, bukan gerak kinematik mock

`bash tools/start_pid.sh tour`: GPS-disabled, posisi ground truth,
Gazebo + ArduPilot + MAVROS, LiDAR real-time, kedua GUI dibuka.
Faktor waktu simulasi 0,2; satu detik simulasi membutuhkan sekitar lima
detik waktu nyata. Ini bukan validasi lokalisasi LiDAR.

Run: `ros_ground_truth_palm_farm_20261010_194953_38358_mapping`.
Setup lulus seluruh pemeriksaan. Pengamat dihentikan pada waktu simulasi
96,7 detik, setelah enam target dilewati, sementara penerbangan dilanjutkan.

| Hasil sampai pengamat dihentikan | Nilai |
| --- | --- |
| Target dilewati | 6 dari 326 |
| HOLD_ABORT / explore | Tidak ada |
| Kecepatan perintah maksimum | 0,297 m/s |
| Percepatan perintah maksimum | 0,107 m/s² |
| Error posisi navigasi maksimum | 0,0143 m |
| Publisher final raw / posisi | 1 / 0 |
| Prefetch diterima / ditolak | 3 / 0 |

Empat target pertama memerlukan 41,299 detik SURVEY dibanding baseline
lama 48,2 detik. Ini bukan A/B terkontrol: controller, GUI, beban CPU, dan
konfigurasi keseluruhan berbeda. Tidak membuktikan seluruh peningkatan
berasal dari profil kecepatan.

Audit debug terpisah sampai sekitar 96,7 detik: pipeline pemetaan sampai
setpoint median 1.593 ms, p95 2.138 ms, maksimum 2.815 ms **waktu nyata**.
Angka tersebut mencakup antrean/pemetaan/transport, bukan waktu PID saja;
bukan jumlah maksimum tahap independen. Planner worker sekitar 0,872–2,319
detik, mayoritas pemeriksaan pengereman. `submit_to_result` prefetch sekitar
18–19,5 detik juga memuat waktu menunggu sampai hasil diambil pada akhir
chunk, bukan waktu komputasi worker. Beban pemetaan belum hilang dengan PID.

Log lokal (tidak dipush sebagai raw data):

```text
reports/pid_run2_setup.json
reports/pid_run2_flight.json
reports/pid_run2_audit.json
reports/ros_ground_truth_palm_farm_20261010_194953_38358_mapping/map_3d/map.html
reports/ros_ground_truth_palm_farm_20261010_194953_38358_mapping/map_3d/navigation_debug.jsonl
```

Laporan pengamat adalah snapshot; log debug dan map.html terus diperbarui
selama simulasi aktif. Error pengamat dan audit berbeda karena sampling.
Clearance peta ROI bukan bukti jarak terhadap semua objek fisik dunia.

Belum tervalidasi: seluruh kebun, belokan/detour lanjutan pada run PID ini,
return/landing PID, dropout PID, kecepatan lebih tinggi dan pesawat nyata.
Penerbangan beberapa target bukan jaminan run selanjutnya tidak gagal.
Setelah penyerahan, agen tidak otomatis memantau atau memperbaiki run.

Untuk menutup simulasi beserta kedua GUI miliknya:

```bash
cd /home/abin/polinasi_lidar
python3 tools/stop_simulation.py
```
