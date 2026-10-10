# Hasil identifikasi — 10 Oktober 2026

Gazebo/ArduPilot, arena kosong, GPS/LiDAR/mapping/kamera/rangefinder tidak
aktif. IMU dan ExternalNav **ground-truth** aktif. Bukan validasi LiDAR/hardware.

## Penerbangan lengkap

- Train: `reports/ros_ground_truth_identification_20261010_053727_25086_identification/`.
- Validasi independen: `reports/ros_ground_truth_identification_20261010_054228_26251_identification/`.
- Keduanya setup lulus, satu publisher setpoint, takeoff → gerakan uji →
  LAND → disarm, `completed=true`, tanpa failure/timing-gap tercatat.
- Train 100 detik dan validasi 43 detik gerakan dalam waktu simulasi;
  takeoff/landing di luar durasi tersebut. Faktor waktu diminta 1,0 khusus
  arena tanpa rendering sensor; langkah fisika tetap 1 ms.
- Snapshot sebelum/sesudah masing-masing berisi 1397 parameter. ATC/PSC/
  WPNAV/MOT tidak berubah, parameter pengendali kedua run sama. PID tidak
  dituning; hover learning dimatikan khusus eksperimen.
- Run awal `053116_23629` menyelesaikan gerakan dan disarm tetapi penanda
  COMPLETE masih menunggu rangefinder yang dimatikan. Log mentah tetap
  `completed=false`, **tidak dipakai fit**. Konfirmasi landing diperbaiki
  khusus arena ground-truth: LAND, disarm, posisi segar dekat lantai yang
  diketahui, dan kecepatan rendah. Navigasi kebun tidak memakai pengecualian ini.

## Parameter hasil log

Model lokal respons tertutup, per sumbu map ENU:

```
p_dot = v
v_dot = kp*(p_cmd_delayed-p) + kv*(v_cmd_delayed-v) + ka*a_cmd_delayed + bias
```

Ini koefisien model, **bukan PID FCU**. Nominal sebelumnya `[3,3,1,0]`.

| Sumbu | kp | kv | ka | bias (m/s²) | Delay efektif |
|---|---:|---:|---:|---:|---:|
| X | 1,566786 | 0,573509 | 1,045632 | 0,003786 | 25 ms |
| Y | 1,922512 | 0,397141 | 1,011433 | 0,004680 | 0 ms |
| Z | 3,579955 | 4,923624 | 0,716511 | 0,000653 | 0 ms |

Delay dipilih dari **train saja**, menggunakan prediksi mandiri 2 detik:
`hypot(RMSE_posisi, 0.25 detik × RMSE_kecepatan)`. Error percepatan satu
langkah tetap dicatat. Delay nol bukan latensi fisik nol: pencarian memakai
grid 25 ms dan dinamika model turut menyerap latensi.

## Validasi dengan gerakan berbeda

Prediksi di-reset hanya pada awal blok 2 detik/gap data. RMSE mengukur
rata-rata besar error (akar rata-rata kuadrat).

| Sumbu | RMSE posisi baru | RMSE nominal | Maksimum error baru | RMSE kecepatan baru |
|---|---:|---:|---:|---:|
| X | 1,096 mm | 1,882 mm | 4,081 mm | 1,253 mm/s |
| Y | 0,676 mm | 1,886 mm | 2,289 mm | 1,101 mm/s |
| Z | 20,053 mm | 23,307 mm | 39,734 mm | 16,840 mm/s |

Semua sumbu lulus batas prototipe: input informatif, parameter tidak pada
batas, maksimum error posisi <5 cm, RMSE kecepatan <5 cm/s, dan RMSE posisi
tidak lebih buruk dari nominal. Bukan jaminan kestabilan adaptive control.
Tracking drone terhadap perintah maksimum sekitar **3,99 cm** pada validasi;
ini berbeda dari error prediksi model.

**Belum selesai:** Z menetap sekitar 3,8 cm di bawah perintah pada validasi,
sedangkan train jauh lebih cocok. Grafik menunjukkan prediksi Z terdorong
kembali ke perintah setiap blok. Selisih frame/EKF/referensi ketinggian atau
respons kontrol belum dipisahkan secara pasti. Jangan mengubah PID, menaikkan
speed kebun, atau menyatakan model siap NMPC/hardware berdasarkan hasil ini.
Batas 5 cm adalah batas eksperimen lokal, bukan sertifikasi pengendali.

## Yang dipasang dan yang belum

- `polinasi_nav/config/identified_pva_model.json`: parameter/delay, metrik,
  dan sumber log hasil fit; `activated=false`.
- Preset identifikasi memakai file itu sebagai seed awal **observer shadow**.
  Tes ROS terisolasi memastikan parameter/delay dimuat sesuai file.
  Belum ada penerbangan tambahan dengan seed baru tersebut.
- RLS pada train menerima 558 pembaruan X, 603 Y, 357 Z. Pembaruan berhenti
  saat data/fase tidak layak. Ini bukan pengukuran massa cairan.
- Model tidak menghasilkan setpoint dan tidak dipakai misi survei.
  NMPC/adaptive flight control belum diimplementasikan/diaktifkan.
- 138 regresi lulus sebelum fit; 11 tes identifikasi lulus lagi setelah
  perubahan pemilihan model berdasarkan rollout train.
- Simulator dihentikan; original project tidak diubah.
- Saat shutdown validasi, configurator mencetak exception karena context ROS
  sudah ditutup. Terjadi setelah COMPLETE/log tersimpan, bukan kegagalan
  penerbangan. Penanganan shutdown diperbaiki; perbaikan ini belum diuji lagi
  pada shutdown simulator penuh.

Model: `reports/identification_model_20261010.json`.
Grafik dan CSV per sumbu: `reports/identification_model_20261010_comparison/`.
Dataset mentah dan parameter ada di direktori train/validation di atas.
Log terminal berada di `.dependencies/` dengan nama run yang sama.

```bash
cd /home/abin/polinasi_lidar
xdg-open reports/identification_model_20261010_comparison/validation_comparison.png
```

Panduan: [identifikasi dan model adaptif](IDENTIFICATION_ADAPTIVE_MODEL.md).
Seed memerlukan envelope sama; observer tidak otomatis memverifikasi PID
FCU pada run berikutnya. Tetap simpan parameter sebelum/sesudah eksperimen.
