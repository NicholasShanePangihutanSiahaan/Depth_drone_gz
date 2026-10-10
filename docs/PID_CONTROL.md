# Branch lidar360-PID

Branch lidar360-mpc menyimpan snapshot MPC sebelumnya. Pada branch ini
`bash tools/start_mapping.sh tour` dan `bash tools/start_pid.sh tour` memakai PID
aktif, bukan MPC. `start_predictive.sh` tetap opsi eksplisit untuk perbandingan,
bukan default. PID/MPC tidak boleh aktif bersamaan.

PID posisi per sumbu menghitung koreksi percepatan:
`a = a_referensi + Kp*(p_ref-p) + Ki*integral(p_ref-p) + Kd*(v_ref-v)`.
Turunan memakai error kecepatan terukur, bukan diferensiasi posisi yang berisik.
Integral dibatasi dan tidak bertambah saat keluaran dibatasi; reset saat state
berubah. Koreksi maksimum 0,15 m/s², acceleration dan jerk tetap mengikuti
batas keselamatan konfigurasi. Tidak membutuhkan parameter model/optimizer.

ArduPilot tetap memiliki kontrol posisi/kecepatan/sikap internal. PID baru ini
adalah feedback tambahan pada acceleration feedforward PVA, **bukan** pengganti
PID sikap Pixhawk atau jaminan kestabilan. Gain awal konservatif perlu diuji.
Satu publisher final MAVROS tetap dipertahankan. Pemetaan, planner, referensi
halus, batas tracking, stale sensor dan braking/collision gate tidak dihapus.

Log `active_controller`, `pid`, `pid_gains`, `pid_wall_max_ms` memperlihatkan
error, integral, saturasi dan keluaran. `processing_pipeline` tetap mencatat
waktu nyata pemetaan sampai publikasi setpoint. `model_controls_flight=false`
dan `predictive_active=false` pada mode PID.

```bash
cd /home/abin/polinasi_lidar
source tools/ros_environment.sh
/usr/bin/python3 tools/generate_pid_configs.py
bash tools/build_ros.sh
bash tools/start_pid.sh tour_check
```

Terminal lain, setelah startup EKF stabil:

```bash
cd /home/abin/polinasi_lidar
source tools/ros_environment.sh
.venv/bin/python tools/set_origin.py --timeout 120
/usr/bin/python3 tools/check_ros_simulation.py --output reports/pid_setup.json
```

Jika `passed=true` (ulang pemeriksaan bila masih menunggu alignment):

```bash
/usr/bin/python3 tools/watch_mapping.py --seconds 1800 --output reports/pid_flight.json
```

Terminal lain dengan environment yang sama:

```bash
/usr/bin/python3 tools/start_mission.py
```

Gunakan `tour` untuk 326 target, `tour_check` untuk empat target dan pulang/land.
Gazebo/RViz dibuka otomatis. Untuk stop semua proses/jendela milik run ini:
`python3 tools/stop_simulation.py`.

Posisi ground truth bukan lokalisasi LiDAR. Jangan mengoperasikan hardware
dengan preset simulasi ini. Hasil run PID dilaporkan terpisah dari hasil MPC.
