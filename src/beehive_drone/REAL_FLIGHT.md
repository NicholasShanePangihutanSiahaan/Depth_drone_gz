# Real-flight checklist

`real_mission.launch.py` tidak menjalankan MAVROS, ZED wrapper, atau proses PCL.
Jalankan ketiganya lebih dahulu. Default launch saat ini adalah
`auto_start:=true`; untuk uji lapangan bertahap selalu override menjadi
`auto_start:=false` sampai seluruh pemeriksaan pra-terbang lulus.

## Arsitektur altitude dan terrain following

- Pixhawk/EKF menyediakan posisi lokal yang stabil untuk navigasi XY dan Z.
- Rangefinder bawah menyediakan tinggi AGL (jarak drone terhadap permukaan).
- `position_setpoint_controller` mengubah target Z lokal secara perlahan agar
  `desired_agl` tetap tercapai ketika tanah naik atau turun.
- Rangefinder tidak menggantikan seluruh local pose. Target yang dikirim ke
  MAVROS tetap `PoseStamped` dalam frame `map`; koreksinya adalah
  `Z_target = Z_local + (desired_agl - range_terfilter)`.
- Jika rangefinder stale/tidak valid, controller menahan setpoint terakhir dan
  tidak melanjutkan XY selama `hold_position_on_range_loss: true`. Ia tidak
  menebak ketinggian tanah.

Konfigurasi utama ada di blok `position_setpoint_controller` pada
`config/real.yaml`. `flight_manager.altitude_source: rangefinder` hanya
menentukan gate selesai takeoff/hover, sedangkan terrain following berlangsung
di controller setpoint selama approach, orbit, dan kembali ke home.

## Kalibrasi yang wajib

1. Isi `camera_offset_*` dan `camera_mount_*` pada `config/real.yaml`
   berdasarkan pemasangan ZED2i terhadap `base_link` (meter dan radian).
2. Pastikan `header.frame_id` point cloud benar dan pohon global tidak berpindah
   ketika drone digeser atau diputar secara manual.
3. Pastikan `/mavros/rangefinder/rangefinder` bertipe `sensor_msgs/msg/Range`,
   finite, lebih besar dari `rangefinder_min_valid`, dan frekuensinya stabil.
   Nilai maksimum sensor/parameter FC harus lebih tinggi dari 1.5 m dengan
   margin memadai; jangan terbang bila sensor berhenti pada nilai maksimum.
4. Tuning batas radius/tinggi batang, orbit, dan slew limit pada `real.yaml`
   dengan propeller dilepas terlebih dahulu.
5. Pastikan hanya satu node yang menerbitkan setpoint MAVROS. Launch nyata
   menjalankan `position_setpoint_controller`; jangan jalankan
   `velocity_controller` pada saat yang sama.
6. Pastikan estimator Pixhawk sudah menerima VisualOdom dan topic
   `/mavros/local_position/pose` stabil sebelum mission launch dijalankan.
7. Untuk konfigurasi EKF, gunakan Barometer sebagai sumber POSZ utama dan
   ExternalNav/Vision untuk posisi horizontal sesuai konfigurasi kendaraan.
   Rangefinder digunakan sebagai pengukuran AGL oleh program; jangan memilih
   RangeFinder sebagai sumber EKF POSZ hanya karena terrain following aktif.

## Pemeriksaan tanpa propeller

```bash
ros2 topic hz /mavros/local_position/pose
ros2 topic hz /mavros/rangefinder/rangefinder
ros2 topic echo /mavros/rangefinder/rangefinder
ros2 topic echo /mavros/statustext/recv
```

Angkat dan turunkan drone secara manual. Rangefinder harus berubah sesuai jarak
ke tanah, local pose tidak boleh melompat, dan setelah mission launch dengan
`auto_start:=false` status terrain harus `TRACKING`:

```bash
ros2 topic echo /control/terrain/status
ros2 topic echo /control/terrain/measured_agl
ros2 topic echo /control/terrain/target_z
```

## Menjalankan

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch beehive_drone real_mission.launch.py
```

Karena default di atas langsung memulai misi, urutan validasi lapangan yang
disarankan adalah:

```bash
ros2 launch beehive_drone real_mission.launch.py auto_start:=false
```

Periksa watchdog sebelum start:

```bash
ros2 topic echo --once /mission/safety_reason
ros2 topic echo --once /mission/safety_ok
```

Setelah semua data sehat dan area aman, mulai misi manual:

```bash
ros2 topic pub --once /mission/start std_msgs/msg/Bool "{data: true}"
```

Untuk pengujian yang memang menghendaki start otomatis:

```bash
ros2 launch beehive_drone real_mission.launch.py auto_start:=true
```

Perpindahan mode melalui RC menghentikan publikasi setpoint otomatis. Pilot
tetap harus siap mengambil alih melalui RC. Uji takeover pada ketinggian rendah
dan area terbuka sebelum menjalankan misi dekat pohon.

## Tahapan validasi lapangan

Keberhasilan Gazebo bukan jaminan drone nyata aman. Lakukan bertahap:

1. Propeller dilepas: validasi topic, frame, range, dan setpoint.
2. Tether/area terbuka: takeoff 1.5 m, hover, takeover, lalu land; tanpa PCL.
3. Gerak maju-mundur di permukaan datar sambil mengecek AGL dan local Z.
4. Lintasan lereng ringan tanpa pohon; target awal kecepatan rendah.
5. Approach dan orbit objek lunak/aman, baru kemudian pohon sawit.

Setelah setiap uji, periksa `mission_summary.json`,
`altitude_diagnostics.csv`, `diagnostic_events.csv`, dan rosbag. Tolak uji
berikutnya bila `tracking_availability_percent` tidak mendekati 100%, terdapat
range dropout, EKF/status text bermasalah, atau AGL error melampaui batas uji.
