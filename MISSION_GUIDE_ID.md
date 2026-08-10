# Panduan misi drone kebun sawit

## Terminal dan urutan menjalankan simulasi

Gunakan terminal baru untuk setiap kelompok proses. Semua terminal ROS harus memakai
environment yang sama.

```bash
cd /home/shane/ProjekAtaka/gazebo_sim
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select uav_interfaces point-cloud-test beehive_drone uav_plantation_sim
source install/setup.bash
```

Urutan proses yang disarankan:

1. Jalankan `plantation_sim.launch.py` dan tunggu Gazebo benar-benar berjalan.
2. Jalankan ArduPilot SITL (`sim_vehicle.py`) dan tunggu EKF/home siap.
3. Jalankan MAVROS dan pastikan FCU terhubung.
4. Jalankan satu dan hanya satu launch misi.

Misi satu pohon:

```bash
ros2 launch beehive_drone mission_gazebo_revised.launch.py
```

Misi semua pohon:

```bash
ros2 launch beehive_drone multi_tree_mission.launch.py
```

Jangan menjalankan `mission.launch.py` dan `mission_gazebo_revised.launch.py`
bersamaan. Pada simulator ini `/zed2i/depth/points` sudah diterbitkan bridge
Gazebo. Karena itu `use_depth_pointcloud` pada `mission.launch.py` sekarang default
`false`; menyalakan konverter depth kedua dapat membuat dua publisher point cloud,
track PCL ganda, dan pusat pohon meloncat.

## Pemeriksaan sebelum arm

```bash
ros2 topic echo --once /mavros/state
ros2 topic hz /mavros/local_position/pose
ros2 topic hz /zed2i/depth/points
ros2 topic echo --once /map/trees
ros2 topic echo --once /map/trees_ready
ros2 topic echo /mission/fsm_state
```

Syarat aman: MAVROS `connected: true`, local pose terus masuk, point cloud hanya
memiliki publisher yang memang diinginkan, dan `trees_ready: true`. Lihat jumlah
publisher dengan:

```bash
ros2 topic info -v /zed2i/depth/points
```

## Mengapa drone sebelumnya bergerak ke kanan/menabrak

Versi lama memilih pohon berdasarkan jarak terdekat saja. Pohon di samping dapat
mengalahkan pohon di depan. ID dari tracker PCL juga langsung dijadikan ID peta;
satu batang yang terdeteksi ulang bisa mendapat ID lain. Lebih berbahaya lagi,
pusat target terus diperbarui selama approach sehingga lingkaran orbit ikut
bergeser.

Revisi ini memakai ID peta hasil asosiasi spasial, memerlukan beberapa observasi,
memilih target misi tunggal dalam kerucut depan, melakukan hover verifikasi, dan
membekukan pusat target sampai orbit selesai. Orbit ditolak atau dibatalkan jika
koridor lingkarannya memotong pohon lain di peta.

## State yang normal

Misi tunggal:

`SET_MODE -> ARM -> TAKEOFF -> SEARCH_TREE -> VERIFY_TARGET -> APPROACH_TREE ->
HOVER_BEFORE_ORBIT -> PREPARE_ORBIT -> WAIT_ORBIT -> POST_ORBIT_HOVER ->
RETURN_PRE_ORBIT -> HOVER_AT_PRE_ORBIT -> RETRACE_HOME -> HOME_HOVER -> LAND`

Misi semua pohon kembali ke `SEARCH_TREE` setelah `HOVER_AT_PRE_ORBIT`. Jika tidak
ada pohon baru, state menjadi `SCAN_FOR_TREE`; scan gagal dilanjutkan dengan
`RETRACE_HOME`.

## Troubleshooting takeoff

- Jika berhenti di `SET_MODE`, periksa `/mavros/state`, nama mode `GUIDED`, dan
  service MAVROS set-mode.
- Jika berhenti di `ARM`, baca `STATUSTEXT` ArduPilot; penyebab umum adalah EKF,
  home, GPS, atau pre-arm check belum siap.
- Jika masuk `TAKEOFF` tetapi tidak naik, pastikan hanya satu instance
  `flight_manager`, `mission_state_machine`, dan `velocity_controller` aktif.
- Jika `mission.launch.py` lama tidak takeoff, penyebab konfigurasi terdahulu
  biasanya `hold_after_takeoff`, PCL node tidak menerima YAML, atau point cloud
  diterbitkan ganda. Ketiganya sudah diperbaiki di source; build dan source ulang.
- Setelah mengubah Python/YAML/launch, jalankan `colcon build --symlink-install`
  lalu `source install/setup.bash` pada setiap terminal lama. Terminal yang belum
  di-source masih memakai file install versi sebelumnya.

## Batas simulasi dan hardware nyata

Algoritma ini tidak menghafal koordinat pohon Gazebo, sehingga bukan overfitting
koordinat world. Akan tetapi parameter radius batang, jarak asosiasi, kecepatan,
clearance, noise, dan transform frame tetap perlu dikalibrasi di lokasi nyata.

Sebelum uji ZED2i + Jetson + Pixhawk, wajib mengganti
`allow_identity_frame_relabel` menjadi `false`, menerbitkan TF terukur dari frame
optical kamera ke body/local odometry, dan memastikan timestamp ZED, MAVROS, serta
Jetson sinkron. Uji bertahap: props-off, takeoff-hold, approach tanpa orbit, orbit
dengan radius besar, baru misi penuh. Peta batang dan vortex menghindari pohon yang
terpetakan; benda non-batang belum mempunyai jaminan deteksi omnidirectional.
Tambahkan sensor/FOV samping-belakang atau occupancy map 3D sebelum menganggap
return-home aman terhadap semua jenis rintangan di dunia nyata.
