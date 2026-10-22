# Pengujian obstacle avoidance berbasis depth

Status: eksperimental, **belum dinyatakan layak terbang lapangan**. Lulus unit
test tidak membuktikan bahwa seluruh orbit aman. Periksa `result.json` dari
pengujian integrasi; jangan menganggap `REAL-STACK READY` sebagai tanda misi
atau avoidance telah berhasil.

## Alur kontrol

```text
FSM / dynamic_orbit_controller -> depth_avoidance_controller -> position_setpoint_controller -> MAVROS
                                      ^
                             depth + CameraInfo + pose FC
```

Saat `enable_depth_avoidance:=true`, node vortex tidak dijalankan. Hanya
supervisor depth yang menerbitkan `/control/safe_target_pose`.

- Depth diproyeksikan menggunakan intrinsics kamera dan pose dengan roll,
  pitch, yaw. Extrinsics kamera harus sesuai pemasangan sebenarnya.
- Minimum pooling mempertahankan obstacle tipis yang berada di antara kolom
  sampling. Pemeriksaan koridor memeriksa seluruh piksel dalam proyeksinya.
- Obstacle disimpan dalam voxel lokal, termasuk saat kamera menghadap ke arah
  lain. Voxel baru dihapus setelah beberapa observasi ruang kosong.
- Planner A* 2D mencari jalan pada ketinggian lokal misi. Obstacle diperbesar
  dengan radius kendaraan dan margin. Batang target **tidak dikecualikan**.
- Drone menghadap jalur sebelum bergerak. Ruang yang belum terlihat tidak
  langsung dianggap aman. Jika titik approach terhalang, dicari titik masuk
  lain pada lingkaran pohon yang sama; saat orbit dicari titik rejoin di depan.
- Fase target orbit ditahan selama hold/detour. Penyelesaian memakai sudut
  gerak aktual bertanda; osilasi maju-mundur tidak dianggap satu putaran.
- Hilangnya depth, pose, goal, atau heartbeat supervisor menahan setpoint.
  Pose/depth reset dan jalur yang terus tertutup dapat melatch HOLD sampai
  operator melakukan evaluasi dan restart. Tidak ada resume otomatis setelah
  reset koordinat.

Mode ini mengarahkan kamera ke jalur, bukan terus-menerus ke pusat pohon.
Dengan satu kamera depan, mempertahankan pandangan ke pohon sekaligus melihat
semua obstacle lateral tidak selalu mungkin. Validasi deteksi bunga/spraying
saat orbit belum tercakup dalam pengujian ini.

## Menjalankan pengujian integrasi

Hentikan sesi Gazebo, SITL, dan MAVROS lain yang memakai port 5760/9002.
Perintah berikut hanya untuk simulator di laptop, bukan untuk Jetson/drone.

```bash
cd /home/shane/ProjekAtaka/gazebo_sim
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select beehive_drone uav_plantation_sim
source install/setup.bash
python3 src/beehive_drone/test/run_depth_avoidance_sitl.py \
  --output "$PWD/log/depth_test_01" --timeout 1400 --trees 3
```

Gunakan nama folder output baru setiap percobaan. Runner menjalankan Gazebo
headless, ArduPilot SITL, MAVROS, dan misi, lalu menghentikan proses-proses yang
dibuatnya. World: `plantation_737c519.sdf`, dengan fixture obstacle tambahan.
Posisi awal pengujian `(2.2, -4.1)` dipilih setelah pemeriksaan clearance mesh;
spawn lama `(0, -3)` berada sangat dekat dengan pelepah pada ketinggian misi.
Jangan menggunakan lokasi takeoff hanya berdasarkan jarak ke pusat batang.

File keluaran: `gazebo.log`, `sitl.log`, `mavros.log`, `mission.log`,
`trajectory.csv`, `result.json`, dan `mission_report/`. Ground truth/SDF dipakai
untuk penilaian pengujian dan adapter deteksi pohon sintetis, **bukan** sebagai
sumber obstacle untuk controller depth.

Kriteria runner saat ini: misi DONE, jumlah pohon inspected sesuai permintaan,
serta jarak badan terhadap fixture dan batang mencukupi. Penilaian ini belum
membuktikan clearance terhadap setiap segitiga mesh pelepah: model pohon lama
hanya mempunyai collision batang. Fixture tambahan mempunyai visual **dan**
collision. Jadi hasil lulus sekalipun tetap merupakan pengujian terbatas.

Untuk tampilan GUI, jalankan world secara manual (jangan bersamaan runner):

```bash
ros2 launch uav_plantation_sim plantation_sim.launch.py \
  world:=plantation_737c519.sdf avoidance_fixture:=true \
  spawn_x:=2.2 spawn_y:=-4.1
```

Setelah SITL dan MAVROS siap sesuai `REAL_STACK_SIM.md`, jalankan:

```bash
ros2 launch beehive_drone gazebo_stack.launch.py \
  tree_source:=sdf tree_world:=plantation_737c519.sdf \
  source_tree_limit:=3 mission_type:=basic_orbit \
  mission_mode:=multi_tree max_trees:=3 expected_tree_count:=1 \
  enable_depth_avoidance:=true auto_start:=false
```

Mulai misi hanya setelah memeriksa `/avoidance/ready`, pose, peta pohon, dan
ruang takeoff. Gunakan prosedur `/mission/start` pada panduan simulasi utama.

## Parameter dan diagnosis

Konfigurasi geometri/sensor: `config/depth_avoidance.yaml`.
Geometri kendaraan awal: radius 0.40 m, margin 0.35 m, half-height 0.30 m.
Ukuran ini bukan pengganti pengukuran drone sebenarnya. Radius approach/orbit
harus cocok dengan konfigurasi misi di `config/real.yaml`.

Dengan avoidance aktif, launch membatasi slew horizontal menjadi 0.015 m per
tick pada 20 Hz (target bergerak maksimum 0.30 m/s). Ini bukan jaminan batas
kecepatan aktual; tuning FC dan jarak pengereman tetap perlu diuji.

```bash
ros2 topic echo /avoidance/status --field data
ros2 topic echo /avoidance/ready
ros2 topic echo /mission/fsm_state
```

`LOOK_BEFORE_MOVE`: menghadap jalur; `DETOUR`: jalur menyimpang dari target
nominal; `UNOBSERVED_HOLD`: koridor belum terbukti terlihat bebas;
`CLEARANCE_HOLD`: posisi sekarang masuk batas clearance obstacle;
`NO_PATH_HOLD`: belum ada jalur lokal yang ditemukan.
`DEPTH_STALE`/`POSE_STALE`/`GOAL_STALE`: input tidak segar.
`BLOCKED_RESTART_REQUIRED` atau reset pose/clock: intervensi operator diperlukan.

## Batas sebelum penggunaan nyata

- Tidak memindahkan kode otomatis ke `polinasi` dan tidak mengubah parameter FC
  atau kalibrasi compass/EKF.
- Pemetaan pohon dari SDF di simulator bukan bukti keberhasilan deteksi AI ZED.
- `use_receipt_time` dan `inf_is_clear` diaktifkan khusus simulasi. Kamera nyata
  harus memakai timestamp yang sinkron; depth invalid/inf bukan ruang bebas.
- Ukur extrinsics, cek arah frame dan satuan depth, validasi loss-of-depth dan
  penghentian proses supervisor, ukur stop distance, serta sediakan pilot dan
  jalur takeover sebelum pengujian terbatas di lapangan.
- Takeoff/landing masih prosedur FC, bukan perencanaan volume vertikal 3D.
- Perencana pada altitude tetap tidak dapat melewati koridor yang benar-benar
  tertutup. Menahan posisi adalah hasil yang benar dalam kondisi tersebut;
  jangan mengurangi margin hanya untuk membuat misi selesai.
- Multi-tree hanya mengunjungi pohon yang terdeteksi/terpetakan dan terjangkau.
  Klaim mengitari seluruh perkebunan memerlukan validasi coverage dan jalur
  untuk setiap pohon, bukan sekadar `max_trees:=0`.
