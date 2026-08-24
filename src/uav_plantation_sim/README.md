# gazebo_sim / uav_plantation_sim

Repository ini berisi simulasi UAV untuk pengujian algoritma persepsi dan navigasi pada perkebunan kelapa sawit sebelum diuji di lapangan. Repository GitHub dapat bernama `gazebo_sim`, sedangkan nama paket ROS 2 di dalamnya adalah `uav_plantation_sim`.

Simulasi menggunakan ROS 2 Humble, Gazebo Sim 8.14.0 / Gazebo Harmonic, model quadcopter X3-style, kamera ZED 2i-style, dan world perkebunan sawit.

## Fitur

- World Gazebo default memiliki tanjakan 10%, puncak, dan turunan untuk uji
  terrain following. World datar tetap tersedia sebagai baseline.
- Model drone quadcopter X3-style dengan mesh body dan propeller lokal.
- Sensor IMU.
- Kamera stereo ZED 2i-style:
  - kamera kiri RGB,
  - kamera kanan RGB,
  - kamera depth RGB-D.
- Downward range sensor berbasis GPU lidar 1D.
- Bridge ROS 2 ke Gazebo melalui `ros_gz_bridge`.
- Kontrol awal menggunakan topic ROS 2 `/cmd_vel`.

## Struktur Paket

```text
gazebo_sim/
├── CMakeLists.txt
├── package.xml
├── README.md
├── config/
│   └── bridge_config.yaml
├── launch/
│   └── plantation_sim.launch.py
├── models/
│   ├── plantation_quadrotor/
│   ├── oil_palm/
│   ├── oil_palm_small/
│   ├── oil_palm_large/
│   └── obstacle models lainnya
└── worlds/
    ├── plantation.sdf
    └── plantation_hilly.sdf
```

## Prasyarat

Direkomendasikan menggunakan:

- Ubuntu 22.04
- ROS 2 Humble
- Gazebo Sim 8.14.0 / Gazebo Harmonic
- `ros_gz_sim`
- `ros_gz_bridge`
- `colcon`
- `rosdep`
- `rqt_image_view` untuk melihat image kamera

## Instalasi Dependensi

Install tools dasar ROS 2:

```bash
sudo apt update
sudo apt install -y \
  python3-colcon-common-extensions \
  python3-rosdep \
  python3-vcstool \
  ros-humble-rqt-image-view
```

Install paket bridge Gazebo untuk ROS 2 Humble:

```bash
sudo apt install -y \
  ros-humble-ros-gz \
  ros-humble-ros-gz-sim \
  ros-humble-ros-gz-bridge
```

Inisialisasi `rosdep` jika belum pernah dilakukan:

```bash
sudo rosdep init
rosdep update
```

Pastikan environment ROS 2 sudah aktif:

```bash
source /opt/ros/humble/setup.bash
```

## Clone Repository

Buat workspace ROS 2:

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```

Clone repository:

```bash
git clone https://github.com/NicholasShanePangihutanSiahaan/Depth_drone_gz.git gazebo_sim
```

Masuk ke workspace:

```bash
cd ~/ros2_ws
```

## Build Paket

Install dependensi dari `package.xml`:

```bash
rosdep install --from-paths src --ignore-src -r -y
```

Build paket:

```bash
colcon build --packages-select uav_plantation_sim
```

Source hasil build:

```bash
source install/setup.bash
```

Agar otomatis tersource setiap terminal baru, tambahkan ke `~/.bashrc`:

```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
```

## Menjalankan Simulasi

Jalankan launch file utama:

```bash
cd ~/ros2_ws
source install/setup.bash
ros2 launch uav_plantation_sim plantation_sim.launch.py
```

Launch file ini akan:

- membuka Gazebo dengan `worlds/plantation_hilly.sdf`,
- mengatur `GZ_SIM_RESOURCE_PATH` agar model lokal dapat ditemukan,
- menjalankan `ros_gz_bridge`,
- spawn drone di landasan awal `(0, 0, 0.1)`.

Untuk baseline datar atau mode headless:

```bash
ros2 launch uav_plantation_sim plantation_sim.launch.py world:=plantation.sdf
ros2 launch uav_plantation_sim plantation_sim.launch.py gz_args:='-s -r -v 3'
```

## Uji misi SITL di terrain berbukit

Build seluruh paket yang terlibat:

```bash
cd ~/ProjekAtaka/gazebo_sim
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select \
  uav_interfaces pcl_cstm_msg point-cloud-test beehive_drone uav_plantation_sim
source install/setup.bash
```

Jalankan empat terminal berikut secara berurutan.

Terminal 1 — Gazebo berbukit:

```bash
ros2 launch uav_plantation_sim plantation_sim.launch.py
```

Terminal 2 — ArduPilot SITL:

```bash
sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON -w \
  --add-param-file=$PWD/src/beehive_drone/config/sitl_sim.parm \
  --console --out=udp:127.0.0.1:14551
```

Terminal 3 — MAVROS:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run mavros mavros_node --ros-args \
  -p fcu_url:=udp://127.0.0.1:14551@
```

Terminal 4 — persepsi dan misi:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch beehive_drone simulation_mission.launch.py
```

`sim_rangefinder_bridge` mengubah `/range` dari Gazebo menjadi
`/simulation/rangefinder`. Controller memakai topic tersebut untuk menjaga AGL
1.5 m, sementara local pose tetap menjadi koordinat navigasi. Laporan uji ada
di `~/beehive_mission_reports/sim/`.

## Menjalankan World Tanpa ROS Launch

Jika hanya ingin membuka world Gazebo tanpa bridge ROS 2:

```bash
cd ~/ros2_ws/src/gazebo_sim
GZ_SIM_RESOURCE_PATH=$PWD/models gz sim worlds/plantation.sdf
```

Mode ini berguna untuk mengecek tampilan world, tetapi topic ROS 2 seperti `/cmd_vel` dan `/zed2i/...` tidak otomatis tersedia.

## Menggerakkan Drone

Drone saat ini dikontrol dengan topic ROS 2 `/cmd_vel` bertipe `geometry_msgs/msg/Twist`.

Pastikan simulasi sudah berjalan, lalu buka terminal baru:

```bash
cd ~/ros2_ws
source install/setup.bash
```

Naik:

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {z: 0.5}}" -r 10
```

Turun:

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {z: -0.3}}" -r 10
```

Maju:

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5}}" -r 10
```

Mundur:

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: -0.5}}" -r 10
```

Geser kiri:

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {y: 0.5}}" -r 10
```

Geser kanan:

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {y: -0.5}}" -r 10
```

Putar yaw:

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{angular: {z: 0.5}}" -r 10
```

Berhenti / hover:

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {z: 0.0}}" -r 10
```

## Menampilkan Kamera ZED 2i

Cek topic kamera:

```bash
ros2 topic list | grep zed2i
```

Tampilkan kamera kiri:

```bash
ros2 run rqt_image_view rqt_image_view /zed2i/left/image_rect_color
```

Tampilkan kamera kanan:

```bash
ros2 run rqt_image_view rqt_image_view /zed2i/right/image_rect_color
```

Tampilkan depth image:

```bash
ros2 run rqt_image_view rqt_image_view /zed2i/depth/depth_registered
```

Jika `rqt_image_view` belum tersedia:

```bash
sudo apt install -y ros-humble-rqt-image-view
```

## Topik ROS 2 Penting

Topic kontrol:

```text
/cmd_vel
```

Topic sensor:

```text
/imu
/range
/zed2i/left/image_rect_color
/zed2i/left/camera_info
/zed2i/right/image_rect_color
/zed2i/right/camera_info
/zed2i/depth/depth_registered
/zed2i/depth/camera_info
```

Cek semua topic:

```bash
ros2 topic list
```

Cek frekuensi kamera:

```bash
ros2 topic hz /zed2i/left/image_rect_color
ros2 topic hz /zed2i/depth/depth_registered
```

Cek data IMU:

```bash
ros2 topic echo /imu
```

Cek data range sensor:

```bash
ros2 topic echo /range
```

## Rincian Sensor

Kamera ZED 2i-style pada simulasi:

- stereo baseline sekitar 12 cm,
- resolusi image 640 x 480,
- update rate 30 Hz,
- horizontal field of view sekitar 80 derajat,
- depth clipping range 0.12 m sampai 15 m,
- RGB clipping range 0.05 m sampai 80 m.

Topic Gazebo dijembatani ke topic ROS 2 melalui file:

```text
config/bridge_config.yaml
```

## Pengujian Cepat

Setelah simulasi berjalan, lakukan urutan berikut:

1. Cek topic:

   ```bash
   ros2 topic list
   ```

2. Cek kamera kiri:

   ```bash
   ros2 topic hz /zed2i/left/image_rect_color
   ```

3. Cek depth:

   ```bash
   ros2 topic hz /zed2i/depth/depth_registered
   ```

4. Tampilkan image:

   ```bash
   ros2 run rqt_image_view rqt_image_view /zed2i/left/image_rect_color
   ```

5. Gerakkan drone naik:

   ```bash
   ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {z: 0.5}}" -r 10
   ```

6. Hentikan drone:

   ```bash
   ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {z: 0.0}}" -r 10
   ```

## Troubleshooting

Jika `ros2 launch` tidak menemukan paket:

```bash
cd ~/ros2_ws
colcon build --packages-select uav_plantation_sim
source install/setup.bash
ros2 pkg list | grep uav_plantation_sim
```

Jika model tidak muncul di Gazebo, pastikan `GZ_SIM_RESOURCE_PATH` mengarah ke folder `models`. Launch file utama sudah mengatur ini otomatis.

Jika topic kamera tidak muncul, pastikan simulasi dijalankan lewat:

```bash
ros2 launch uav_plantation_sim plantation_sim.launch.py
```

Jika `rqt_image_view` kosong, cek apakah topic image memiliki data:

```bash
ros2 topic hz /zed2i/left/image_rect_color
```

Jika command `/cmd_vel` tidak menggerakkan drone, cek bridge:

```bash
ros2 node list
ros2 topic info /cmd_vel
```

## Status ArduPilot dan Gazebo Harmonic

Drone sudah menggunakan ArduPilot SITL (Software In The Loop) yang terhubung dengan Flight Controller pada simulasi Gazebo Harmonic. 
Rincian instalasi dapat dilihat pada https://github.com/ArduPilot/ardupilot_gazebo/blob/main/README.md

## Harmonic (apt)
Untuk instalasi Gazebo Harmonic (apt) dapat menggunakan command line berikut:
```bash
sudo apt update
sudo apt install libgz-sim8-dev rapidjson-dev
sudo apt install libopencv-dev libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev gstreamer1.0-plugins-bad gstreamer1.0-libav gstreamer1.0-gl
```

### Rosdep:
Untuk depedensi ROS, dapat menjalankan perintah berikut:
```bash
export GZ_VERSION=harmonic
sudo bash -c 'wget https://raw.githubusercontent.com/osrf/osrf-rosdep/master/gz/00-gazebo.list -O /etc/ros/rosdep/sources.list.d/00-gazebo.list'
rosdep update
rosdep resolve gz-harmonic
rosdep install --from-paths src --ignore-src -y
```

### Clone repo:
Clone repository berikut dan build repository berikut:
```bash
git clone https://github.com/ArduPilot/ardupilot_gazebo
cd ardupilot_gazebo
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=RelWithDebInfo
make -j4
```

### Configure

#### Terminal
Jika ingin langsung menjalankan Gazebo Environment di terminal, maka jalankan command berikut sebelum menjalankan Gazebo:
```bash
cd ~/ardupilot_gazebo
export GZ_SIM_SYSTEM_PLUGIN_PATH=$HOME/ardupilot_gazebo/build:$GZ_SIM_SYSTEM_PLUGIN_PATH
export GZ_SIM_RESOURCE_PATH=$HOME/ardupilot_gazebo/models:$HOME/ardupilot_gazebo/worlds:$GZ_SIM_RESOURCE_PATH
```

#### Bashrc
Jika ingin langsung menjalankan Gazebo Environment setiap membuka terminal/menyalakan laptop, perintah hanya perlu dilakukan satu kali:
```bash
echo 'export GZ_SIM_SYSTEM_PLUGIN_PATH=$HOME/ardupilot_gazebo/build:${GZ_SIM_SYSTEM_PLUGIN_PATH}' >> ~/.bashrc
echo 'export GZ_SIM_RESOURCE_PATH=$HOME/ardupilot_gazebo/models:$HOME/ardupilot_gazebo/worlds:${GZ_SIM_RESOURCE_PATH}' >> ~/.bashrc
```

```bash
source ~/.bashrc
```

Untuk menjalankan SITL yang terhubung dengan model quadcopter X3-style:
```bash
sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON --map --console
```

### ROS dan SITL
Agar Gazebo, SITL dan program-program ROS dapat terhubung, protokol komunikasi perlu disesuaikan dengan perintah
#### SITL 
```bash
sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON --console --out=udp:127.0.0.1:14551
```

#### ROS
```bash
ros2 run mavros mavros_node --ros-args -p fcu_url:=udp://127.0.0.1:14551@
```
Masing-masing dijalankan di terminal yang berbeda. Skrip python dapat dijalankan setelah keduanya berjalan
