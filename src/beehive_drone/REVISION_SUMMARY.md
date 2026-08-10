# Ringkasan Penggantian File

| File lama | File revisi | Perubahan inti |
|---|---|---|
| `mission_params.py` | `beehive_drone/mission_params.py` | Parameter lengkap, timeout, PCL, orbit, safety, controller |
| `flight_manager.py` | `beehive_drone/flight_manager.py` | Service guard, status connected, hover berbasis altitude + velocity |
| `tree_mapper.py` | `beehive_drone/tree_mapper.py` | Menjadi mapper utama berbasis `TrackedCylinderArray` |
| `tree_localizer.py` | `beehive_drone/tree_localizer.py` | Transformasi TF2 dan `PointStamped` |
| `yolo_gazebo_detector.py` | `beehive_drone/yolo_gazebo_detector.py` | Parameter model/device, CameraInfo, standard point output |
| `mission_state_machine.py` | `beehive_drone/mission_state_machine.py` | FSM baru, target verification, timeout, home aktual, landing confirmation |
| `dynamic_orbit_controller.py` | `beehive_drone/dynamic_orbit_controller.py` | ALIGNING/ORBITING, signed progress, real velocity use, timeout |
| `vortex_avoidance_controller.py` | `beehive_drone/vortex_avoidance_controller.py` | Influence field lebih awal, target tree exclusion, stale goal guard |
| `velocity_controller.py` | `beehive_drone/velocity_controller.py` | Stale failsafe, acceleration limiting, kontrol XY/Z terpisah |
| `mission_analyzer.py` | `beehive_drone/mission_analyzer.py` | Sampling terkendali, CSV otomatis, optional plot |
| — | `launch/complete_mission.launch.py` | Menjalankan seluruh pipeline dan remapping PCL |
| — | `config/mission.yaml` | Tuning terpusat |
| — | `math_utils.py` | Utilitas quaternion, yaw, clamp, angle wrap |
