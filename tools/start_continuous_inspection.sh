#!/usr/bin/env bash
set -eo pipefail
project_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$project_dir"
source tools/ros_environment.sh
python3 tools/generate_continuous_inspection.py
colcon build --packages-select polinasi_nav --symlink-install --build-base build_ros --install-base install_ros
export POLINASI_SIM_GUI=1 POLINASI_GAZEBO_GUI=0 POLINASI_CAMERA_GUI=1
export POLINASI_SIM_RTF=.15
unset POLINASI_SENSOR_TEST_WORLD
exec bash "$project_dir/tools/start_simulation.sh" ground_truth palm_farm mapping "$project_dir/reports/continuous_inspection/config.json"
