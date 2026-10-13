#!/usr/bin/env bash
# Simulation only; takeoff requires the separate acknowledged mission-start.
set -eo pipefail
project_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$project_dir"
source tools/ros_environment.sh
python3 tools/generate_rolling_hover_test.py --rate "${1:-0.4}"
colcon build --packages-select polinasi_nav --symlink-install --build-base build_ros --install-base install_ros
export POLINASI_SIM_GUI=1
export POLINASI_GAZEBO_GUI=1
export POLINASI_CAMERA_GUI=0
export POLINASI_SIM_RTF="${POLINASI_SIM_RTF:-0.15}"
unset POLINASI_SENSOR_TEST_WORLD
exec bash "$project_dir/tools/start_simulation.sh" ground_truth palm_farm mapping \
  "$project_dir/reports/rolling_hover/config.json"
