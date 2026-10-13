#!/usr/bin/env bash
set -eo pipefail
project_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$project_dir"
source tools/ros_environment.sh
stage="${1:-hover}"
case "$stage" in hover|inspection) ;; *) exit 2 ;; esac
python3 tools/generate_sensor_localisation.py
colcon build --packages-select polinasi_nav --symlink-install --build-base build_ros --install-base install_ros
export POLINASI_SIM_GUI=1 POLINASI_GAZEBO_GUI=0 POLINASI_CAMERA_GUI=1
export POLINASI_SIM_RTF=.15
unset POLINASI_SENSOR_TEST_WORLD
exec bash "$project_dir/tools/start_simulation.sh" sensor palm_farm mapping "$project_dir/reports/sensor_localisation/$stage.json"
