#!/usr/bin/env bash
set -eo pipefail
project_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
POLINASI_SKIP_PROJECT_OVERLAY=1
source "$project_dir/tools/ros_environment.sh"
cd "$project_dir"
# New prefixes preserve the earlier standalone packaging artifacts.
colcon --log-base log_ros build --symlink-install --build-base build_ros \
  --install-base install_ros --base-paths polinasi_nav beehive_drone uav_interfaces \
  --packages-select uav_interfaces beehive_drone polinasi_nav \
  --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3 \
  -DCMAKE_C_COMPILER=/usr/bin/gcc -DCMAKE_CXX_COMPILER=/usr/bin/g++
source "$project_dir/install_ros/setup.bash"
# Test nodes must not publish into a simultaneously running simulation.
ROS_DOMAIN_ID="${POLINASI_TEST_ROS_DOMAIN_ID:-87}" \
colcon --log-base log_ros test --build-base build_ros --install-base install_ros \
  --packages-select polinasi_nav --event-handlers console_direct+
colcon test-result --test-result-base build_ros --all --verbose
