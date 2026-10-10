#!/usr/bin/env bash
# Source this file in a dedicated simulation terminal. Does not edit shell rc files.
polinasi_project_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
if [[ ! -f /opt/ros/humble/setup.bash ]]; then
  printf 'ROS 2 Humble is not installed yet.\n' >&2
  return 1 2>/dev/null || exit 1
fi
# Existing ROS 1/pyenv sessions must not contaminate this ROS 2 process.
unset ROS_PACKAGE_PATH ROS_MASTER_URI ROS_IP ROS_HOSTNAME
unset ROS_DISTRO ROS_VERSION ROS_PYTHON_VERSION
unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH PYTHONPATH LD_LIBRARY_PATH
export PATH=/usr/bin:/bin:/usr/sbin:/sbin
source /opt/ros/humble/setup.bash
if [[ "${POLINASI_ROS_SYSTEM_ONLY:-0}" != 1 && -f "$polinasi_project_dir/.dependencies/mavros_ws/install/setup.bash" ]]; then
  source "$polinasi_project_dir/.dependencies/mavros_ws/install/setup.bash"
fi
if [[ "${POLINASI_ROS_SYSTEM_ONLY:-0}" != 1 && "${POLINASI_SKIP_PROJECT_OVERLAY:-0}" != 1 && -f "$polinasi_project_dir/install_ros/setup.bash" ]]; then
  source "$polinasi_project_dir/install_ros/setup.bash"
fi
export ROS_LOCALHOST_ONLY=1
# Keep mission/control topics apart from unrelated ROS sessions on this host.
export ROS_DOMAIN_ID="${POLINASI_ROS_DOMAIN_ID:-86}"
export GZ_PARTITION=polinasi_lidar
export OPENBLAS_NUM_THREADS=1
export OMP_NUM_THREADS=1
