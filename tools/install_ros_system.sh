#!/usr/bin/env bash
# Administrator-only dependencies for the loopback simulation. No hardware I/O.
set -euo pipefail
export PATH=/usr/sbin:/usr/bin:/sbin:/bin
if [[ "$EUID" -ne 0 ]]; then
  printf 'Run through Ubuntu authentication: pkexec /usr/bin/bash %s\n' "$0"
  exit 1
fi
source /etc/os-release
if [[ "$ID" != ubuntu || "$VERSION_ID" != 22.04 ]]; then
  printf 'This installer is scoped to Ubuntu 22.04.\n' >&2
  exit 2
fi
project_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
bootstrap="$project_dir/.dependencies/ros2-apt-source.deb"
if [[ ! -f "$bootstrap" || "$(dpkg-deb --field "$bootstrap" Package)" != ros2-apt-source ]]; then
  printf 'Missing official ros2-apt-source bootstrap package.\n' >&2
  exit 2
fi
dpkg -i "$bootstrap"
# Refresh just ROS and the already-configured OSRF repository. Existing Ubuntu
# package indices supply platform dependencies; unrelated app repos are untouched.
apt-get update -o Dir::Etc::sourcelist=sources.list.d/ros2.sources \
  -o Dir::Etc::sourceparts=- -o APT::Get::List-Cleanup=0
apt-get update -o Dir::Etc::sourcelist=sources.list.d/gazebo-stable.list \
  -o Dir::Etc::sourceparts=- -o APT::Get::List-Cleanup=0
apt-get update -o Dir::Etc::sourcelist=sources.list \
  -o Dir::Etc::sourceparts=- -o APT::Get::List-Cleanup=0
packages=(
  ros-humble-ros-base ros-humble-rviz2
  ros-humble-sensor-msgs-py ros-humble-tf2-ros
  ros-humble-ros-gzharmonic python3-colcon-common-extensions
  ros-humble-ament-cmake ros-humble-rosidl-default-generators
  ros-humble-diagnostic-updater ros-humble-visualization-msgs
  ros-humble-mavlink ros-humble-geographic-msgs ros-humble-tf2-eigen
  ros-humble-eigen-stl-containers ros-humble-angles
  libgeographic-dev libasio-dev libeigen3-dev
  ros-humble-pluginlib ros-humble-eigen3-cmake-module
  ros-humble-std-srvs ros-humble-control-msgs ros-humble-trajectory-msgs
  ros-humble-ament-cmake-python ros-humble-message-filters
  python3-numpy python3-scipy python3-matplotlib python3-pytest
)
# This fails instead of removing existing packages if a dependency conflicts.
apt-get --no-remove --simulate install "${packages[@]}"
DEBIAN_FRONTEND=noninteractive apt-get --no-remove -y install "${packages[@]}"
if [[ ! -f /usr/share/GeographicLib/geoids/egm96-5.pgm ]]; then
  printf 'GeographicLib dataset missing: install it from the pinned MAVROS source.\n' >&2
  exit 2
fi
printf 'ROS 2 Humble simulation dependencies installed.\n'
