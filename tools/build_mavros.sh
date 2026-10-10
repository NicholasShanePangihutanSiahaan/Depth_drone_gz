#!/usr/bin/env bash
set -eo pipefail
project_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
POLINASI_ROS_SYSTEM_ONLY=1
source "$project_dir/tools/ros_environment.sh"
workspace="$project_dir/.dependencies/mavros_ws"
test -f "$workspace/src/mavros/mavros/package.xml"
cd "$workspace"
export MAKEFLAGS=-j2
export CMAKE_BUILD_PARALLEL_LEVEL=2
colcon build --packages-up-to mavros mavros_extras --parallel-workers 2 \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF \
  -DPython3_EXECUTABLE=/usr/bin/python3 \
  -DCMAKE_C_COMPILER=/usr/bin/gcc -DCMAKE_CXX_COMPILER=/usr/bin/g++
