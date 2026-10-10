#!/usr/bin/env bash
# GUI only: attach to the existing private simulation, never start another server.
set -eo pipefail
project_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source "$project_dir/tools/ros_environment.sh"
plugin_dir="$project_dir/.dependencies/ardupilot_gazebo"
export GZ_SIM_RESOURCE_PATH="$project_dir/polinasi_nav/models:$plugin_dir/models${GZ_SIM_RESOURCE_PATH:+:$GZ_SIM_RESOURCE_PATH}"
exec gz sim -g
