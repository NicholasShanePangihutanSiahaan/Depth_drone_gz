#!/usr/bin/env bash
set -euo pipefail
project_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
scenario="${1:-clear_orbit}"
case "$scenario" in clear_orbit|leaf_blocked|vertical_detour|fully_blocked|sensor_dropout|palm_farm|identification) ;; *) exit 2 ;; esac
plugin_dir="$project_dir/.dependencies/ardupilot_gazebo"
test -f "$plugin_dir/build/libArduPilotPlugin.so"
export GZ_SIM_SYSTEM_PLUGIN_PATH="$plugin_dir/build${GZ_SIM_SYSTEM_PLUGIN_PATH:+:$GZ_SIM_SYSTEM_PLUGIN_PATH}"
export GZ_SIM_RESOURCE_PATH="$project_dir/polinasi_nav/models:$plugin_dir/models${GZ_SIM_RESOURCE_PATH:+:$GZ_SIM_RESOURCE_PATH}"
# A private partition isolates this test from other running Gazebo simulations.
export GZ_PARTITION=polinasi_lidar
world_path="${POLINASI_SIM_WORLD:-$project_dir/polinasi_nav/worlds/$scenario.sdf}"
if [[ -n "${POLINASI_SIM_WORLD:-}" ]]; then
  run_resource_dir="$(dirname -- "$world_path")/models"
  export GZ_SIM_RESOURCE_PATH="$run_resource_dir:$GZ_SIM_RESOURCE_PATH"
fi
test -f "$world_path"
exec gz sim -s -r --headless-rendering -v3 "$world_path"
