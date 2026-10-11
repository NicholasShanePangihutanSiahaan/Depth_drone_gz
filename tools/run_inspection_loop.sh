#!/usr/bin/env bash
# Narrow simulation-only entry point suitable for a remembered approval rule.
set -eo pipefail
project_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$project_dir"
source "$project_dir/tools/ros_environment.sh"
export POLINASI_SIM_GUI=1
export POLINASI_GAZEBO_GUI=0
export POLINASI_CAMERA_GUI=1
export POLINASI_SIM_RTF=0.15
exec /usr/bin/python3 "$project_dir/tools/run_inspection_loop.py" "$@"
