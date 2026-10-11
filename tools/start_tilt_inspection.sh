#!/usr/bin/env bash
set -eo pipefail
project_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
export POLINASI_SIM_RTF="${POLINASI_SIM_RTF:-0.15}"
export POLINASI_GAZEBO_GUI=0
export POLINASI_CAMERA_GUI=1
exec bash "$project_dir/tools/start_simulation.sh" ground_truth palm_farm mapping \
  "$project_dir/polinasi_nav/config/tilt_inspection.json"
