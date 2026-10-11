#!/usr/bin/env bash
# No mission start/arming: empty-floor A/B sensor experiment.
set -eo pipefail
project_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
variant="${1:-visible}"
case "$variant" in visible|hidden|fixture) ;; *) exit 2 ;; esac
export POLINASI_SIM_RTF="${POLINASI_SIM_RTF:-0.25}"
export POLINASI_GAZEBO_GUI=0
export POLINASI_CAMERA_GUI=1
exec bash "$project_dir/tools/start_simulation.sh" ground_truth identification mapping \
  "$project_dir/polinasi_nav/config/empty_mount_${variant}.json"
