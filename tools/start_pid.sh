#!/usr/bin/env bash
# GPS-disabled simulation; PID feedback, one PVA publisher, no MPC worker.
set -eo pipefail
project_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
preset="${1:-tour}"
mode="${2:-ground_truth}"
case "$preset" in tour|tour_check|smoke|check) ;; *) printf 'Use tour, tour_check, smoke, or check.\n' >&2; exit 2 ;; esac
export POLINASI_SIM_RTF="${POLINASI_SIM_RTF:-0.2}"
exec bash "$project_dir/tools/start_simulation.sh" "$mode" palm_farm mapping \
  "$project_dir/polinasi_nav/config/pid_${preset}.json"
