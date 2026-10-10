#!/usr/bin/env bash
# Separate simulated mapping mission. Default tour; smoke is a small launch-pad loop.
set -eo pipefail
project_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
preset="${1:-tour}"
mode="${2:-ground_truth}"
case "$preset" in tour|smoke|check) ;; *) printf 'Preset must be tour, smoke, or check.\n' >&2; exit 2 ;; esac
# Wall-clock pacing only: retain the 1 ms physics step and all safety limits.
# Override with POLINASI_SIM_RTF=0.2 on machines unable to sustain the default.
export POLINASI_SIM_RTF="${POLINASI_SIM_RTF:-0.2}"
printf 'Mapping preset: %s; requested simulation time factor: %s.\n' "$preset" "$POLINASI_SIM_RTF"
exec bash "$project_dir/tools/start_simulation.sh" "$mode" palm_farm mapping \
  "$project_dir/polinasi_nav/config/mapping_${preset}.json"
