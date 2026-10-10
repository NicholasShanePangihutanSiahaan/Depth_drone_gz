#!/usr/bin/env bash
set -eo pipefail
project_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
profile="${1:-train}"
case "$profile" in train|validation) ;; *) printf 'Use train or validation.\n' >&2; exit 2 ;; esac
export POLINASI_SIM_RTF="${POLINASI_SIM_RTF:-0.2}"
exec bash "$project_dir/tools/start_simulation.sh" ground_truth identification identification \
  "$project_dir/polinasi_nav/config/identification_${profile}.json"
