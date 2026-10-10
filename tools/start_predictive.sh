#!/usr/bin/env bash
set -eo pipefail
project_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
preset="${1:-arena}"
export POLINASI_SIM_RTF="${POLINASI_SIM_RTF:-0.2}"
case "$preset" in
  arena) scenario=identification; mission=identification ;;
  smoke|tour|tour_check) scenario=palm_farm; mission=mapping ;;
  *) printf 'Use arena, smoke, tour, or tour_check.\n' >&2; exit 2 ;;
esac
exec bash "$project_dir/tools/start_simulation.sh" ground_truth "$scenario" "$mission" \
  "$project_dir/polinasi_nav/config/predictive_${preset}.json"
