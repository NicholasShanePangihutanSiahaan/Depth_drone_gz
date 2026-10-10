#!/usr/bin/env bash
set -euo pipefail
project_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
ap_dir="$project_dir/.dependencies/ardupilot"
test -x "$ap_dir/build/sitl/bin/arducopter"
run_name="${1:-sitl-run}"
if [[ ! "$run_name" =~ ^[a-zA-Z0-9_-]+$ ]]; then exit 2; fi
mkdir -p "$project_dir/.dependencies/$run_name"
cd "$project_dir/.dependencies/$run_name"
# Separate persistent parameter/log directory. No serial devices or real FCU.
extra_defaults="${POLINASI_SITL_EXTRA_DEFAULTS:-}"
if [[ -n "$extra_defaults" ]]; then test -f "$extra_defaults"; extra_defaults=",$extra_defaults"; fi
exec "$ap_dir/build/sitl/bin/arducopter" --model JSON --speedup 1 --sim-address 127.0.0.1 \
  --defaults "$ap_dir/Tools/autotest/default_params/copter.parm,$ap_dir/Tools/autotest/default_params/gazebo-iris.parm,$project_dir/polinasi_nav/config/sitl.parm$extra_defaults" \
  --home=-35.363261,149.165230,584,90 --serial0=udpclient:127.0.0.1:14550
