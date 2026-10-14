#!/usr/bin/env bash
# Bounded headless validation; stop on failure, never retry blindly.
set -eo pipefail
project_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$project_dir"
stage="${1:-inspection}"
scenario="${2:-palm_farm}"
case "$stage" in hover|inspection) ;; *) exit 2 ;; esac
case "$scenario" in palm_farm) ;;
  palm_farm_flowers)
    baseline="${3:-}"
    [[ -f "$baseline" ]] || { echo 'A completed old-map baseline summary is required before the flower world'; exit 2; }
    /usr/bin/python3 tools/check_sensor_mission_completion.py "$baseline" || exit 2 ;;
  *) exit 2 ;;
esac
source tools/ros_environment.sh
folder="$project_dir/reports/sensor_localisation_validation_$(date +%Y%m%d_%H%M%S)"
mkdir -p "$folder"
echo "Validation logs: $folder"
touch "$folder/launcher.log"
bash tools/start_sensor_localisation.sh "$stage" "$scenario" > "$folder/launcher.log" 2>&1 &
launcher_pid=$!
run_dir=""
audit_pid=""
cleanup() {
  if [[ -n "$audit_pid" ]]; then
    kill -INT "$audit_pid" 2>/dev/null || true
    wait "$audit_pid" || true
  fi
  if [[ -n "$run_dir" ]]; then
    /usr/bin/python3 tools/stop_simulation.py --run "$run_dir" || true
  else
    kill -TERM "$launcher_pid" 2>/dev/null || true
  fi
  wait "$launcher_pid" || true
}
trap cleanup EXIT INT TERM
for attempt in {1..120}; do
  if ! kill -0 "$launcher_pid" 2>/dev/null; then
    echo 'Launcher failed; inspect launcher.log'; exit 1
  fi
  run_dir="$(sed -n 's/.*Logs: //p' "$folder/launcher.log" | tail -n 1)"
  if [[ -n "$run_dir" ]]; then break; fi
  sleep 1
done
[[ -n "$run_dir" ]] || exit 1
printf '%s\n' "$run_dir" > "$folder/run.txt"
.venv/bin/python tools/set_origin.py --timeout 120 > "$folder/origin.log" 2>&1
setup_ready=0
for attempt in {1..3}; do
  /usr/bin/python3 tools/check_ros_simulation.py --output "$folder/setup.json" > "$folder/setup.log" 2>&1 || true
  if /usr/bin/python3 -c 'import json,sys;sys.exit(not json.load(open(sys.argv[1]))["passed"])' "$folder/setup.json"; then
    setup_ready=1; break
  fi
done
[[ "$setup_ready" == 1 ]] || { echo 'Setup failed; no arming command sent'; exit 1; }
/usr/bin/python3 tools/audit_sensor_localisation.py --output "$folder/accuracy.jsonl" --seconds 5400 --stop-run "$run_dir" > "$folder/audit.log" 2>&1 &
audit_pid=$!
/usr/bin/python3 tools/start_mission.py --timeout 120 > "$folder/start.log" 2>&1
echo "Mission started ($stage). Audit: $folder/audit.log"
wait "$audit_pid"
/usr/bin/python3 -c 'import json,sys;c=json.load(open(sys.argv[1]));print(json.dumps({k:v for k,v in c.items() if k not in ("last_navigation","last_localisation_debug")}));sys.exit(bool(c["stop_reason"]) or c["last_navigation"].get("mission")!="COMPLETE")' "$folder/accuracy.summary.json"
