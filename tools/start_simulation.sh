#!/usr/bin/env bash
# Supervise only our child processes. Ctrl-C stops this simulation cleanly.
set -eo pipefail
project_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
mode="${1:-ground_truth}"
scenario="${2:-clear_orbit}"
mission="${3:-inspection}"
case "$mission" in inspection|mapping|identification) ;; *) printf 'Mission must be inspection, mapping, or identification.\n' >&2; exit 2 ;; esac
case "$mode" in ground_truth|sensor) ;; *) printf 'Mode must be ground_truth or sensor.\n' >&2; exit 2 ;; esac
case "$scenario" in clear_orbit|leaf_blocked|vertical_detour|fully_blocked|sensor_dropout|palm_farm|identification) ;; *) exit 2 ;; esac
if [[ "$mission" == identification ]]; then
  [[ "$mode" == ground_truth && "$scenario" == identification ]] || exit 2
  export POLINASI_SITL_EXTRA_DEFAULTS="$project_dir/polinasi_nav/config/sitl_identification.parm"
fi
source "$project_dir/tools/ros_environment.sh"
test -f "$project_dir/install_ros/setup.bash"
if pgrep -f "$project_dir/install_ros/polinasi_nav/lib/polinasi_nav/(mapping_navigation|mapping_io|identification_navigation|identification_recorder|navigation)( |$)" >/dev/null; then
  printf 'An existing navigation process is still running. Stop that simulation first; starting another could create duplicate control publishers.\n' >&2
  exit 1
fi
if ss -H -lun 'sport = :9002 or sport = :14550' | grep -q .; then
  printf 'A process already uses simulation port 9002 or 14550. Stop that instance first.\n' >&2
  exit 1
fi
if ss -H -ltn 'sport = :5762' | grep -q .; then
  printf 'A process already uses SITL TCP port 5762. Stop the previous simulator first; do not start a second instance.\n' >&2
  exit 1
fi
run_label="ros_${mode}_${scenario}_$(date +%Y%m%d_%H%M%S)_$$"
if [[ "$mission" == mapping ]]; then run_label="${run_label}_mapping"; fi
if [[ "$mission" == identification ]]; then run_label="${run_label}_identification"; fi
run_dir="$project_dir/.dependencies/$run_label"
map_log_dir="$project_dir/reports/$run_label/map_3d"
nav_config="$project_dir/polinasi_nav/config/navigation.json"
if [[ "$scenario" == palm_farm ]]; then nav_config="$project_dir/polinasi_nav/config/palm_farm_navigation.json"; fi
if [[ "$mission" == mapping ]]; then nav_config="$project_dir/polinasi_nav/config/mapping_tour.json"; fi
if [[ "$mission" == identification ]]; then nav_config="$project_dir/polinasi_nav/config/identification_train.json"; fi
if [[ -n "${4:-}" ]]; then nav_config="$4"; fi
test -f "$nav_config"
mkdir -p "$run_dir"
if [[ "$mission" == mapping || "$mission" == identification ]]; then
  python3 "$project_dir/tools/prepare_sim_world.py" "$project_dir/polinasi_nav/worlds/$scenario.sdf" \
    "$run_dir/world.sdf" --factor "${POLINASI_SIM_RTF:-0.2}" --sensor-config "$nav_config"
  export POLINASI_SIM_WORLD="$run_dir/world.sdf"
  nav_config="$run_dir/mission_config.json"
fi
gazebo_pid=; sitl_pid=; ros_pid=; gazebo_gui_pid=; rviz_pid=
cleanup() {
  trap - EXIT INT TERM HUP
  for child in "$rviz_pid" "$gazebo_gui_pid" "$ros_pid" "$sitl_pid" "$gazebo_pid"; do
    if [[ -n "$child" ]] && kill -0 -- "-$child" 2>/dev/null; then
      kill -INT -- "-$child" 2>/dev/null || true
    fi
  done
  # Background SITL can inherit ignored SIGINT from bash. Give ROS/Gazebo a
  # brief graceful shutdown, then terminate only private process groups this
  # launcher owns, including ROS nodes and their spawned CPU workers.
  for attempt in {1..25}; do
    running=false
    for child in "$rviz_pid" "$gazebo_gui_pid" "$ros_pid" "$sitl_pid" "$gazebo_pid"; do
      if [[ -n "$child" ]] && kill -0 -- "-$child" 2>/dev/null; then running=true; fi
    done
    if [[ "$running" == false ]]; then break; fi
    sleep 0.2
  done
  for child in "$rviz_pid" "$gazebo_gui_pid" "$ros_pid" "$sitl_pid" "$gazebo_pid"; do
    if [[ -n "$child" ]] && kill -0 -- "-$child" 2>/dev/null; then
      kill -TERM -- "-$child" 2>/dev/null || true
    fi
  done
  sleep 1
  for child in "$rviz_pid" "$gazebo_gui_pid" "$ros_pid" "$sitl_pid" "$gazebo_pid"; do
    if [[ -n "$child" ]] && kill -0 -- "-$child" 2>/dev/null; then
      kill -KILL -- "-$child" 2>/dev/null || true
    fi
  done
  wait 2>/dev/null || true
  python3 "$project_dir/tools/stop_simulation.py" --mark-stopped "$run_dir" || true
}
trap cleanup EXIT INT TERM HUP
setsid bash "$project_dir/tools/run_gazebo.sh" "$scenario" >"$run_dir/gazebo.log" 2>&1 & gazebo_pid=$!
setsid bash "$project_dir/tools/run_sitl.sh" "$run_label" >"$run_dir/sitl.log" 2>&1 & sitl_pid=$!
drop_after=-1.0
if [[ "$scenario" == sensor_dropout ]]; then drop_after=25.0; fi
setsid ros2 launch polinasi_nav sitl_navigation.launch.py localisation:="$mode" \
  mission:="$mission" drop_after:="$drop_after" config:="$nav_config" map_log_dir:="$map_log_dir" >"$run_dir/ros.log" 2>&1 & ros_pid=$!
if [[ "${POLINASI_SIM_GUI:-1}" == 1 && -n "${DISPLAY:-}${WAYLAND_DISPLAY:-}" ]]; then
  setsid bash "$project_dir/tools/view_simulation.sh" >"$run_dir/gazebo_gui.log" 2>&1 & gazebo_gui_pid=$!
  setsid rviz2 -d "$project_dir/polinasi_nav/config/navigation.rviz" \
    --ros-args -p use_sim_time:=true >"$run_dir/rviz.log" 2>&1 & rviz_pid=$!
  printf 'Gazebo and RViz opened; this launcher also closes both when stopped.\n'
elif [[ "${POLINASI_SIM_GUI:-1}" == 1 ]]; then
  printf 'No desktop display is available; cannot show Gazebo/RViz.\n' >&2
  exit 1
fi
python3 "$project_dir/tools/stop_simulation.py" --register "$run_dir" --launcher "$$"
printf 'Simulation running (%s, %s mission). Logs: %s\n' "$mode" "$mission" "$run_dir"
if [[ "$mission" == identification ]]; then
  printf 'Empty identification arena: LiDAR/mapping disabled; response log: %s/identification.json\n' "$(dirname -- "$map_log_dir")"
else
  printf 'Saved 3D map: %s/map.html (updated every 10 simulation seconds and on clean shutdown).\n' "$map_log_dir"
fi
if [[ -z "$gazebo_gui_pid" ]]; then
  printf 'Open Gazebo separately if desired: bash %s/tools/view_simulation.sh\n' "$project_dir"
fi
printf 'To stop this run and both windows: python3 %s/tools/stop_simulation.py --run %s\n' "$project_dir" "$run_dir"
printf 'In another terminal: source %s/tools/ros_environment.sh\n' "$project_dir"
printf 'Then set origin with .venv/bin/python tools/set_origin.py and inspect /navigation/status.\n'
printf 'Mission waits for acknowledged start: python3 %s/tools/start_mission.py\n' "$project_dir"
wait -n "$gazebo_pid" "$sitl_pid" "$ros_pid"
