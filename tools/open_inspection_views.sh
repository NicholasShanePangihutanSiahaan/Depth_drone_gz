#!/usr/bin/env bash
set -eo pipefail
project_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source "$project_dir/tools/ros_environment.sh"
exec /usr/bin/python3 "$project_dir/tools/open_inspection_views.py" "$@"
