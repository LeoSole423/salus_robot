#!/usr/bin/env bash
set -euo pipefail

repo_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${repo_dir}"

visual=true
if [[ "${1:-}" == "--headless" ]]; then
  visual=false
elif [[ $# -gt 0 ]]; then
  echo "Usage: ./tools/sim.sh [--headless]" >&2
  exit 2
fi

docker compose up -d --build

if [[ "${visual}" == "true" ]]; then
  xhost +local:docker >/dev/null
  trap 'xhost -local:docker >/dev/null 2>&1 || true' EXIT
  launch_args="gz_args:=-r rviz:=true"
else
  launch_args="gz_args:=-r\ -s rviz:=false"
fi

docker compose exec ros2 bash -lc "
  set -e
  source /opt/ros/humble/setup.bash
  cd /ros2_ws
  colcon build --symlink-install --packages-up-to salus_bringup
  source /ros2_ws/install/setup.bash
  exec ros2 launch salus_bringup integration_sim.launch.py ${launch_args}
"
