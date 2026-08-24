#!/usr/bin/env bash
set -euo pipefail

repo_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${repo_dir}"

visual=true
cockpit=false
for option in "$@"; do
  case "${option}" in
    --headless)
      visual=false
      ;;
    --cockpit)
      cockpit=true
      ;;
    *)
      echo "Usage: ./tools/sim.sh [--headless] [--cockpit]" >&2
      exit 2
      ;;
  esac
done

docker compose up -d --build

if [[ "${visual}" == "true" ]]; then
  xhost +local:docker >/dev/null
  trap 'xhost -local:docker >/dev/null 2>&1 || true' EXIT
  launch_args="gz_args:=-r rviz:=true"
else
  launch_args="gz_args:=-r\ -s rviz:=false"
fi

if [[ "${cockpit}" == "true" ]]; then
  launch_args+=" launch_routes:=true launch_patrol:=true launch_web:=true web_ws_port:=8766"
  echo "Cockpit backend available at ws://localhost:8766"
  echo "In the Cockpit repository, run: npm run dev"
fi

docker compose exec ros2 bash -lc "
  set -e
  source /opt/ros/humble/setup.bash
  cd /ros2_ws
  colcon build --symlink-install --packages-up-to salus_bringup
  source /ros2_ws/install/setup.bash
  exec ros2 launch salus_bringup integration_sim.launch.py ${launch_args}
"
