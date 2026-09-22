#!/usr/bin/env bash
set -euo pipefail

repo_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${repo_dir}"

headless=false
rviz=false
adaptive_dense=false
for option in "$@"; do
  case "${option}" in
    --headless) headless=true ;;
    --rviz) rviz=true ;;
    --adaptive-dense) adaptive_dense=true ;;
    *)
      echo "Usage: ./tools/sim_operational.sh [--headless] [--rviz] [--adaptive-dense]" >&2
      exit 2
      ;;
  esac
done

if [[ "${headless}" == "true" && "${rviz}" == "true" ]]; then
  echo "--rviz cannot be used with --headless" >&2
  exit 2
fi

route_args=""
if [[ "${adaptive_dense}" == "true" ]]; then
  route_args=" route_execution_mode:=adaptive_dense adaptive_dense_leg_max_m:=8.0 adaptive_dense_horizon_m:=35.0"
fi

docker compose up -d --build

if [[ "${headless}" == "false" ]]; then
  xhost +local:docker >/dev/null
  trap 'xhost -local:docker >/dev/null 2>&1 || true' EXIT
fi

echo "Cockpit backend available at ws://localhost:8766"
echo "In the Cockpit repository, run: git switch migration/salus-robot-cockpit && npm run dev"

docker compose exec ros2 bash -lc "
  set -e
  source /opt/ros/humble/setup.bash
  cd /ros2_ws
  colcon build --symlink-install --packages-up-to salus_bringup
  source /ros2_ws/install/setup.bash
  exec ros2 launch salus_bringup sim_operational.launch.py headless:=${headless} rviz:=${rviz}${route_args}
"
