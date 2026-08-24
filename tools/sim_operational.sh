#!/usr/bin/env bash
set -euo pipefail

repo_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${repo_dir}"

headless=false
rviz=false
for option in "$@"; do
  case "${option}" in
    --headless) headless=true ;;
    --rviz) rviz=true ;;
    *)
      echo "Usage: ./tools/sim_operational.sh [--headless] [--rviz]" >&2
      exit 2
      ;;
  esac
done

if [[ "${headless}" == "true" && "${rviz}" == "true" ]]; then
  echo "--rviz cannot be used with --headless" >&2
  exit 2
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
  exec ros2 launch salus_bringup sim_operational.launch.py headless:=${headless} rviz:=${rviz}
"
