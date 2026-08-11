#!/usr/bin/env bash
set -euo pipefail
repo_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${repo_dir}"
if ! docker compose ps --status running --services | grep -qx ros2; then
  echo "La simulacion no esta activa. Ejecuta primero: ./tools/sim.sh" >&2
  exit 1
fi

docker compose exec ros2 bash -lc \
  'source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash && exec bash'
