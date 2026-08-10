#!/usr/bin/env bash
set -euo pipefail
repo_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${repo_dir}"
docker compose run --rm ros2 bash -lc \
  'source /opt/ros/humble/setup.bash && colcon build --symlink-install --event-handlers console_direct+'

