#!/usr/bin/env bash
set -euo pipefail
repo_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${repo_dir}"
python3 tools/validate_repository.py
python3 -m unittest tools/test_real_runtime_cache.py
docker compose run --rm ros2 bash -lc \
  'source /opt/ros/humble/setup.bash && colcon build --symlink-install && source install/setup.bash && colcon test --packages-skip rslidar_msg --event-handlers console_direct+ && colcon test-result --verbose && bash tools/test_smoke_harness.sh'
