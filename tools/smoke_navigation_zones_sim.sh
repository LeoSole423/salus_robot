#!/usr/bin/env bash
set -euo pipefail
repo_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${repo_dir}"
docker compose run --rm ros2 bash -lc '
  set -eo pipefail
  export ROS_DOMAIN_ID=45 GZ_PARTITION="salus_zones_$$"
  source /opt/ros/humble/setup.bash; source /ros2_ws/install/setup.bash; set -u
  source /ros2_ws/tools/smoke_harness.sh
  smoke_init keepout-runtime
  trap smoke_cleanup EXIT
  runtime_dir="runtime/zones-smoke-$$"
  free_world="$(ros2 pkg prefix salus_simulation)/share/salus_simulation/worlds/free.world"
  smoke_start_launch zones "ros2 launch salus_bringup integration_sim.launch.py zones_runtime_dir:=${runtime_dir} world:=${free_world}"
  smoke_wait_node /zones_manager 40
  smoke_wait_node /keepout_filter_mask_server 40
  smoke_wait_lifecycle /keepout_filter_mask_server 40
  smoke_wait_topic /keepout_filter_mask 30
  test "$(ros2 topic type /costmap_filter_info)" = "nav2_msgs/msg/CostmapFilterInfo"
  smoke_run navigation_zones "python3 /ros2_ws/tools/smoke_navigation_zones_sim.py"
  smoke_note "keepout_mask_runtime_valid"
'
