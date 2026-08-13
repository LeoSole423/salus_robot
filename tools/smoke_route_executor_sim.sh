#!/usr/bin/env bash
set -euo pipefail
repo_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${repo_dir}"
docker compose run --rm ros2 bash -lc '
  set -eo pipefail
  export ROS_DOMAIN_ID=47 GZ_PARTITION="salus_routes_$$"
  source /opt/ros/humble/setup.bash; source /ros2_ws/install/setup.bash; set -u
  source /ros2_ws/tools/smoke_harness.sh
  smoke_init routes-free-world
  trap smoke_cleanup EXIT
  free_world="$(ros2 pkg prefix salus_simulation)/share/salus_simulation/worlds/free.world"
  smoke_start_launch routes "ros2 launch salus_bringup integration_sim.launch.py launch_routes:=true world:=${free_world}"
  smoke_wait_node /route_executor 40
  for node in /bt_navigator /planner_server /controller_server; do smoke_wait_lifecycle "${node}" 40; done
  smoke_wait "service:/fromLL" 40 "services=\"\$(ros2 service list 2>/dev/null || true)\"; grep -qx /fromLL <<<\"\${services}\""
  smoke_wait "action:/navigate_to_pose" 40 "actions=\"\$(ros2 action list 2>/dev/null || true)\"; grep -qx /navigate_to_pose <<<\"\${actions}\""
  smoke_run route_executor "python3 /ros2_ws/tools/smoke_route_executor_sim.py"
  smoke_note "route_mission_progress_valid"
'
