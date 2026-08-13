#!/usr/bin/env bash
set -euo pipefail
repo_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${repo_dir}"
docker compose run --rm ros2 bash -lc '
  set -eo pipefail
  export ROS_DOMAIN_ID=44 GZ_PARTITION="salus_navigation_$$"
  source /opt/ros/humble/setup.bash; source /ros2_ws/install/setup.bash; set -u
  source /ros2_ws/tools/smoke_harness.sh
  smoke_init navigation-free-world
  trap smoke_cleanup EXIT
  free_world="$(ros2 pkg prefix salus_simulation)/share/salus_simulation/worlds/free.world"
  smoke_start_launch navigation "ros2 launch salus_bringup integration_sim.launch.py world:=${free_world}"
  for node in /planner_server /controller_server /smoother_server /bt_navigator /behavior_server; do smoke_wait_node "${node}" 40; done
  smoke_wait_lifecycle /bt_navigator 40
  smoke_wait_topic /cmd_vel 30
  smoke_wait "service:/fromLL" 40 "services=\"\$(ros2 service list 2>/dev/null || true)\"; grep -qx /fromLL <<<\"\${services}\""
  smoke_wait "service:/path_health/evaluate" 40 "services=\"\$(ros2 service list 2>/dev/null || true)\"; grep -qx /path_health/evaluate <<<\"\${services}\""
  smoke_wait "action:/navigate_to_pose" 40 "actions=\"\$(ros2 action list 2>/dev/null || true)\"; grep -qx /navigate_to_pose <<<\"\${actions}\""
  test "$(ros2 topic type /cmd_vel_safe)" = "geometry_msgs/msg/Twist"
  test "$(ros2 topic type /cmd_vel_final)" = "salus_interfaces/msg/CmdVelFinal"
  test "$(ros2 topic type /plan)" = "nav_msgs/msg/Path"
  test "$(ros2 service type /path_health/evaluate)" = "salus_interfaces/srv/EvaluatePathHealth"
  smoke_run navigation_core "python3 /ros2_ws/tools/smoke_navigation_core_sim.py"
  smoke_note "navigation_goal_cancel_manual_valid"
'
