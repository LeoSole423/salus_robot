#!/usr/bin/env bash
set -euo pipefail
repo_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${repo_dir}"

docker compose run --rm ros2 bash -lc '
  set -eo pipefail
  export ROS_DOMAIN_ID=46 GZ_PARTITION="salus_motion_$$"
  source /opt/ros/humble/setup.bash; source /ros2_ws/install/setup.bash; set -u
  source /ros2_ws/tools/smoke_harness.sh
  smoke_init motion-free-world
  trap smoke_cleanup EXIT
  free_world="$(ros2 pkg prefix salus_simulation)/share/salus_simulation/worlds/free.world"
  smoke_start_launch motion "ros2 launch salus_simulation motion_sim.launch.py world:=${free_world}"
  smoke_start_launch control "ros2 launch salus_control control_sim.launch.py"
  # The Gazebo /clock bridge can be discovered before ros2cli has a matching
  # reader. The functional Python scenario below waits on and validates the
  # actual odometry/joint-state messages.
  smoke_wait_topic /clock 30
  smoke_wait_topic /odom_raw 30
  smoke_wait_topic /joint_states 30
  python3 /ros2_ws/tools/smoke_motion_sim.py
  test "$(ros2 topic type /odom_raw)" = "nav_msgs/msg/Odometry"
  test "$(ros2 topic type /joint_states)" = "sensor_msgs/msg/JointState"
  test "$(ros2 topic info /tf -v | grep -c "Publisher count: 1")" -eq 1
  smoke_note "motion_contracts_valid"
'
