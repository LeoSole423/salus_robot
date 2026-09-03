# Physical static TF owner

Issue #188 ports the minimum real-description owner needed for stationary
RS16/perception preflight. `salus_description/launch/description_real.launch.py`
starts exactly one `robot_state_publisher` and supplies its `robot_description`
from the canonical `urdf/salus_robot.urdf.xacro` with `use_sim:=false` and
`use_sim_time=false`.

The launch owns only transforms derived from the URDF. Structural and isolated
runtime tests verify `/tf_static` contains the fixed
`base_footprint -> base_link` and `base_link -> lidar_link` transforms, with
the LiDAR mount at `xyz=0.92 0 0.65`, `rpy=0 0.1745 0`. Existing IMU/GPS mounts
are also checked. The false-simulation expansion contains no Gazebo plugins.

The profile does not start joint-state publishing, localization, RS16,
perception, MAVROS, NTRIP, Nav2, UART or Gazebo, and it never creates
`odom -> base_footprint` or `map -> odom`. No Jetson, hardware launch or robot
movement is part of this evidence; physical validation remains pending.
