# Issue #61 — global yaw-rate authority

## Scope

The global EKF currently receives yaw-rate from both the local filtered
odometry path and `/imu/data_global`. This cut characterizes the two single-
source alternatives against the current duplicate-fusion baseline using the
independent nominal simulation profile from #64.

## Frozen variants

- `baseline_duplicate`: current `odom0` yaw-rate plus `imu0` yaw-rate; reference
  only.
- `local_odom_yaw_rate`: `odom0` provides `vx`, `vy`, and yaw-rate; `imu0`
  yaw-rate is disabled.
- `direct_imu_yaw_rate`: `odom0` provides `vx` and `vy` only; `imu0` provides
  yaw-rate.

All variants preserve GNSS X/Y, global orientation, stationary gating, frames,
timeouts and covariances. The local EKF, Nav2, safety and real sensor sources
are outside this cut.

## Implementation boundary

Simulation accepts an optional `global_ekf_params_file`; the default remains
the existing baseline while A/B/C characterization runs. Matrix provenance
records the selected global YAML and its SHA-256. The final paired campaign
selected `local_odom_yaw_rate` as the single production authority. The sim and
real defaults now disable `imu0` yaw-rate while retaining the
`/imu/data_global` publication for other consumers.

## Evidence interpretation

The evaluator's terminal outcome remains separate from Nav2 terminal success.
Position/yaw error and stationary behavior are evidence; no new performance
gate is introduced. If the two single-source candidates are equivalent within
the paired simulation noise, `local_odom_yaw_rate` is preferred because the
global EKF then consumes the already-filtered local motion estimate without
re-fusing its IMU contribution. The final campaign used 45 trials (15 per
variant, with paired seeds). B and C had comparable position, covariance,
Nav2-terminal, and functional outcomes; C showed a modestly lower yaw error in
this synthetic sample, but no consistent functional advantage. B was selected
for the cleaner hierarchy. A remains the duplicate-fusion reference only.
