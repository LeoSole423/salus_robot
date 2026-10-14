# Real GNSS antenna origin — 2026-10-14

## Facts and characterization

Jetson stationary passive audit on main dd45050: robot_localization 3.5.4,
MAVROS 2.14.0; /global_position/raw/fix, /hardware/gnss_primary/fix and
/salus/gps/fix all carry base_link. TF base_footprint -> base_link is zero;
base_footprint -> gps_link is (0.66, 0.20, 0.62) m, matching the URDF.
The active native navsat_transform consumes /salus/gps/fix and /odometry/global.
No motion or command publisher was used for this audit.

MAVROS GPS_RAW_INT handler copies raw receiver LLA, unlike its fused global
position handler. Native navsat_transform looks up base -> fix.header.frame_id
and removes the rotated offset using world -> base TF at the measurement stamp:
https://github.com/mavlink/mavros/blob/2.14.0/mavros/src/plugins/global_position.cpp
https://github.com/cra-ros-pkg/robot_localization/blob/3.5.4/src/navsat_transform.cpp

## Decision and compatibility

Normalize only the validated raw GNSS origin at the hardware adapter. Keep its
expected incoming base_link identity, all numeric fields, covariance, status,
timestamp and IMU identity unchanged. Explicit gnss_output_frame=gps_link in
real_hardware also configures the downstream selector. Generic observation
launches retain the original frame by default. This is metadata identification
of raw antenna coordinates, not an additional numeric compensation. Do not
apply it to Pixhawk fused global position or already compensated coordinates.
No MAVROS parameters change: its shared frame parameters also label fused data.
See ADR 0014 for the real public frame correction.

## Verification and limits

Characterization tests preserve the default output and NO_FIX, verify the real
wiring, reject unexpected input frames, and preserve every field except the
explicit origin metadata. A native navsat software fixture will check rotating
lever arms under a manual datum. Full build/tests and selected CI are required.
Stationary Jetson verification can establish the frame/TF and corrected position
path. Turn accuracy on physical hardware remains unvalidated until an operator
runs a measured driving test. Do not transfer the simulated ~2 degree result to
hardware as a guarantee. Preserve private datum/calibrations and source/cache
rollback during activation; coordinate localization restart after origin change.

## Local evidence

The software-only `tools/check_real_gnss_antenna.py` drives the production raw
adapter and GNSS selector into two native navsat_transform instances on isolated
ROS domain 159, using the real navsat configuration with a synthetic datum and
TF matching the URDF. Compared with the original base_link input, corrected
outputs remove the expected rotated (0.66, 0.20) m arm at 0, +69.3, -69.3 and
90 degrees (error below 0.005 m). This checks geometry, not EKF driving accuracy.
The fixture is registered in the bringup test suite and publishes no commands.
