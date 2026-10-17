# 0015 — Observe BMI088 with explicit USB timing before sensor cutover

Status: accepted for implementation, 2026-10-15.

Use a read-only salus_hardware serial owner polling timed diagnostic type 03,
with request token, device boot ID and MCU receive/capture timestamps. Derive
ROS sample timestamps from the host request/response interval, subtract sample
age and optional configured filter delay; reject excessive latency, invalid
chronology, duplicates and stale samples. Report timing uncertainty rather than
claiming hardware synchronization or a synchronized MCU clock.

Keep gyro and acceleration in separate sensor_msgs/Imu streams, each stamped
at its own acquisition boundary. Orientation is unavailable. Calibration v1
is applied in the sensor frame with raw outputs retained. Mounting is TF, never
an undocumented frame relabel. No automatic fallback or publication on the
operational /salus/imu/data topic. Comparison transforms gyro vectors into a
common frame and pairs timestamps, reporting statistical metrics only.

This is an observation backend, not the final shared control/IMU USB gateway.
It cannot coexist with the legacy control reader on the same S3 port. That
future gateway and higher-rate multiplexing require another bounded change.
No hardware parity or Pixhawk removal is implied: GNSS/RTCM remain separate.
