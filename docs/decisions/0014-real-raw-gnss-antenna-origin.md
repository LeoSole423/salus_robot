# 0014 — Identify real raw GNSS measurements at the antenna

Status: accepted, 2026-10-14.

The real MAVROS raw GPS fix is receiver/antenna LLA but arrives labelled
base_link. Native navsat_transform therefore sees a zero lever arm despite the
URDF's correct gps_link mount. Real hardware composition now explicitly maps
validated raw base_link fixes to gps_link at the Pixhawk adapter, and configures
the GNSS selector to expect gps_link. Topic names, type and QoS remain unchanged;
/salus/gps/fix now identifies the physical receiver origin. Consumers using only
LLA keep the same numeric measurement; TF consumers must use its actual header.
The generic observation profile and adapter default preserve base_link.

We retain native navsat compensation and unchanged EKF inputs/masks. We do not
numerically compensate in the adapter, relabel fused Pixhawk position, change
IMU calibration, or reinterpret a geodetic datum. This avoids double removal and
shared MAVROS frame parameter side effects. Restart the localization composition
when activating this origin correction, preserving private configuration.

Evidence: ../migration-evidence/intent/real-gnss-antenna-frame.md.
