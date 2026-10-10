# Battery startup observation — 2026-10-10

Jetson source d6afc04 initially failed startup because the ESP32 configuration
and battery adapter both selected /dev/ttyUSB0 after USB enumeration changed.
With operator authorization and accessible E-stop, the effective host wrapper
was corrected to resolve the ESP32 CP2102 by stable /dev/serial/by-id identity,
validate the physical device and map it into the container as /dev/salus_drive.
The CH340 battery retains its own stable identity and /dev/salus_battery mapping.
A backup preceded the edit; bash syntax validation passed. The existing service
retry policy restarted the runtime successfully.

Read-only evidence: battery approximately 49.73 V, -0.9 A, 45/50 Ah, 90%,
21 °C and 15 cell voltages. Battery mission guard was fresh, ready and OK.
Nav2 startup reported all lifecycle nodes active; odometry, TF and scan were
fresh. Drive telemetry remained disabled, source NONE, speed 0.0 m/s.
WebSocket and Cockpit displayed coherent battery data under STARTUP_LOCKED.
At this checkpoint Cockpit build passed and tests were 193 passed/1 skipped.

No driving, low-battery HOME, BMS disconnection or reboot persistence was tested.
MAVROS reported no GPS fix. Host systemd/environment files still contain old
/dev/ttyUSB0 references including ExecStartPre; the effective wrapper overrides
the launch port. Consolidating those privileged files remains maintenance work.
This is evidence of a host deployment correction, not a change to the generic
repository launcher. Private deployment files and raw evidence remain external.
