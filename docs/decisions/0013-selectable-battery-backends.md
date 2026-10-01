# ADR 0013 — replaceable battery measurement backends

Accepted, 2026-10-01.

Drive transport and battery measurement have independent ownership. Real MVP
uses Pylontech US2000 RS485 in salus_hardware, selected by backend name and an
explicit serial device. Backend implementations return the same immutable
BatterySample. The thin battery node publishes sensor_msgs/BatteryState privately;
salus_control consumes that generic interface and retains authority over public
BatteryState, BatteryMissionGuard and existing controller JSON. No control-to-
hardware Python dependency is introduced. Future adapters implement read/close
and register in create_battery_backend; consumers require no battery-specific code.

Real UART profile selects external measurements; it does not consult ESP32
battery telemetry or silently fall back after a failure. ESP32 remains the drive
transport. Simulation keeps its existing transport measurements and presets.
Public message definitions and voltage guard thresholds remain compatible.
Operator SOC comes from the BMS capacity ratio. Missing data breaks the continuous
voltage guard timers without clearing an existing latch. Public stale BatteryState
has its original measurement timestamp, present=false and unknown scalar values.

Pylontech multi-module packs and other firmware revisions are rejected until
characterized. No Modbus assumption, console command or settings write is used.
Bench connectivity is proven; deployed ROS and real mission parity are pending.
