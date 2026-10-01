# Intent — selectable BMS battery source (2026-10-01)

User requests US2000 RS485 measurements instead of ESP32 ADC, with replaceable
battery adapters. Legacy f358349 established voltage/persistence mission guards;
0208160 separated operator SOC from mission authority. Current control couples
battery samples to drive transport. Preserve public BatteryState, JSON telemetry,
and BatteryMissionGuard; preserve voltage thresholds and simulation behavior.

Observed bench: Jetson CH340 needs ch341 for kernel 6.8.12-1021-tegra. After
installation, address 2 / 115200 8N1 replied with valid checksums, 49.80 V,
-0.4 A, 46.5/50 Ah (93%), 15 cells, five temperatures and 181 cycles.
Evidence was recorded on Jetson in ~/battery-diagnostics/readings.json.
This proves stationary serial connectivity, not deployed ROS integration,
calibration accuracy, mission behavior on hardware or hardware parity.

Design: hardware owns an extensible read-only BatteryBackend and immutable sample;
Pylontech parser is pure and strict. An isolated ROS node publishes fresh
sensor_msgs/BatteryState on /battery/backend_state. Control selects transport
(simulation/legacy) or external source, never silently falls back to ESP32,
and remains sole publisher of public state, mission guard and Cockpit JSON.
BMS SOC is remaining/total capacity, not a voltage curve. Existing voltage
mission policy continues using BMS voltage. Gaps break persistence continuity,
but never clear a latched guard. Timeout reports unavailable/stale explicitly.
Real MVP defaults to external Pylontech; simulation retains transport source.
No actuation launch or external repository edits are authorized by this cut.

## Implementation verification

- Humble build passed (16 packages).
- Full colcon suite passed: 1239 tests, 0 errors/failures, one skip; after the
  final standard-message and invalid-sample tests, focused rerun plus aggregate
  results: 1241 tests, 0 errors/failures, one skip.
- Host infrastructure tests: 36 passed.
- Control simulation smoke passed, including battery presets.
- New backend read the physical BMS in an isolated diagnostic directory on
  Jetson: 49.774 V, -0.4 A, 46.5/50 Ah, 93%, 181 cycles. No ROS motor runtime
  or repository deployment was started.
- Repository validator passes on a clean deliverable snapshot. Direct tools/test.sh
  preflight fails on pre-existing untracked .agents skill markdown (home paths and
  C++ syntax interpreted as links); no user skill files were edited or removed.
- Patrol/HOME smoke failed before battery-return testing: JOIN_LOOP timed out;
  route executor reported anchor selection no_near_segment. Baseline diagnosis
  against main is pending. Do not classify this failure as a battery regression
  without baseline evidence.
