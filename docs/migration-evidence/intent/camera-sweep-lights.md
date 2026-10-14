# Camera sweep and steady lights

Intent: Cockpit commands one full mobile pan from physical left to right,
preserving tilt, with an explicit stop. Separate steady light controls use physically verified mechanisms for
the fixed and mobile lenses.
MediaMTX remains the video owner; salus_hardware owns ONVIF/LAPI and secrets.

Evidence: Uniarch firmware DIPC-B6109.1.120.251218 accepts LightCtrl PUT,
Type=1/Mode=1 or 2/Interval=60; operator confirmed flashing and steady light
on general index 0 (2026-10-14). LightCtrl indices 1, 2 and singular Channel/2 all lit only the fixed lens.
Operator confirmed mobile white light only with LampCtrl/Video channel 2 Type=1
and Exposure/Video DayNight.Mode=2. Both complete original profiles were restored
and GET matched the backups after the test.
Existing ONVIF mapping spans pan 0–350 degrees, tilt 0–90. Cockpit's corrected
horizontal arrows make increasing pan physical left. Existing presets stay intact.

Design: additive CameraFeatures service, controlled WebSocket start/on commands,
unrestricted stop/off commands. A timer advances a bounded sweep and renews
60-second light orders every 40 seconds. Light state is last acknowledged intent,
unknown at startup/after failure: GET LightCtrl is not physical telemetry.
Simulation implements the same surface. Manual PT commands cancel an active
sweep. Tests never address real hardware.

Validation: Humble build 16 packages; full first-party colcon results 1308 tests,
0 errors/failures (one skipped); hardware 187, web 73; focused policy/ONVIF/
protocol/mapping 36 passed; Cockpit build and 201 passed/1 skipped; extended
WebSocket smoke passed, including state, sweep start/stop, lights and off/stop
without lease. Host runtime tests 36 passed; smoke harness selftest passed.

The standard tools/test.sh preflight encounters pre-existing untracked .agents
documentation violations; validation of the tracked tree plus this change passes.
Historical rslidar_msg results remain excluded as prescribed; its stale lint
results do not represent this run. No user files or old results were removed.

Physical light mechanisms confirmed; new automatic sweep acceptance and real
runtime activation remain pending. No robot motion authorized. Working source
is based on salus_robot c6bed23 and current Cockpit main fd69a8e, whose camera
grid, corrected PTZ and rear stream are already integrated. The older migration
branch diverges; an attempted integration on a new branch was aborted cleanly
and changes are isolated on agent/camera-sweep-lights-current, preserving main.

