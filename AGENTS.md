# AGENTS

## Project status

- Project: `salus_robot`, clean-room successor to `ROS2_SALUS`.
- ROS distribution: Humble on Ubuntu 22.04.
- Current milestone: the integrated simulation includes control, localization,
  3D LiDAR, safety, Nav2, routes, patrol/HOME, Cockpit and simulated PTZ.
- Hardware adapters and final `sim.launch.py` / `real.launch.py` remain pending.
- Never claim hardware parity without bank, bag or robot evidence.

## Sources of truth

1. Public ROS contracts: `src/salus_interfaces`.
2. Runtime composition: `src/salus_bringup/launch`.
3. Ownership and dependency rules: `docs/package-map.yaml`.
4. Architectural decisions: `docs/decisions/`.
5. Migration status and evidence: `docs/migration-status.yaml`.

The old `ROS2_SALUS`, `cockpit`, and firmware repositories are external
references. Do not edit them from work scoped to this repository. Do not copy
legacy code unless the migration map classifies it and an ADR records any
architectural compromise.

## Required workflow

Read [`docs/agent-development-workflow.md`](docs/agent-development-workflow.md)
before generating code. It is the source of truth for historical
characterization, pure-domain design, branch/PR handling, SOL/Terra handoffs,
tests, CI diagnosis and merge criteria.

For migration work:

1. Investigate the relevant legacy commits, tests and current runtime first.
2. Write or update an intent sheet under `docs/migration-evidence/intent/`.
3. Preserve public contracts unless an ADR approves a compatibility plan.
4. Implement pure policies/state machines before thin ROS adapters.
5. Add characterization tests before changing behavior.
6. Work on one `agent/*` branch and one bounded PR at a time.
7. Do not merge until local evidence and all required CI jobs are green.

## Repository rules

- Every ROS package starts with `salus_` and has one subsystem owner.
- Shared message/service/action definitions belong only in `salus_interfaces`.
- Complete real/sim compositions belong only in `salus_bringup`.
- Subsystem packages may expose partial launches for isolated tests.
- Hardware-specific code cannot be imported by simulation or core algorithms.
- Keep real and simulated backends behind the same documented contract.
- Do not vendor third-party sources under `src`; pin them in `dependencies.repos`.
- New parameters need type, default, unit, valid range, and operational meaning.
- New topics/services/actions need producer, consumer, type, QoS, and lifecycle.
- Avoid absolute host paths and hidden environment assumptions.

## Commands

```bash
./tools/up.sh
./tools/build.sh
./tools/test.sh
./tools/smoke_control_sim.sh
./tools/smoke_motion_sim.sh
./tools/smoke_localization_sim.sh
./tools/smoke_safety_sim.sh
./tools/smoke_navigation_core_sim.sh
./tools/smoke_navigation_zones_sim.sh
./tools/smoke_route_executor_sim.sh
./tools/smoke_patrol_battery_sim.sh
./tools/smoke_navigation_snapshot.sh
./tools/smoke_web_cockpit.sh
./tools/smoke_integration_sim.sh
./tools/sim.sh
./tools/cmd_vel_sim.sh straight
./tools/shell.sh
```

Run focused package tests while iterating, then the full repository validation.
Do not run a real launch against hardware without an explicit operator request
and an accessible E-stop.

## Definition of done

- package README and `docs/package-map.yaml` agree;
- build and tests pass in the Humble container;
- public contracts and runtime wiring are documented;
- real/sim behavior and failure modes have tests;
- migration status and relevant ADRs are updated.
- the PR records intent, scope, evidence, limitations and hardware status;
- `build-unit`, `simulation-core` and `navigation-missions` are green when
  required by the changed boundaries.
