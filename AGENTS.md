# AGENTS

## Project status

- Project: `salus_robot`, clean-room successor to `ROS2_SALUS`.
- ROS distribution: Humble on Ubuntu 22.04.
- Current milestone: control/battery parity in simulation; full robot remains
  non-operational.
- Never claim that a skeleton launch can operate or move the robot.

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
