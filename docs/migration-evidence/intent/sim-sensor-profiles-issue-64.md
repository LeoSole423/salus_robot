# Intent evidence: issue #64 simulation sensor profiles

## Scope

This Subagente C cut defines and wires the simulation-only profiles `clean`,
`independent_nominal` and `degraded`. It also makes profile and deterministic
seed identity visible in matrix and smoke artifacts.

The profiles are ROS 2 parameter files consumed by the simulation sensor
adapters.
They do not change real launches, real parameters, `/odom_raw`, or any
productive drive/IMU/GNSS path. `/odom_raw` remains evaluator ground truth.

## Contract

- Profile files live in `salus_simulation/config/sensor_profiles/`.
- `sim_sensor_profile` accepts exactly the three profile IDs and defaults to
  `clean`.
- `sim_sensor_seed` is a non-negative integer launch value and defaults to `6400`.
  The profile files do not contain a seed.
- Matrix YAMLs may set `sim_sensor_profile` and `sim_sensor_seed`. Existing
  matrices remain compatible and default to `clean`/`6400`.
- Expanded matrix repetitions use `sim_sensor_seed + repetition - 1`.
  Case and speed are deliberately not included so the same repetition remains
  comparable across scenarios.

## Exact profile values

The three YAMLs are the frozen values for this cut. Units are encoded in each
key; probabilities are in `[0, 1]`; `quality_transition_period_s: 0` means no
transition schedule for `clean`. The launch selector chooses one file and
passes it to the three matching nodes; the seed remains a separate parameter.

Freshness is intentionally represented by the existing delivery/dropout and
consumer timeout behavior. No unused `stale_after_s` parameter is defined.

## Evidence limits

This is deterministic configuration/wiring evidence only. It does not claim
that simulation sensor behavior is validated against the physical robot or
that degraded localization remains operational on hardware.
