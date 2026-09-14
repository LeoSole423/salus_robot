# Issue 244: matched HARD_VERTEX_CURRENT vs SPARSE_FILLET_R4

## Scope

This is an evaluation-only experiment on branch
`agent/issue-244-sparse-fillet-ab`, based on
`5e8bd3b6069cc4cf1f5f7eebdb0d4bf2c745db9e`. It does not alter the route
executor, Nav2 configuration, planner parameters, steering limits, or the
real profile. Both arms use the same `NavigateThroughPoses` action and exactly
three poses. No arc samples were dispatched to Nav2.

The experiment stopped after `SINGLE_CORNER_90`; `BOUNDARY_CORNER_90` and
`TRACK3_WIDE_R8` were not run.

## Matched fixture

The common mission is `P0=(0,0) -> V=(8,0) -> P2=(8,8)`, with start yaw
`0°`, final yaw `90°`, and `R=4 m`. The first pose is the common tangent
entry `(4,0,0°)`.

| Arm | Dispatched through-poses |
|---|---|
| `HARD_VERTEX_CURRENT` | `(4,0,0°)`, `(8,0,0°)`, `(8,8,90°)` |
| `SPARSE_FILLET_R4` | `(4,0,0°)`, `(8,4,90°)`, `(8,8,90°)` |

`SPARSE_FILLET_R4` uses `valid_fillet_r4()` only to derive the tangent exit
and the arm-reference geometry. Its internal arc samples are retained for
metrics, never sent as through-poses. The matrix used the `free` world,
`clean`, seed base `6400` (seeds 6400–6404), 0.8 m/s, and the same stock
Smac/DUBIN/costmap setup for both arms.

## Results

| Metric | HARD_VERTEX_CURRENT | SPARSE_FILLET_R4 |
|---|---:|---:|
| repetitions completed | 5/5 | 5/5 |
| setup failures | 0 | 0 |
| Nav2 terminal success | 5/5 | 5/5 |
| evaluator PASS | 0/5 | 0/5 |
| individual plans | 5 | 5 |
| common-reference length median (m) | 39.087 | 13.785 |
| common-reference detour ratio median | 3.564 | 1.257 |
| common-reference max deviation median (m) | 7.876 | 1.047 |
| common-reference self-intersections median | 1 | 0 |
| arm-reference max deviation median (m) | 7.876 | 0.202 |
| arm-reference self-intersections median | 1 | 0 |
| common-reference heading variation median (rad) | 5.952 | 1.789 |
| curvature sign changes median | 6 | 2 |

The hard-vertex arm produced a repeatable long looping path in all five
repetitions: 39.087 m median, detour 3.564, and one self-intersection per
plan. The sparse fillet arm produced 13.785 m median, detour 1.257, zero
self-intersections in every plan, and an arm-reference deviation of 0.202 m.
Both arms reached Nav2 terminal success in all repetitions; the evaluator
outcome remained `functional_failure` because the existing turn-sign gate
observed the simulation's sign convention as opposite to the scenario's
`left` expectation. That gate result is retained and was not hidden or
changed for this experiment.

## Decision

Classification: **SPARSE_FILLET_IMPROVES** for this frozen
`SINGLE_CORNER_90` case. Under a matched three-pose contract, replacing the
hard logical vertex constraint with the sparse tangent-exit constraint removes
the repeated self-intersection and substantially reduces detour and lateral
deviation. This is evidence for the next design step only; it does not yet
authorize a production change or establish checkpoint semantics for a
multi-chunk route.

The stop point applies: no boundary or Track3 experiment was run and no
production PR was opened.

## Artifacts and commands

External artifact roots, preserved outside the repository (the absolute host
locations are included in the issue comment, not in versioned documentation):

- `issue244-sparse-fillet-isolation`
- `issue244-sparse-fillet-current`
- `issue244-sparse-fillet-fillet`

Each matrix root contains five per-trial bundles and
`summary/matrix-summary.json`. The isolation root contains
`isolation-report.json`. Each trial summary includes the exact
`dispatched_poses` list and separate common/arm geometry metrics.

The two matrix commands were:

```bash
SALUS_NAV_GEOMETRY_VARIANT=hard_vertex_current ./tools/nav_eval.sh matrix \
  src/salus_evaluation/config/matrices/issue244_valid_fillet_single_corner.yaml \
  <artifact-dir>/issue244-sparse-fillet-current --jobs 1

SALUS_NAV_GEOMETRY_VARIANT=sparse_fillet_r4 ./tools/nav_eval.sh matrix \
  src/salus_evaluation/config/matrices/issue244_valid_fillet_single_corner.yaml \
  <artifact-dir>/issue244-sparse-fillet-fillet --jobs 1
```
