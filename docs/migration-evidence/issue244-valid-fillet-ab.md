# Issue 244: evaluation-only CURRENT vs VALID_FILLET_R4

## Scope

This experiment is evaluation-only on branch `agent/issue-244-valid-fillet-ab`,
based on `ff5b7ab5687f9e3c57586a7ed1ca716996b974fd`. It does not alter the
route executor, Nav2 configuration, planner parameters, steering limits, or
the real profile. The `valid_fillet_r4` arm sends an evaluation-only
`NavigateThroughPoses` request; the `current` arm retains the existing
evaluation path.

The experiment stopped after the first `SINGLE_CORNER_90` gate. The
`BOUNDARY_CORNER_90` and `TRACK3_WIDE_R8` arms were not run because the fillet
arm did not improve the first scenario and produced pathological plans.

## Fixture and validity

The fixture uses a left 90-degree corner with local points
`P0=(0,0)`, `V=(8,0)`, and `P2=(8,8)`, with `R=4 m`. The pure
`valid_fillet_r4()` helper derives tangent entry `(4,0)`, tangent exit
`(8,4)`, and a sampled quarter-circle between them. The helper reports a
valid geometry; the results below evaluate the actual Nav2 plans produced from
that geometry, not the helper in isolation.

Both arms used `clean`, seed base `6400`, speed `0.8 m/s`, five repetitions,
and the same world/costmap. The matrix was run serially after a passing
`nav_eval.sh isolation` check. No physical hardware was used.

## Results

| Metric | CURRENT | VALID_FILLET_R4 |
|---|---:|---:|
| repetitions completed | 5 | 5 |
| setup failures | 0 | 0 |
| evaluator passed | 3/5 | 0/5 |
| evaluator functional failures | 2/5 (arrival gate) | 5/5 |
| Nav2 terminal success | 5/5 | 0/5 |
| individual plans measured | 9 | 21 |
| plan length median (m) | 11.681 | 279.683 |
| detour ratio median | 1.065 | 21.168 |
| max deviation median (m) | 1.431 | 8.855 |
| self-intersections median (count) | 0 | 94 |
| total heading variation median (rad) | 1.474 | 63.607 |
| max absolute curvature median (1/m) | 0.389 | 1.285 |
| curvature sign changes median | 0 | 18 |

CURRENT produced zero self-intersections in all nine individual plans and
completed the Nav2 action in all five repetitions. Its two evaluator failures
were terminal-arrival gate results while Nav2 still reported success; they are
retained as functional outcomes and were not relabeled as setup failures.

VALID_FILLET_R4 produced 79--103 self-intersections per individual plan,
252.880--301.151 m plan lengths, detour ratios 16.802--28.443, and 14--23
curvature sign changes. All five repetitions failed functionally and none
reached a successful Nav2 terminal result. This is a repeatable regression in
the evaluated arm, not an improvement hidden by an arrival threshold.

## Decision

`VALID_FILLET_R4`: **NO_MATERIAL_IMPROVEMENT**. It fails the first scenario
gate because it introduces severe topology and curvature pathologies in every
repetition. The stop point therefore applies: do not continue to boundary or
wide-track experiments and do not promote this geometry to production.

## Artifacts

External artifact roots (preserved outside the repository):

- `/home/leosole/Desktop/SALUS-artifacts/issue244-valid-fillet-isolation`
- `/home/leosole/Desktop/SALUS-artifacts/issue244-valid-fillet-current`
- `/home/leosole/Desktop/SALUS-artifacts/issue244-valid-fillet-fillet`

Each matrix root contains per-trial bundles and
`summary/matrix-summary.json`; the isolation root contains
`isolation-report.json`.

Commands used for the matrices:

```bash
SALUS_NAV_GEOMETRY_VARIANT=current ./tools/nav_eval.sh matrix \
  src/salus_evaluation/config/matrices/issue244_valid_fillet_single_corner.yaml \
  /home/leosole/Desktop/SALUS-artifacts/issue244-valid-fillet-current --jobs 1

SALUS_NAV_GEOMETRY_VARIANT=valid_fillet_r4 ./tools/nav_eval.sh matrix \
  src/salus_evaluation/config/matrices/issue244_valid_fillet_single_corner.yaml \
  /home/leosole/Desktop/SALUS-artifacts/issue244-valid-fillet-fillet --jobs 1
```
