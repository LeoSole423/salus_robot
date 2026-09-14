# Issue 244: sparse fillet across a request boundary

## Scope and provenance

This is an evaluation-only cut based on
`e51764e4e17db07d8985eedd559650df9f07b69d`, on branch
`agent/issue-244-sparse-boundary-ab`. It changes only the evaluation runner,
tests, and matrix fixtures. It does not change `route_executor`, chunking,
route preparation, Nav2 parameters, steering, RPP, safety, or any real
configuration.

All trials used the `free` world, `clean` sensors, stock Humble Smac/DUBIN,
`minimum_turning_radius=4.0`, 0.8 m/s, serial execution, and seed base 6400.
The existing turn-sign gate was left untouched and is reported separately from
Nav2 terminal status and geometry.

The canonical local geometry is `P0=(0,0)`, logical vertex `V=(8,0)`,
`P2=(8,8)`, `R=4`: `E=(4,0,0°)`, `M=(6.8284271247,1.1715728753,45°)`,
`X=(8,4,90°)`, and `F=(8,8,90°)`. `V` is never dispatched by a sparse arm.
The midpoint is computed from `valid_fillet_r4()`; no dense arc samples are
sent to Nav2.

## Results

| Phase / arm | Trials | Nav2 terminal success | evaluator PASS | individual-plan self intersections | geometry result |
|---|---:|---:|---:|---:|---|
| F0 `SPARSE_SINGLE_3` | 3 | 3/3 | 2/3 | 0/3 | healthy |
| F1 `SPARSE_SINGLE_3` control | 5 | 5/5 | 0/5 | 0/5 | healthy |
| F1 `SPARSE_BOUNDARY_EXIT` | 5 | 5/5 | 1/5 | A/B 0/10 | healthy |
| F2 `SPARSE_SINGLE_4` | 3 | 3/3 | 1/3 | 1/3 | over-constrained |

The evaluator PASS column includes the unchanged, known turn-sign convention
gate; it is not used as a geometry veto. One setup failure in `SINGLE_4` was
rerun once because it was the known transient `FollowPath` runtime-parameter
rejection. The final three `SINGLE_4` trials were all valid and all showed the
same topological failure.

### Fase 1 boundary details

`SPARSE_SINGLE_3` dispatches one request `[E, X, F]`. The boundary arm
dispatches exactly `[E, X]`, waits for request A terminal success, then sends
`[F]` using the real post-A odometry as the action start. No logical vertex or
dense arc sample is present in either request.

Across the five valid boundary trials:

- request A: length 9.785 m, detour 1.151, max deviation 1.047 m, zero
  self-intersections in every plan;
- request B: median length 5.052 m and median detour 1.007, zero
  self-intersections in every plan;
- executable A+B length estimate median 14.837 m;
- actual final-A to initial-B heading discontinuity median 16.88°;
- terminal A to dispatch B was immediate (0.0 s in the recorded ROS stamps);
- cross-plan intersections were reported separately (0 or 1) and never counted
  as self-intersections;
- the robot pose at B dispatch, its `goal_generation`, and every plan's
  `request_index` are persisted in each trial bundle.

Classification: **`BOUNDARY_EXIT_EQUIVALENT`**. Splitting the valid sparse
constraints at tangent exit did not introduce a long arc or a topological
loop. The small variation in B length is the expected consequence of starting
the second request from the actual post-A odometry rather than inventing an
exact X pose.

### Fase 2 results

`SPARSE_SINGLE_4` dispatches `[E, M, X, F]` in one request. In all three valid
trials it produced approximately 39.234 m, detour 3.578, max deviation
8.223 m, and one self-intersection. Therefore the required classification is
**`MIDARC_POSE_OVERCONSTRAINS`**. The exploratory `BOUNDARY_MIDARC` bundles
are retained for audit only; they were not used to authorize a subsequent
phase and do not change the stop decision. They likewise showed a pathological
second plan, but cannot distinguish boundary from the already-failed
single-request midpoint case.

## Artifacts and validation

External artifact roots (all preserved outside the repository):

- `/home/leosole/Desktop/SALUS-artifacts/issue244-sparse-boundary-isolation`
- `/home/leosole/Desktop/SALUS-artifacts/issue244-sparse-boundary-final-single3`
- `/home/leosole/Desktop/SALUS-artifacts/issue244-sparse-boundary-final-exit`
- `/home/leosole/Desktop/SALUS-artifacts/issue244-sparse-single-4`
- `/home/leosole/Desktop/SALUS-artifacts/issue244-sparse-boundary-midarc`

Each completed trial contains exact request poses/yaws, request index,
generation, robot pose at dispatch, per-plan provenance, individual geometry
metrics, and the boundary transition record where applicable. The isolation
report passed domains/partitions, clocks, lifecycle, sibling death, and
cleanup.

Validation on the final branch:

- focused evaluation/provenance tests: 23 passed;
- full `./tools/build.sh`: passed, 14 packages;
- full `./tools/test.sh`: passed, 1071 tests, 0 errors, 0 failures, 2 skipped;
- `py_compile`: passed;
- `git diff --check`: passed.

Stop point reached after Fase 1/Fase 2. No TRACK3, production implementation,
Jetson, hardware, PR, or merge was performed.
