# #244 TRACK3 sparse causal cut

Evaluation-only evidence for `agent/issue-244-track3-sparse-causal`. This cut
does not change `route_executor`, route preparation, Nav2 configuration, or any
real profile.

## Frozen contract

The route-level fixture is the existing `chunk_continuity_runner` TRACK3
fixture, with nominal mission radius 8 m:

| point | x (m) | y (m) | heading |
| --- | ---: | ---: | ---: |
| P0 | 1.500000 | 0.000000 | 15 deg |
| P1 | 5.500000 | 1.071797 | 45 deg |
| P2 | 8.428203 | 4.000000 | 45 deg |
| P3 | 9.500000 | 8.000000 | 45 deg |
| E1 | 4.464724 | 0.794395 | 15 deg |
| X1 | 6.257875 | 1.829672 | 45 deg |
| S1 | 6.207107 | 1.778904 | 45 deg |

`E1`/`X1` are derived with the existing `valid_fillet_r4()` helper. `S1` is
the first point from the productive 1 m leg expansion of P1 -> P2. The planner
contract remains `minimum_turning_radius=4.0 m`; R8 is only the nominal TRACK3
mission curve. No M or dense arc samples are dispatched.

## T0 route-level CURRENT

Matrix: `issue244_chunk_continuity.yaml`, serial, five attempts, existing
productive `route_executor` service. Three attempts produced a second-chunk
plan with the historical pathological geometry; two attempts failed to observe
the second plan and are retained as functional failures rather than rerun.

| attempt | result | plan B length (m) | plan B detour | plan B self-intersections |
| ---: | --- | ---: | ---: | ---: |
| 1 | valid | 82.079 | 16.00 | 4 |
| 2 | valid | 78.802 | 20.83 | 3 |
| 3 | valid | 82.323 | 15.49 | 4 |
| 4 | functional failure: no B plan | — | — | — |
| 5 | functional failure: no B plan | — | — | — |

Classification: `TRACK3_CURRENT_REPRODUCED`.

## T1 matched A/B

Both arms use the same evaluation runner, two sequential
`NavigateThroughPoses` requests, the same scenario, seeds, speed, world,
costmap and planner R4. Request B is sent only from the success callback of
request A and each bundle persists the post-A odometry at B dispatch.

| arm | valid trials | Nav2 terminal success | A length / detour / max dev / self-X (median) | B length / detour / max dev / self-X (median) | min distance to logical P1 (median, max) |
| --- | ---: | ---: | --- | --- | --- |
| `TRACK3_CURRENT_BOUNDARY` | 5/5 | 5/5 | 5.023 / 1.005 / 0.884 / 0 | 5.204 / 1.010 / 0.261 / 0 | 0.255 m, 0.281 m |
| `TRACK3_SPARSE_EXIT` | 5/5 | 5/5 | 31.427 / 5.229 / 8.268 / 1 | 29.078 / 7.145 / 7.970 / 0 | 0.032 m, 0.063 m |

All sparse trials passed within the unchanged 1.2 m semantic checkpoint
tolerance, but all five produced the same pathological plan-A topology. The
cross-plan intersections are reported separately (median 3 for sparse); they
are not included in individual-plan self-intersection counts.

Classification: `TRACK3_SPARSE_NO_IMPROVEMENT`. The sparse exit preserves P1
proximity but worsens the matched planner geometry, so it is not a production
candidate from this experiment.

## Artifacts

Artifacts are stored outside the repository under the host evidence root:

- `SALUS-artifacts/issue244-track3-sparse-causal-isolation-20260914-1`
- `SALUS-artifacts/issue244-track3-sparse-causal-t0-20260914-1`
- `SALUS-artifacts/issue244-track3-sparse-causal-t1-current-20260914-1`
- `SALUS-artifacts/issue244-track3-sparse-causal-t1-sparse-20260914-1`

The T1 bundles include exact request poses, `request_index`,
`goal_generation`, dispatch robot pose, terminal status, individual plan
metrics, geometry contract and `min_robot_distance_to_logical_P1_m`.

