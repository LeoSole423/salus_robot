# Issue #244 Track 3 — CURRENT route_executor evidence

Status: `INCONCLUSIVE`.

This report is evidence only. It does not propose a product change, alter Nav2
tuning, or authorize a PR.

## Scope and provenance

- Repository: `LeoSole423/salus_robot`
- Base: `main` at `f918d4bcfac33f6f8d20d5af02d175f6a1b9a82d`
- Evidence branch: `agent/issue-244-track3-reproducer`
- Evidence implementation SHA: `5077714f0169cd245c68e5730d3a45b574396d1a`
- Worktree: `/home/leosole/Desktop/SALUS-worktrees/salus_robot-issue-244-track3`
- No production source or tuning files were changed. No physical test was run.

The reproducer submits one geographic mission through the public productive
`/route_executor/set_route_mission_ll` service and observes the productive
route executor. It does not launch `NavigateThroughPoses` manually. The fixture
puts the first chunk boundary at 30 degrees of a 90-degree, 8 m-radius turn;
the planner contract remains the existing 4 m minimum-turning-radius profile.
Only CURRENT (`terminal_incoming`) was selected. No alternative policy or
heuristic was tested.

The existing A branch is already remote and was preserved:
`origin/agent/issue-244-chunk-continuity-subagent-a` at `91d132c`.
Its historical artifacts remain under
`/home/leosole/Desktop/SALUS-worktrees/artifacts-issue244-subagent-a-final/`.
The previous `SHARED_TANGENT` experiment (`91d132c`) is explicitly
`INVALIDATED FOR DESIGN DECISION`; none of its measurements are used here.

## Commands

```text
./tools/build.sh
docker compose run --rm ros2 bash -lc 'source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash && colcon test --packages-select salus_evaluation --event-handlers console_direct+ && colcon test-result --verbose'
./tools/nav_eval.sh isolation /home/leosole/Desktop/SALUS-worktrees/artifacts-issue244-track3-isolation
./tools/nav_eval.sh matrix src/salus_evaluation/config/matrices/issue244_chunk_continuity.yaml /home/leosole/Desktop/SALUS-worktrees/artifacts-issue244-track3-current-v2 --jobs 2
```

Build passed. Focused tests passed (`86 passed`, two warnings). Isolation passed
with independent ROS/Gazebo domains and simultaneous workers. The matrix was
CURRENT-only and ran with `--jobs 2`.

## Partial evidence

Artifacts:

- Isolation: `/home/leosole/Desktop/SALUS-worktrees/artifacts-issue244-track3-isolation/isolation-report.json`
- Matrix summary: `/home/leosole/Desktop/SALUS-worktrees/artifacts-issue244-track3-current-v2/summary/`
- rep01: `/home/leosole/Desktop/SALUS-worktrees/artifacts-issue244-track3-current-v2/trials/terminal_incoming-wide_turn_boundary-left-r8-v0p8-rep01/`
- rep02: `/home/leosole/Desktop/SALUS-worktrees/artifacts-issue244-track3-current-v2/trials/terminal_incoming-wide_turn_boundary-left-r8-v0p8-rep02/`

rep01 captured two productive `ROUTE_CHUNK_DISPATCHED` events, mission and
active-chunk paths, dispatch poses, yaws, checkpoint/goal activity in the
launch trace, and two `/plan` messages (A: 69 points, B: 205 points). The
partial geometry calculation from those `/plan` samples was:

| metric | rep01 partial value |
|---|---:|
| length A / B / total | 26.576 m / 78.773 m / 105.349 m |
| combined direct distance / detour ratio | 5.506 m / 19.135 |
| max deviation A / B / combined | 8.286 m / 8.426 m / 8.426 m |
| self-intersections plan A / B | 0 / 3 |
| crossings A/B | 5 |
| final heading A / initial heading B | 0.341 rad / -0.540 rad |
| final-A to initial-B distance | 2.693 m |
| actual `/plan` boundary angular discontinuity | 0.881 rad (50.455°) |
| dispatched yaw ranges A / B | 0.150–14.930° / 37.982–45.093° |

These values are retained as partial observations, not as a validated
reproduction result. The rep01 summary's old `REPRODUCED` label is not accepted
as the final conclusion because the observation protocol was not stable across
the matrix.

rep02 also reached productive chunk A and chunk B dispatch. It captured the
dispatches, robot poses, yaws, checkpoint events, and terminal chunk-A
`NavigateThroughPoses` success. It then failed the observation requirement:
`did not observe /plan for both route chunks`; the launch log shows the planner
aborting B with `GridBased: failed to create plan, no valid path found`.
Consequently the requested A/B/total plan metrics cannot be established for
both repetitions. The matrix summary reports one `passed` and one
`functional_failure`, with no aggregated continuity metrics.

## Conclusion and limitation

`NOT_REPRODUCED/INCONCLUSIVE`: the current productive route executor was
exercised and produced useful partial evidence, including one anomalous plan,
but the PC/sim observation path could not reliably provide `/plan` for both
chunks. This is an infrastructure/observation limitation, not evidence to
select a product policy or geometry fix. No isolation study, tangential
transition experiment, heuristic, production change, or PR follows from this
run.
