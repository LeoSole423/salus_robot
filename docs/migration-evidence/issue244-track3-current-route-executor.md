# Issue #244 Track 3 — CURRENT route-level provenance

Status: `INCONCLUSIVE`.

This is evaluation evidence only. It does not change production navigation,
Nav2 tuning, or yaw policy.

## Provenance and method

- Repository: `LeoSole423/salus_robot`
- Base: `main@f918d4bcfac33f6f8d20d5af02d175f6a1b9a82d`
- Evidence branch: `agent/issue-244-track3-reproducer`
- Final evidence tooling SHA: `f54c5c44d7d727a8ea03e5ef1b87e500fcac8b56`
- Policy: CURRENT / `terminal_incoming` only
- No Jetson, hardware, or production source/configuration was changed.

The runner submits one geographic mission through the productive
`/route_executor/set_route_mission_ll` service. It is not a pair of manually
launched Nav2 actions. Each `ROUTE_CHUNK_DISPATCHED` opens a causal window,
matched to `GOAL_ACCEPTED` and `GOAL_RESULT_*` by `goal_generation`. Every
`/plan` in that window is retained, with nearest `/odometry/global` samples at
dispatch, plan, and result timestamps. Metrics are computed per independent
plan; independent plans are never concatenated for self-intersection counts.

## Commands and validation

```text
./tools/build.sh
docker compose run --rm ros2 bash -lc 'source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash && colcon test --packages-select salus_evaluation --event-handlers console_direct+ && colcon test-result --verbose'
./tools/nav_eval.sh isolation <artifact-dir>/artifacts-issue244-track3-provenance-isolation
./tools/nav_eval.sh matrix src/salus_evaluation/config/matrices/issue244_chunk_continuity.yaml <artifact-dir>/artifacts-issue244-track3-current-provenance --jobs 1
```

Build passed. `salus_evaluation` passed with 88 tests. Isolation passed with
independent ROS/Gazebo domains and clean worker teardown. The CURRENT matrix
was serial and used five repetitions; no alternative policy was run.

## CURRENT repetitions

| repetition | chunk A plans / self-intersections | chunk B plans / self-intersections | observation |
|---|---|---|---|
| 1 | 1 / 0 | 3 / 4, 5, 3 | pathological B plan observed |
| 2 | 2 / 0, 0 | 0 / — | B plan not observed before result |
| 3 | 1 / 0 | 2 / 4, 5 | pathological B plan observed |
| 4 | 1 / 1 | 2 / 4, 3 | pathological B plan observed |
| 5 | 2 / 0, 0 | 0 / — | B plan not observed before result |

The clearest observation is repetition 1, chunk `1`, first `/plan`: length
`82.3231 m`, direct distance `5.3151 m`, detour ratio `15.4886`, max deviation
`8.3523 m`, and `4` self-intersections. Later replans in that same causal
window remained pathological. The corresponding window was `goal_generation=2`;
its dispatch contained the exact five `poses_xy` and `yaws_deg`, and the
nearest odometry at the first plan was
`x=5.1346463`, `y=0.6434666`, `yaw=0.2235617 rad`.

The first plan of that repetition (chunk `0`) was `33.7889 m` with zero
self-intersections. A→B crossing count is reported separately (`7` for the
transition) and is not included in either plan's self-intersection metric.
The transition summary for this observation reports final-A/initial-B heading
change `0.09956 rad` and endpoint distance `1.25 m`; this does not establish a
boundary-yaw cause by itself.

## Frozen direct planner replay

The frozen input was replayed three times in a fresh, single simulation using
`ComputePathThroughPoses`, with `use_start=true`, `planner_id=GridBased`, the
same `free.world`, `nav2_core_no_obstacles_sim.yaml`, DUBIN model, radius `4.0 m`,
and the exact start/goals/yaws above. The action server was available, but all
three requests were rejected before returning a path. Therefore the frozen
input did not reproduce the pathological path, nor did it produce a normal
path that can be compared. Classification: `INCONCLUSIVE`; the missing signal
is an accepted/resulting path for the frozen action input in the same planner
context. No production conclusion follows.

Artifacts:

- isolation: `artifacts-issue244-track3-provenance-isolation/isolation-report.json`
- CURRENT matrix: `artifacts-issue244-track3-current-provenance/summary/`
- frozen replay: `artifacts-issue244-track3-planner-replay/replay.json`

The historical SHARED_TANGENT experiment remains retained but is
`INVALIDATED FOR DESIGN DECISION` because its corner/bisector geometry and
concatenated-plan metric were methodologically invalid.

## Conclusion

`INCONCLUSIVE`: CURRENT route-level execution repeatedly observed a long,
self-intersecting individual plan B, but two of five runs did not expose a B
plan in the observation window and the direct frozen planner replay was
rejected without a path. This supports a route-level observation worth a
follow-up, but does not identify a deterministic planner-only cause. No fix is
recommended from this cut.
