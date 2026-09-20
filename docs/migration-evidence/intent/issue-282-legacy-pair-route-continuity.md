# Intent evidence — #282 legacy-pair route continuity

## Scope

- Legacy source: `ROS2_SALUS/src/navegacion_gps` at its current operational
  checkout, plus the sanitized physical loop capture recorded in #244.
- New owner: `salus_navigation` route domain and its thin ROS adapter.
- Characterization base: `salus_robot@eba4017bbd45ac47f882ded4d541ad08b42acdfc`.
- Included: generic routes in PC/simulation, fixed two-checkpoint windows,
  ordered progress evidence and an opt-in A/B.
- Excluded: real deployment, adaptive three-point windows, Nav2 tuning,
  steering/radius changes, #272/#276/#277 and retirement of legacy.

## Evidence

| Source | What it demonstrates | Confidence |
| --- | --- | --- |
| `ROS2_SALUS/navegacion_gps/route_executor.py` | Legacy chunk construction, advancement and success-brake suppression | high |
| `ROS2_SALUS/.../navigate_through_poses_*` | `RemovePassedGoals radius=2.5` in the operational BT | high |
| `salus_navigation/route_chunker.py` | Current one-real-checkpoint terminal invariant | high |
| `salus_navigation/route_executor_node.py` | Current event accounting and missing odometry timestamp retention | high |
| Physical capture summarized in #244 | Legacy two-pose chunks and smoother observed operation | medium-high |
| Future PC/sim A/B | Whether the recovered contract improves the new stack without semantic regression | pending |

## Scope and decision

This cut specifies the first PC/simulation experiment for issue #282.  It does
not enable a new real-robot behavior.  The candidate recovers the physically
validated legacy dispatch window for ordinary routes:

```text
[Pi (intermediate), Pi+1 (terminal)] -> NavigateThroughPoses
```

The current one-real-checkpoint mode remains the default and the fallback.
Adaptive windows, three-checkpoint windows and path-admission against a
costmap are explicitly deferred.

## Historical facts

### ROS2_SALUS

- `build_chunk_waypoints()` normally included the current point and the next
  point, and `next_chunk_start_index()` advanced after the terminal point.
- A two-point chunk therefore behaved as `[P0,P1]`, followed by `[P2,P3]`.
- The NavigateThroughPoses behavior tree used `RemovePassedGoals` with a
  `2.5 m` radius and retained the last goal.
- The terminal Nav2 goal checker used `1.2 m` XY tolerance.
- Success braking was suppressed while an open route had remaining geometry
  and for loop routes.
- Legacy emitted `ROUTE_CHECKPOINT_REACHED` for the terminal chunk target.  It
  did not independently prove passage through every intermediate point.

### salus_robot at the base SHA

- `route_chunker.build_chunk()` ends every finite request at the first real
  (`key=True`) checkpoint.
- With production spacing `35/120/5` and no synthetic points this produces one
  `NavigateToPose` per original checkpoint.
- `_complete_current_chunk()` currently emits every key in a chunk from one
  terminal Nav2 result.  Merely permitting two keys would therefore fabricate
  an intermediate success.
- `_on_pose()` currently stores only XYZ from `/odometry/global`; it discards
  the source timestamp and cannot prove freshness.
- The yaw policy already validated by #244 remains: approach heading for the
  first automatic pose, incoming heading for the automatic terminal, and no
  modification of explicit yaws.

## Contract for the experimental mode

### Mode and compatibility

- Add a validated route-executor mode with values `single_checkpoint` and
  `legacy_pair`.
- Default is `single_checkpoint` in generic, simulation and real launches.
- The A/B harness explicitly selects a mode.  This cut cannot alter the real
  runtime default.
- Reuse `SetRouteMissionLL`, `SetNavGoalLL`, NavigateThroughPoses, the existing
  BT and the existing command/safety chain.  No public ROS interface or command
  authority is added.

### Hard and soft checkpoints

- A `normal` original checkpoint with no action and no explicit yaw may be an
  intermediate soft checkpoint.
- A checkpoint is hard when it has an action, has an explicit yaw while it
  would otherwise be intermediate, has any role other than `normal`, or is the
  terminal point of the finite request.
- The final point of an open route is terminal and hard.
- Patrol, HOME and patrol-exit submissions must label their checkpoints hard
  before the mode may be used outside the generic route A/B.  The first
  product cut does not soften their current semantics.
- Synthetic points remain geometric horizon only.  They never generate
  checkpoint success or actions.

### Chunk construction

- In `legacy_pair`, build disjoint ordinary windows: `[P0,P1]`, then `[P2,P3]`.
- A hard checkpoint encountered after a soft point may be the terminal second
  point; a chunk beginning at a hard checkpoint is a singleton.
- Synthetic points between two originals remain in the request, but at most
  two real checkpoint occurrences may exist in a chunk.
- Never dispatch a full loop cycle in one request.
- Loop closure may produce `[Plast,P0]`.  Each checkpoint occurrence therefore
  carries its own `(input_index, loop_iteration)` identity: `Plast` belongs to
  iteration N and `P0` to N+1.  Chunk-level iteration alone is insufficient.
- `next_start()` advances after the terminal occurrence without repeating or
  omitting an original checkpoint.

### Intermediate acceptance

- The soft checkpoint radius is exactly `2.5 m`, matching the legacy
  RemovePassedGoals radius.  The terminal remains governed by the existing
  `1.2 m` Nav2 tolerance; neither value is retuned in this experiment.
- Accept only the next expected soft occurrence and at most one occurrence per
  odometry sample.  Overlapping waypoint radii cannot acknowledge two points
  from one pose.
- Evidence must be `/odometry/global` with finite XY, a positive source stamp,
  monotonically non-regressing source timestamps and a finite reception time.
- The ROS-stamp age and steady-clock reception age must both be no greater than
  the new `route_progress_pose_max_age_s` parameter.  Its experimental default
  is `0.5 s`, type double, valid range `(0, 2.0]`.  A future-dated source stamp
  beyond `0.1 s` is invalid.  These checks are route progress checks, not EKF
  or Nav2 timeout changes.
- Entering the 2.5 m radius is sufficient for legacy parity.  Do not add a
  heading, corridor or perpendicular-plane requirement in the first A/B.
- Store minimum observed distance for every expected checkpoint occurrence.
- Emit `ROUTE_CHECKPOINT_REACHED` once per
  `(mission_id, loop_iteration, input_index)`, with completion source
  `fresh_odometry_radius` for a soft point and `nav2_succeeded` for the
  terminal.

### Terminal result and failure behavior

- A current, generation-correlated Nav2 success may complete only the terminal
  checkpoint occurrence.
- Before accepting terminal success, every preceding soft occurrence in the
  same request must already be acknowledged.
- If Nav2 succeeds while a soft occurrence is pending, pause through the
  existing cancel/brake path and emit `ROUTE_CHECKPOINT_SEQUENCE_INCOMPLETE`.
  Never backfill or infer the missing event from RemovePassedGoals or the
  planned path.
- Stale Nav2 results remain rejected by the existing epoch/event-floor logic.
- Retry of the same chunk preserves already acknowledged occurrences and their
  minimum distances.  A newly built chunk, replacement mission, cancellation
  or terminal mission state resets the chunk tracker.  Deduplication persists
  for the active mission and loop occurrence.
- Manual takeover, explicit cancel, collision safety and recovery authority
  remain unchanged.

## Evaluation contract

Use the sanitized physical legacy route fixture.  Compare paired runs with the
same route, XY/yaws, spawn, world, maps, Nav2 configuration and seeds:

- A: `single_checkpoint`.
- B: `legacy_pair`.

Required scenarios are an open route of at least three checkpoints, one full
broad loop plus causal entry into the second iteration, a controlled overshoot,
an action checkpoint, and patrol/HOME proving hard behavior.  Capture exact
requests, action type, initial plans and replans correlated to the goal,
executed odometry, minimum waypoint distances, checkpoint events, steering,
saturation, positive command/brake intervals and terminal results.

B passes only if it improves executed continuity without missed/duplicate
checkpoint occurrences, premature actions, weaker safety, or regressions in
cancel/retry/recovery/manual takeover.  `/plan` detour alone cannot select a
winner.

## Required tests before simulation

- Pairing for even/odd open routes and loops, including `[Plast,P0]` identity.
- Synthetic geometry, hard boundaries, explicit yaw and actions.
- Exactly-once ordered soft acceptance at 2.5 m.
- Rejection of stale, future, non-monotonic and non-finite odometry.
- Overlapping radii acknowledge at most the expected occurrence.
- Terminal success with a pending intermediate fails closed.
- Retry/cancel/replacement mission and loop deduplication.
- `single_checkpoint` remains byte-for-byte equivalent at request/event level.
- Existing yaw behavior is unchanged for single- and multi-pose requests.
- Patrol/HOME remain hard and actions execute once at their own checkpoint.

## Stop point

The implementation PR remains Draft and opt-in.  Classification is
`LEGACY_PAIR_PC_SIM_PASS`, `NO_SAFE_IMPROVEMENT` or `INCONCLUSIVE`.  No Jetson,
real default change or physical movement is authorized by this evidence.

Evidence state: `characterized`.  Hardware parity remains unvalidated for the
new implementation.
