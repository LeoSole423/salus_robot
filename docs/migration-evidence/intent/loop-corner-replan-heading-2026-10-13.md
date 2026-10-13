# First intermediate heading in the second real loop replan

## Intent and scope

Reduce the second loop incident from issue #307 without weakening obstacle
safety or altering Smac tuning. Owner: `salus_navigation.route_preparation`;
the existing route adapter still sends one finite Nav2 request. Included:
first automatic intermediate heading selection, a translated captured fixture,
unit regressions and a native planner-only replay tool. No Jetson deployment,
physical motion, public interfaces, terminal success or checkpoint semantics.

## Evidence and confidence

| Source | Observation | Confidence |
|---|---|---|
| `salus_nav_loops_20260924T235727Z/bag`, `a0940cb`, original DB SHA256 recorded in analysis inventory | At 23:58:42.651 UTC, chunk `[14,15]` requests headings −11.878° and −90.346°. Initial plan is ~50.6 m. At 23:58:46.028 clearance degrades; replan first leg is 36.248 m to a target ~10.2 m away. Checkpoint 14 is acknowledged only at 52.442. | High for messages and ordering |
| Recorded mission geometry around 14 | Incoming direction −55.388°, outgoing −90.673°; circular bisector −73.030°. | High for geometry, not terrain labels |
| Production `dispatch_yaws` before this change | Current robot heading −11.878° replaces the bisector because their difference is 61.152° (>60°). That pre-turn heading remains in Nav2's intermediate goal during later replans. | High, code and failing regression |
| Installed Humble Smac 1.1.20, empty costmap, same normalized poses | Captured heading produces first replan leg 35.812 m; corrected heading produces 10.276 m. Initial corrected approach remains short, 14.245 m. | High for planner reproduction; not hardware |
| Recorded OccupancyGrid, inverted Nav2 cost translation at lower and upper bounds | Both reconstructions produce 36.266 m before and 10.423 m after. Recorded raw internal 0–255 costs are unavailable, so these grids are lossy bounds, not exact runtime replay. | High for experiment; bounded reconstruction confidence |

The empty-map counterfactual demonstrates that the captured heading alone is
sufficient to produce this detour. Occupancy may have triggered replanning;
this finding does not identify physical obstacles, grass or ground ghosts.
The two costmap reconstructions corroborate the mechanism but do not prove
which exact costmap or TF sample the production planner used internally.

## Change and preserved behavior

For a first automatic checkpoint that has a following Nav2 pose, compare its
route tangent against the geometric bearing robot→checkpoint instead of the
current robot heading, provided that bearing is within the existing 60°
sharp-join threshold of the robot heading. A more lateral approach retains
the previous reference. Only a tangent difference exceeding that threshold
substitutes that tangent with the approach bearing. In the captured case,
bearing is ~−51.18°, so the tangent −73.03° is kept. Do not freeze a pre-turn
heading at a waypoint the vehicle will reach after entering a curve.

Singletons retain their previous heading reference. Explicit yaws, synthetic
first points, following points, legacy policy, actions and checkpoint order are
unchanged. Missing/invalid/zero-length approach geometry retains the existing
heading fallback. No new parameter, velocity, radius, collision timeout,
costmap threshold, goal tolerance or stop authority is introduced.

## Reproduction

The translated fixture is
`src/salus_evaluation/test/fixtures/loop_corner_replan.json`. Native tool:

```bash
python3 tools/test_loop_corner_planner.py --output-dir artifacts/loop-corner-planner
```

Run after build/source in an isolated Humble container (`--network none`).
Optional `--costmap-dir` adds the four lower/upper map exports. The C++ harness
has no controller and calls only the installed planner on fixed costmaps.
It processes each goal sequentially, using the previous segment's final pose
as the next start. The final goal heading is held at the captured −90.346° to
isolate the first-goal change. Smac parameters come from `nav2_core_real.yaml`,
which is unchanged from the captured commit. Unspecified planner defaults come
from installed Nav2 1.1.20, not an inferred Jetson parameter dump.

The costmap fixture preserves recorded resolution, dimensions, grid alignment,
footprint and padding. A costmap with no inflation plugin uses full-footprint
collision checking rather than the inflation shortcut. Exported maps already
contain inflation costs; no second inflation pass is added. No costmap updates
occurred between the selected full snapshots and the experiment anchors.

Checks require the captured replan to exceed 3× direct distance and the current
replan to remain below 1.5×, in empty and optionally reconstructed maps. A sharp
join control preserves the original six-meter entry at heading 0°, rather than
blindly imposing the −90° corner tangent. Inputs, all output path points and
summary metrics are retained in the output directory. This is planner evidence;
it does not establish executed curvature, steering saturation or stopping.

## Verification and remaining work

- Regression failed before the fix: requested −11.878° rather than −73.030°.
- Route domain tests: 68 passed; package tests: 306 passed, no failures.
- Native planner comparison: 13 cases passed, including initial/replan,
  empty/lower/upper maps and the existing sharp-join control.
- Build: 14 packages passed.
- Full suite: 1244 tests, no errors/failures, 2 skips. The 36 repository/runtime
  Python checks and smoke harness self-test also passed.
- Loop motion smoke with automatic yaws and `legacy_pair`: passed,
  `routes-rulo-loop-20261013T135255-1`. It observes the first lap and the causal
  dispatch of iteration 1, clean cancellation, the unchanged wide-turn
  topology comparison and navigation profile contracts. This smoke is a
  broad synthetic loop, not a full physical-route replay.
- Patrol/HOME battery smoke: failed before goal dispatch, with
  `loop route preparation rejected: anchor selection no_near_segment`,
  `patrol-rulo-heading-20261013T135507-1`. A comparison with the original
  `main` heading policy fails the same phase after 70 s with that rejection,
  `patrol-rulo-baseline-20261013T135823-1`. The four-point patrol
  fixture starts six meters from HOME; current loop admission requires a
  nearby finite segment (five meters by default). This reaches the #315
  admission boundary before `dispatch_yaws`. Patrol/HOME acceptance is not
  claimed by this patch; the change stays Draft pending review of that
  separate join/admission problem.
- Standalone flake8 reports the same 16 pre-existing style violations as
  `main`, with no additional violations in the modified policy. This is
  recorded separately from the passing colcon suite.

The initial broader approach-bearing change failed the existing wide-turn
topology gate: a lateral northward first leg while facing east produced an
extra self-intersection. The final rule retains the old reference for sharp
lateral joins and has a regression for that case. No smoke gate was weakened.

Evidence classification: characterized with offline planner validation.
Hardware validation: not performed. The original bag predates #318 and #321;
the heading correction addresses a distinct cause. Issue #307 stays open for
physical obstacle-stop acceptance and remaining incident classifications.

The corrected reconstructed-map plan totals 44.270 m rather than 71.888 m.
Only the first intermediate heading changed in this A/B. Reducing the loop
does not establish a universal no-detour policy or justify bypassing an
obstacle. Local artifact paths and sensitive map coordinates are not included
in this portable evidence; the fixture is translated to waypoint 14 at (0,0).


## CI follow-up on PR #323

Run `38409913941` failed the real navigation fixture's tilted-return marking
check and the canonical navigation smoke's right-turn plan wait. The latter
log contains goal preemption. Nav2 Humble's NavigateToPose navigator creates
its own `goal_pose` subscriber and sends that pose to its own action server;
SALUS also received the public topic and sent an action. A single public
publication therefore had two competing goal submission paths.

Both core launches now remap only bt_navigator's direct RViz input to
`/nav2/internal_goal_pose`. SALUS publishes nothing there. Public `/goal_pose`
continues through the existing arbiter, including keepout/manual checks,
without changing action names or the command chain. The real PC fixture and
canonical smoke require exactly `nav_command_server` in the public topic's
subscription graph. This is an endpoint regression gate rather than a source
text assertion. Upstream reference:
https://github.com/ros-navigation/navigation2/blob/humble/nav2_bt_navigator/src/navigators/navigate_to_pose.cpp

The costmap fixture now visits only the measured observation rectangle.
A full-grid oracle checks identical cell counts at positive/negative sides
and clipped grid boundaries; lethal counts, clearing percentages and timeouts
are unchanged. Measured local evaluation is about 0.28 ms instead of 0.91 ms
on a 300x300 grid. Failure output also includes input cycles, marked-cell count,
scan discovery and launch logs. This optimization removes unnecessary observer
work but does not by itself prove the cause of the original CI marking failure.

Local validation after these changes: full build and suite in a 4-CPU isolated
container passed (1245 tests, 2 skips); the canonical navigation smoke passed
in a 2-CPU isolated container and verified the single public goal subscriber.
An additional 2-CPU full-suite stress run had an unrelated control comparison
failure; it is retained in artifacts and not counted as a passing suite.
The final GitHub run remains the acceptance gate; no Jetson deployment.
