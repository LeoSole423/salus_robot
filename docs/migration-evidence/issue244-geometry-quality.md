# Issue 244: geometry-quality benchmark

This is an evaluation-only cut. It does not change `route_executor`, Nav2
configuration, controller behavior, safety, or hardware deployment.

## Provenance and commands

- branch: `agent/issue-244-geometry-quality`
- base: `agent/issue-244-track3-reproducer@c73ac628333aa709a31e2ad0247fd03724983924`
- Smac packages: `ros-humble-navigation2` and `ros-humble-nav2-smac-planner`,
  both version `1.1.20-1jammy` (2026-08-04 builds)
- installed Smac headers expose `getClosestAngularBin()` and
  `getAngleFromBin()`, with no `getAngle()` equivalent from navigation2#4549:
  `SMAC_4549_ABSENT`
- isolation: `PASS` on the final run, including distinct domains/partitions,
  monotonic clocks, active Nav2, sibling death isolation, and cleanup
- isolation artifact:
  `artifacts-issue244-geometry-quality-isolation-rerun`

The existing commands were used for the campaign:

```bash
./tools/nav_eval.sh isolation <output-dir>
./tools/nav_eval.sh matrix <matrix.yaml> <output-dir> --jobs 1
./tools/nav_eval.sh matrix <matrix.yaml> <output-dir> --jobs 2
```

The frozen direct `ComputePathThroughPoses` replay remained blocked because the
integration graph does not expose a planner action server for that direct
endpoint (`ros2 action info /compute_path_through_poses` showed zero servers).
The action sanity call was therefore not accepted. No Nav2 rebuild or sandbox
was introduced: `REPLAY_BLOCKED` under the stop rule.

## Metrics

`salus_evaluation.geometry_quality` resamples each individual plan at 0.25 m
and reports total heading variation, curvature sign changes with a 0.02 1/m
deadband, max/P95 absolute curvature, equivalent Ackermann steering using a
0.94 m wheelbase, and signed lateral error RMS/max/sign changes against the
nominal reference polyline. Existing length, direct distance, detour,
max-deviation, and self-intersection metrics are retained separately for each
plan. Independent plan A/B crossings are reported separately by the chunk
runner and are never counted as self-intersections of a concatenated path.

`valid_fillet_r4()` is a pure evaluation helper only. It derives tangent entry
and exit points, circle center, deterministic arc samples, and tangential
headings for `P0 -> V -> P2`; it rejects degenerate, U-turn, and too-short-leg
geometry. It is not connected to production route preparation or dispatch.

## Campaign results

All static trials used `clean`, speed 0.8 m/s, and fresh isolated simulation
instances. Functional outcomes are retained even when the matrix exits nonzero.

| case | repetitions | valid bundles | plan observations | representative result |
|---|---:|---:|---:|---|
| STRAIGHT | 3 | 3/3 | 5 | mostly clean; one plan had 0.327 rad total heading variation and 0.252 1/m max curvature |
| SINGLE_CORNER_90 | 3 | 3/3 | 4 | valid but wide: about 6.38 rad total heading variation and 0.389 1/m max curvature |
| BOUNDARY_CORNER_90 | 3 | 1/3 | 4 in valid run | plan A straight; plan B length 30.89 m, detour 3.58, max deviation 8.50 m, self-intersections 0 in the valid run; another captured plan B had SI=1 before the reproducer gate timed out |
| TRACK3_WIDE_R8 | 3 | 1/3 | 3 in valid run | plan B length 78.77 m, detour 20.82, max deviation 8.42 m, self-intersections 3 |

Artifacts:

- static serial matrix:
  `artifacts-issue244-geometry-quality-static-serial`
- route matrix with two workers:
  `artifacts-issue244-geometry-quality-boundary-jobs2-causal`

The two route cases also had non-valid observation attempts where the executor
did not dispatch chunk B or no plan arrived after its dispatch in the bounded
observation window. Those attempts remain in the bundles and are not silently
converted to navigation success.

## Classification and recommendation

- `TOPOLOGICAL_LOOP`: **reproduced** in the individual TRACK3 plan B (3
  self-intersections); this is not an artifact of concatenating independent
  plans.
- `VALID_BUT_WIDE_TURN`: **observed** in SINGLE_CORNER_90 and route-level
  boundary/Track3 plans, with large detour/max-deviation values.
- `WOBBLY`: **intermittently observed** in STRAIGHT, where the current Smac
  binary lacks the #4549 interface/fingerprint. This is evidence for a later
  isolated Smac A/B, not for blaming route chunks or changing production yaw.

The next recommended cut is a narrowly scoped evaluation A/B of the exact
navigation2#4549 fix/overlay against the installed Smac binary, beginning with
the STRAIGHT case. Do not backport it to production from this report alone.
The boundary/fillet hypothesis remains evaluation evidence only; the logical
checkpoint/action semantics and the dependency on #57/#63 must be resolved
before any production driving-geometry change.
