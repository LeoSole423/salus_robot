# #244 exact productive replay

Evaluation-only causal cut from
`agent/issue-244-track3-sparse-causal@169557ca7505aefa65b565170eb0914206ac43d7`.
No route, Nav2, controller, safety or real-profile changes were made.

## R0 — request/state comparison

Source: T0 pathological trial
`terminal_incoming-wide_turn_boundary-left-r8-v0p8-rep02`.

The T0 productive chunk B dispatched five poses, with input indices
`[1, 1, 1, 1, 2]` and yaws `[38.3473888, 45.0933080, 45.0933080,
45.0933080, 45.0933080]` degrees. T1 CURRENT dispatched only two poses:
`S1` and `P2`. Therefore the requests are not numerically equivalent.

The T0 chunk-B dispatch state was approximately
`(5.274779, 0.703354, 0.244267 rad)` from `/odometry/global` at stamp 7.602 s.
The T1 CURRENT example was approximately
`(4.265286, 0.829558, 0.241950 rad)`. The state is not equivalent either.

R0 classification: `NOT_EQUIVALENT`; R1 was required.

## R1 — exact productive request replay

The replay fixture contains the exact T0 request A (six poses) and request B
(five poses), including the captured XY/yaw values. The harness sends A and B
sequentially through `NavigateThroughPoses`; B is sent only after A succeeds.
The simulator uses the normal spawn because the existing EKF startup does not
preserve a non-zero Gazebo spawn yaw in the map odometry frame. The post-A
odometry at B dispatch is persisted for every trial, and the T0 B state delta
is reported rather than hidden.

| repetition | B start delta to T0 (m) | B yaw delta (rad) | B plan length range (m) | B detour range | B self-X range |
|---:|---:|---:|---:|---:|---:|
| 1 | 0.279 | 0.021 | 78.617–82.160 | 13.673–19.286 | 3–5 |
| 2 | 0.275 | 0.043 | 76.998–83.655 | 14.000–19.286 | 3–7 |
| 3 | 0.275 | 0.026 | 78.503–83.655 | 14.273–20.740 | 5–7 |
| 4 | 0.296 | 0.029 | 76.998–83.655 | 14.000–19.286 | 3–7 |
| 5 | 0.246 | 0.014 | 78.617–82.160 | 13.673–19.647 | 3–5 |

All five trials accepted both sequential requests and returned terminal Nav2
success. Every trial produced an individual chunk-B plan with self-intersection
and detour in the same order as the T0 pathology. No concatenated-plan metric
was used.

R1 classification: `EXACT_PRODUCTIVE_REQUEST_REPRODUCES`.

## Artifacts

- Isolation: `SALUS-artifacts/issue244-exact-productive-replay-isolation-20260914-1`
- R1: `SALUS-artifacts/issue244-exact-productive-replay-r1-20260915-2`
- R1 source fixture: `src/salus_evaluation/config/replays/issue244_t0_rep02_chunk_b.json`

The R1 bundle persists exact requests, request index, goal generation, every
observed individual plan, terminal status, post-A odometry at B dispatch and
the replay provenance.

The result supports the conclusion that the exact productive request sequence
is sufficient to reproduce the pathological chunk-B geometry. It does not yet
identify which subset of the five B poses or which runtime context is the
minimal trigger; that is the next investigation, not part of this cut.
