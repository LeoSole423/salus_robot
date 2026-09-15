# Issue #244 — request B delta-debug

Evaluation-only replay of `issue244_t0_rep02_chunk_b.json`, using the exact
captured request A in every arm.  Nav2 remained on the same free-world
configuration with DUBIN and `minimum_turning_radius=4.0 m`; no production
parameters were changed.

## Matrix

Each arm used five serial trials.  The values below are the per-trial extrema
over individual request-B plans (plans were never concatenated for topology).
All 20 trials had terminal status `4` (`SUCCEEDED`) and evaluator outcome
`passed`.

| Arm | Request-B poses | B plan count (range) | self-X (min/mean/max) | detour (min/mean/max) | length m (min/mean/max) | max deviation m (min/mean/max) | Classification |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | --- |
| D0 FULL5 | 5 | 2–3 | 2/3.8/7 | 10.994/14.416/19.647 | 57.091/67.763/83.655 | 8.321/8.432/8.507 | PATHOLOGY_REPRODUCES |
| D1 ENDPOINTS_ONLY | 2 (B0,B4) | 2 | 0/0/0 | 1.021/1.021/1.021 | 5.263/5.263/5.263 | 0.224/0.224/0.224 | PATHOLOGY_CLEARS |
| D2 NORMALIZE_B0_YAW | 5 | 2–3 | 3/4.2/5 | 11.367/18.007/20.406 | 58.587/77.446/82.160 | 8.321/8.454/8.507 | PATHOLOGY_REPRODUCES |
| D3 DROP_B0 | 4 (B1–B4) | 1–2 | 3/5.0/7 | 11.367/15.143/16.232 | 58.587/78.044/83.656 | 8.321/8.432/8.507 | PATHOLOGY_REPRODUCES |

D0 was reconfirmed with the updated harness because the harness changed, so
the baseline is not only the earlier R1 artifact.  The earlier exact replay
remains preserved separately.

## Causal conclusion

D1 is the only arm that clears in all five trials.  Removing the intermediate
poses B1–B3 while preserving B0/B4 eliminates the individual-plan
self-intersections and the large detour.  Changing only B0 yaw (D2) does not
clear the result, and removing B0 while retaining B1–B4 (D3) does not clear it.
The strongest conclusion supported by this cut is therefore that the
intermediate pose set B1–B3 is causally relevant as a group.  This does not yet
identify the smallest subset or justify a production change.

All dispatched request records, robot pose at dispatch, and exact pose/yaw
values are retained in the trial `summary.json` files.  Each D1–D3 arm also
has a `delta-debug-audit.json` with the structured diff against FULL5 derived
from those recorded requests; D0 was generated with the final structured-diff
field in the harness.  The artifacts are:

- `issue244-request-b-d0-full5-20260915-1`
- `issue244-request-b-d1-endpoints-20260915-1`
- `issue244-request-b-d2-normalize-20260915-1`
- `issue244-request-b-d3-drop-b0-20260915-1`

The three post-run audit files were derived only from recorded dispatched
requests and the frozen replay source.  The D1–D3 matrix metadata records the
parent source SHA because those runs used the uncommitted equivalent harness
before the final artifact-field commit; no request or planning behavior
changed in that documentation-only addition.  No Jetson, hardware, or
production runtime was used.
