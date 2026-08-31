# Intent evidence — vector keepout runtime migration (#146)

## Scope

- Legacy source: GeoJSON → fixed 3000 × 3000 PGM/YAML → `LoadMap` →
  `OccupancyGrid` / `KeepoutFilter`.
- New destination: GeoJSON → projected map-frame vector state → bounded
  `VectorKeepoutLayer` in rolling local and global costmaps.
- Included: transactional zones ownership, Nav2 configuration/startup, goal
  rejection, snapshot rendering, and removal of legacy map-server runtime.
- Outside scope: robot/hardware validation and localization redesign.

## Facts and inference

- The global costmap is a 300 m rolling `map` window and the local costmap is
  a 30 m rolling `odom` window; neither is a world map.
- #145 rasterizes only the requested costmap patch after bounding-box
  intersection. A zone at x=1050 therefore needs no 1 km raster.
- `ProjectedKeepoutState.revision` is process-local. Consumers accept a state
  whose revision differs from the last accepted state, including a lower value
  after a publisher restart.

## Commit semantics

1. Normalize and project a GeoJSON candidate.
2. Atomically persist the GeoJSON for operator updates.
3. Commit in-memory authority, increment revision, and publish reliable
   transient-local state.

A failed projection or persistence never changes the accepted document or
publishes a candidate. `GetZonesState.mask_ready` remains API-compatible and
means that the accepted vector state is available; `mask_source` reports
`projected_vector_state` rather than a legacy image source.

## Validation and limits

- Unit tests cover empty state, disabled zones, holes, long-range goal
  rejection, snapshot vector rendering, and removal of legacy dependencies.
- The costmap plugin's geometry tests cover core/halo and bounded patch work.
- `VectorKeepoutLayerPolicy.FarZonesDoNotDirtyRollingWindow` verifies that a
  polygon around x=1000 does not dirty a 30 m active window and that only the
  intersecting polygon reaches rasterization. The requested patch allocation is
  exactly `width × height` cells of that rolling update, independent of world
  extent; no global keepout image is allocated.
- `python3 tools/run_registered_smoke.py vector_keepout_long_range --context
  manual` performs the deterministic Humble/Fortress validation. It creates A,
  B and C once in `map`, then physically teleports the vehicle from the origin
  to about 350 m and 700 m. `ComputePathToPose(use_start=true)` uses explicit
  endpoints across each fixed polygon, so the detour assertion never depends
  on residual EKF heading or the observed vehicle pose.
- The probe asserts a 1200 × 1200, 0.25 m global window and a 300 × 300, 0.1 m
  local window at all three locations, records their origins, and requires no
  `/keepout_filter_mask` publisher. It samples both a core and halo cell for
  local map→odom correction, zone move and zone removal; clearing must restore
  the observed no-keepout baseline, not merely a non-lethal cost.
- The smoke records process-local revisions and observational service-to-state
  and state-to-costmap latencies, plus an informational Nav2 process RSS/CPU
  snapshot. These measurements are evidence, not performance gates.
- The 2026-08-31 manual run passed in 14.44 s. Its JSON evidence is preserved
  by the smoke harness under `artifacts/smokes/`; it demonstrated A/B/C goal
  rejection and detours, fixed map geometry, two rolling shifts, local
  map→odom clearing, B move/removal clearing, and zero legacy-mask publishers.
  No hardware launch was run by this migration.
