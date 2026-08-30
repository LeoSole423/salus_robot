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
- Full deterministic teleport/rolling-window simulation and hardware evidence
  still require execution in the Humble simulation environment; no hardware
  launch was run by this migration.
