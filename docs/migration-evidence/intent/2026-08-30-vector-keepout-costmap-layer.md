# Intent evidence — bounded vector keepout costmap layer (#145)

## Facts

- `zones_manager` publishes revisioned enabled projected polygons on
  `/zones_manager/projected_keepouts` in `map` with reliable, transient-local QoS
  (PR #150, commit `b949e4f`).
- Existing `zones_geojson.py` renders a 100-cost core and uses an exponential,
  bounded halo controlled by `degrade_radius_m`, `degrade_edge_cost`, and
  `degrade_min_cost`.
- Current Nav2 configurations have rolling local `odom` and global `map`
  costmaps, and retain the fixed mask KeepoutFilter.
- Nav2 Humble exposes `nav2_costmap_2d::CostmapLayer`, `updateBounds`,
  `updateCosts`, and `updateWithMax`; its layer API was inspected in the Humble
  container on 2026-08-30.

## Design inference

The vector layer owns no world raster. On every costmap update it filters source
polygons by bounding box against the requested update window plus halo and
rasterizes only that bounded patch. The source stays in `map`; for a local
costmap the patch is transformed from `map` to the costmap frame on each update.
Revision replacement reports both previous and new bounds, permitting Nav2 to
recompute cells left behind by removed or moved geometry. Costs only raise the
master grid; this layer never overwrites obstacle/inflation costs downward.

## Limits / hardware status

This is an opt-in plugin only. The legacy KeepoutFilter remains authoritative
until #146 validates global long-range and local map→odom-correction behavior in
simulation and hardware-representative evidence. No robot or hardware launch
was run for this change.
