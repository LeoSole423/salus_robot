# salus_navigation_costmap

`VectorKeepoutLayer` is an opt-in Nav2 Humble `CostmapLayer` that consumes the
revisioned map-frame polygons published by `zones_manager`. It allocates only a
requested costmap update patch and uses polygon bounding boxes before testing
cells; it never creates or publishes a world-sized raster.

The source topic is `/zones_manager/projected_keepouts` (`ProjectedKeepoutState`,
reliable + transient-local). `map_frame` defaults to `map`. For a global map
costmap no transform is required. For an `odom` local costmap, the layer looks
up `odom <- map` during updates so map-fixed zones follow localization
corrections rather than robot motion.

Cost is max-merged into the Nav2 master grid: core cells are lethal (254) and
the optional halo follows the legacy exponential profile. It does not lower
costs from obstacle or inflation layers. On a revision it expands bounds for
both prior and new polygons so Nav2 can recompute old cells.

It is deliberately disabled by default. #145 does not alter the production
Nav2 plugin lists or retire the legacy `KeepoutFilter`; #146 owns activation and
long-range validation.

Parameters: `enabled` (bool, `false`), `source_topic` (string), `map_frame`
(string, `map`), `halo_radius_m` (double, `1.5 m`), `halo_edge_cost` (integer
1..99, `12`) and `halo_min_cost` (integer 1..edge, `1`).
