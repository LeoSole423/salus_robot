# #320 — preserve radial classification while reducing cloud conversion cost

## Intent and evidence

The stationary Jetson runtime at `0b7e667` uses the radial ground classifier,
not the previous fixed-height filter. Its source/install symlink, live
parameters and ROS endpoints were inspected before measuring. On approximately
13k-point clouds, input was 10 Hz and scan_clean approximately 7 Hz. Input
depth=1 bounds the queue but does not preserve the sensor rate. Source-age
percentiles include driver acquisition and scheduling and must not be described
as filter execution time.

The current adapter converts the structured array returned by Humble
`read_points` to Python tuples, then back to float64 before transformation.
A disconnected Jetson prototype removing that round trip reduced a single-cloud
median from 129.79 to 73.59 ms with identical serialized XYZ output. Eight later
real clouds, urban/rural profiles and interleaved execution yielded 48 equal
complete message pairs. Those clouds were smaller, approximately 7.7k points;
they are not a controlled comparison against the first observation.

## Bounded change

- Owner: `salus_perception/scan_ground_filter.py`.
- Keep `read_points` and its selected XYZ fields and dense/NaN policy.
- Stack the three named fields directly and explicitly retain float64, matching
  the former Python float conversion before transformation.
- Keep radial classification, point order, serialized XYZ32 output, preserved
  stamp, output frame, latest-transform lookup and missing-TF behavior unchanged.
- Preserve input BEST_EFFORT KEEP_LAST depth=1 and every detection/safety setting.
- Apply the existing development-image tf2 >=0.25.24 requirement to the real
  Dockerfile too; inspected Jetson started on 0.25.22.

## Verification and deployment boundary

Characterization covers empty/nonempty, reordered XYZ with an extra field and
padding, float32/float64, both byte orders and dense/non-dense NaN/Inf inputs,
comparing old/new conversions and serialized urban/rural outputs. Existing
adapter tests continue to cover frame/stamp and NaN behavior.

PC validation: 14-package build; 1266 tests, zero errors/failures, two skips;
LiDAR, safety and navigation-snapshot smokes. These do not establish physical
motion safety or full-navigation performance.

The operator authorized deploying the optimization, updating tf2 and restarting
the real service for stationary measurement. Runtime deployment is based on its
existing `0b7e667` and cherry-picks only this bounded change; newer main
navigation/heading changes and scan-freshness PR #324 are not part of this
deployment. Preserve private configuration and runtime state, retain original
image/manifest/source for rollback, and inspect effective versions/config after
restart. Record the resulting stationary metrics in the companion evidence;
do not claim >=10 Hz from disconnected profiling alone.
