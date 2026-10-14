# #320 — optimization and stationary Jetson validation

## Conclusion

The optimized deployed radial filter sustained the observed RS16 rate for ten stationary minutes: 10.00 Hz through scan_clean, with zero missing source stamps within the common input/output window (5999 matched input stamps). The fresh original baseline delivered 9.44 Hz with 68/1200 source stamps missing. Both windows contained approximately 7.7–7.8k points/cloud. This does not establish full-navigation performance or ten-minute throughput for the earlier ~13k-point scene.

The earlier ~13k-point original observation delivered about 7 Hz. A disconnected prototype on one such cloud reduced median adapter work 129.79→73.59 ms and preserved XYZ output bytes. Eight smaller later clouds and two profiles yielded 48 identical complete-message pairs under interleaved execution. Characterization tests also cover binary layouts and NaN/Inf semantics; the classifier itself is unchanged.

## Active deployment and controls

- Deployed source: `2a3c1896556fd955b05ccae6af774713bcfcc76e`, clean branch `agent/issue320-stationary-runtime`, cherry-picked onto the prior `0b7e667`.
- Active ARM64 image: `sha256:3601eeb4be47f9a9d2faccd61fa754222ac92268a305caecb2de6b773c7281c0`. Only tf2 core/ros/py were upgraded to 0.25.24; Nav2 Collision Monitor remains 1.1.20.
- The operator authorized deployment/restart and confirmed robot stationary/E-stop accessible. No movement or navigation commands were sent.
- Urban slopes 10°/13°, divider 1°, height/distance split 0.20 m, range 20 m, target base_footprint, input BEST_EFFORT depth=1. Ground filter, pointcloud-to-scan, noise and Collision Monitor live parameter blocks are identical before/after.
- RS16 rate/resolution/detection settings and safety limits were not changed. Driver uses host clock and first-point stamps.
- Private configuration and calibration/datum files were left in place. Runtime state was copied to the new cache; original source/image/manifest retained for rollback. Active workspace has an alias in the canonical cache root for future normal preparation.
- This bounded deployment does **not** include #324 scan-freshness correction or #325 heading changes. No claim is made about physical stop behavior or readiness for navigation.

## Metrics

Age is Jetson wall-clock receipt minus source header stamp, in ms. Hz uses monotonic receipt span; percentiles are linearly interpolated. Matched-stage deltas use monotonic receipt with identical stamps. They include scheduling/transport and are not callback execution timings.

| Metric | Original baseline (120 s) | Optimized (600 s) |
|---|---:|---:|
| /scan_3d Hz | 10.001 | 10.000 |
| /scan_3d age p50, ms | 106.9 | 104.7 |
| /scan_3d age p95, ms | 119.7 | 110.7 |
| /scan_3d age p99, ms | 127.1 | 116.1 |
| /scan_3d age max, ms | 140.3 | 138.9 |
| /scan_3d max receipt gap, ms | 133.9 | 130.8 |
| /obstacles_cloud Hz | 9.434 | 10.001 |
| /obstacles_cloud age p50, ms | 222.1 | 156.5 |
| /obstacles_cloud age p95, ms | 310.8 | 209.8 |
| /obstacles_cloud age p99, ms | 349.8 | 236.6 |
| /obstacles_cloud age max, ms | 447.5 | 290.8 |
| /obstacles_cloud max receipt gap, ms | 359.5 | 212.8 |
| /scan Hz | 9.436 | 10.002 |
| /scan age p50, ms | 224.3 | 157.9 |
| /scan age p95, ms | 313.9 | 212.4 |
| /scan age p99, ms | 351.6 | 240.3 |
| /scan age max, ms | 448.5 | 291.9 |
| /scan max receipt gap, ms | 356.4 | 212.6 |
| /scan_clean Hz | 9.436 | 10.002 |
| /scan_clean age p50, ms | 227.8 | 160.1 |
| /scan_clean age p95, ms | 321.5 | 216.9 |
| /scan_clean age p99, ms | 354.8 | 246.0 |
| /scan_clean age max, ms | 452.1 | 295.7 |
| /scan_clean max receipt gap, ms | 344.1 | 212.9 |

| Same-stamp stage delta, ms | Original p50/p95/p99 | Optimized p50/p95/p99 |
|---|---:|---:|
| /scan_3d -> /obstacles_cloud | 115.4/197.3/238.1 | 51.6/102.2/128.7 |
| /obstacles_cloud -> /scan | 1.3/8.9/14.6 | 1.2/7.0/12.7 |
| /scan -> /scan_clean | 2.4/11.1/20.0 | 2.0/8.8/15.3 |

All observed stage pairs preserve the stamp. Counts after deployment are 6001 input clouds and 6000 messages per later stage. The unmatched output stamp precedes the first observed input by ~103 ms; the two unmatched input stamps are at the end, after the last output. These are observation-window boundaries, not observed sustained frame loss. No duplicates or interior downstream losses appear.

| Process | CPU mean % original/optimized | RSS median MiB original/optimized |
|---|---:|---:|
| scan_ground_filter | 79.8/53.8 | 69.5/63.6 |
| pointcloud_to_laserscan_node | 2.7/2.4 | 28.7/25.1 |
| scan_noise_filter | 3.6/3.7 | 52.7/51.4 |
| /nav2_collision_monitor/collision_monitor | 4.2/4.0 | 25.8/23.5 |
| nav_snapshot_server | 31.2/29.2 | 194.2/179.6 |

CPU 100% denotes one core, measured from /proc tick deltas every second. Filter mean CPU falls 79.8→53.8% (~33% reduction); clean-scan p99 age falls 354.8→246.0 ms (~31%). Filter RSS median falls 69.5→63.6 MiB. The observer costs ~12.2% of a core and stores only metadata, not cloud payloads; its RSS growth reflects retained JSON rows. Filter RSS stays bounded at ~63.6 MiB.

No increasing age trend is seen: clean-scan age means in the first/last quarters are 173.5/170.5 ms. The max clean-scan age is 295.7 ms; max receipt gap 212.9 ms. The rate claim does not imply perfectly periodic delivery or zero scheduler jitter.

All 6000 wheel odometry samples reported zero linear and angular velocity. This is telemetry plus operator confirmation, not an independent physical stopping test.

## Observer impact control

A separate 60 s scans-only window after the full observation delivered 600
scan_clean messages at 10.006 Hz, age p50/p95/p99 153.0/198.3/223.3 ms.
Observer mean CPU was 9.0% of a core versus 12.2% with all four stages;
filter mean CPU was 53.0% versus 53.8%. The measured rate remains 10 Hz
without the cloud subscriptions. Age percentiles differ by ~10–23 ms;
sequential unequal windows cannot distinguish probe impact from natural load
variation. This establishes a measurable instrumentation cost, not zero impact.

Final bounded search of the last 15 minutes of container logs (up to 20000
lines) found no timestamp/source-ignoring, MessageFilter-drop, cloud-rejection
or extrapolation matches. Service is active/running with NRestarts=0 following
the authorized activation. No additional service restarts were performed.

## Verification and limits

- PC: 14 packages built; full suite 1266 tests, zero failures/errors, two skips; LiDAR, safety and navigation-snapshot smokes pass.
- ARM64: 21 focused tests pass in the new image; 16-package physical workspace builds. No hardware launches were run during image preparation.
- CI run 38589290236 at code/evidence head 9e92508 passed build-unit, classify-changes and six selected smokes. The later final documentation commit may create a separate run.
- Original baseline and optimized observation are sequential, not randomized, and have differing duration. tf2 update and service restart accompany deployment; the entire live improvement cannot be attributed experimentally to conversion alone. Disconnected interleaved replay provides independent evidence that conversion is cheaper with equivalent output.
- Ages at scan_clean are observations at this sidecar, not direct receipt/internal validity in Collision Monitor. No relevant rosout warnings occurred in the long observation; bounded log search is recorded separately. Absence does not establish all consumers accepted every scan.
- The earlier bags are versioned historical context, not an A/B control; see the initial report and #277 provenance. No new timestamp/TF rejection has been established that warrants reopening #277.
- Classification equivalence is demonstrated for the stated corpus/cases, not obstacle-detection quality in every terrain. High-load navigation and larger-cloud sustained validation remain outside this stationary test.

## Reproduce

Raw metadata and /proc samples: predeploy-baseline.json, postdeploy-600.json, plus scans-only controls. probe.py subscribes locally inside the active container with BEST_EFFORT depth=1; no publications. Run windows sequentially with sensors already active, stationary robot and E-stop. analyze.py creates summary.json. profile-multicloud.py captures at most eight clouds (under 4 MiB), disconnects, and alternates original/candidate urban/rural work. Effective versions/hashes/parameters are in provenance.txt and postdeploy-provenance.txt; upgrade-tf2.sh, activate.sh and rollback.sh record the deployment procedures without credentials.
