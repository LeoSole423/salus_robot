# Offline Smac replay

The native harness loads the installed Humble Smac plugin and calls
`createPlan(start, goal)` on each finite segment. It configures a costmap with
no sensor layers, does not activate a costmap update loop, and never starts a
controller or sends motion actions. Run in a container with `--network none`;
ROS initialization and lifecycle publishers remain inside that container.

After building this workspace and sourcing `install/setup.bash`:

```bash
python3 tools/test_loop_corner_planner.py --output-dir artifacts/loop-corner-planner
```

The script builds the harness, obtains the current dispatch yaw from the
production policy, and compares it to the captured request in a translated
fixture. It gates the reproduced baseline, the shorter corrected replan, the
initial plan and the existing sharp-join protection. Output includes exact
input parameters, all path points and a compact summary. This tests the
planner and request geometry, not the executed trajectory or physical safety.

An optional `--costmap-dir` accepts `initial-lower.bin`, `initial-upper.bin`,
`replan-lower.bin` and `replan-upper.bin`, with dimensions/origin matching the
fixture. These are reconstructed costs from the corresponding bag's recorded
OccupancyGrid; lower/upper represent the lossy inverse of Nav2's translation
table. They are not exact internal costmap snapshots. Unknown, inscribed and
lethal costs must map to 255, 253 and 254. Do not replace them with a linear
0–100→0–255 scaling or add another inflation layer to already inflated data.

Input/output paths are explicit. Original bags are not modified or included
in this repository. Runtime planner configuration comes from
`src/salus_navigation/config/nav2_core_real.yaml`; default plugin parameters
come from the installed Nav2 version, which CMake records in its output.
