# Keep intermediate Nav2 goals current while following a healthy path

## Physical evidence

The operator reported three loops on 2026-09-24/25. Bags and incident notes are
stored only in the Jetson diagnostics directory.

At 00:13:22 UTC a two-pose route chunk contained a synthetic intermediate pose
at map y≈6334 and a real checkpoint at y≈6285. The robot passed the synthetic
pose around 00:13:50 while its healthy path continued south. At 00:14:14 a
keepout revision triggered replanning, when the robot was at y≈6303. Nav2's
new path returned more than 30 m north to the old synthetic pose, then headed
south again. `nav_observer` measured a 38.42 m backward leg and 6.35 detour
ratio. The global costmap showed zero cost along the forward corridor in the
available snapshot; nearby LiDAR returns did not show a blocked road.

## Cause

The Humble `RemovePassedGoals` BT action removes only the first nonterminal
goal when the robot is within its configured radius. The current tree ticked
it only inside `ComputeHealthyPathThroughPoses`, which runs when a new path is
needed. A pose passed while the existing path remains healthy is therefore
still in `{goals}` at a later replan. At that point it can be far outside the
2.5 m radius and cannot be removed.

## Change and boundary

Tick the existing `RemovePassedGoals` node before the keep-or-replan fallback,
inside the existing 2 Hz rate controller. This keeps the ordered goal list
current as the robot passes intermediate poses without changing the current
path, planner, costmap, controller or terminal goal. Set the node's frames to
`map` and `base_footprint`, matching this robot's Nav2 frame contract.

The node still depends on fresh TF and a 2.5 m proximity visit. This addresses
the captured stale-intermediate-goal mechanism; it does not rule out a loop
required by obstacles or heading constraints. The real Jetson service must be
restarted to load the updated BT XML and validated with an operator present.
