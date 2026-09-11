# RPP search bound and NavigateThroughPoses retries

## Scope

This characterization isolates the two remaining items from issue #244. It
does not change planner/controller tuning, path geometry, route anchoring,
recovery trees, safety or hardware behavior except for the single RPP search
bound proven by the A/B below.

## RPP search-bound A/B

The pre-change profile did not set `max_robot_pose_search_dist`. In the
Humble RPP used by the simulation, its effective value was the local costmap
extent, observed as `14.975 m`. The comparison used the existing route smoke
stack and `/received_global_plan` from the real RPP controller.

The deterministic path contained two close passages near the robot. The later
passage was beyond 4 m of integrated path distance from the path start.

| Search bound | First selected branch | Evidence |
| --- | --- | --- |
| `14.975 m` (pre-change effective value) | later crossing | first transformed pose was on the later `0.1 m` passage |
| `4.0 m` | first local passage | first transformed pose was on the local `0.5 m` passage |

This is a controller branch-selection effect. It does not change the planner's
generated geometry or claim to fix planner self-intersections. The bound is
now set to `4.0` in `nav2_core_sim.yaml`,
`nav2_core_no_obstacles_sim.yaml` and `nav2_core_real.yaml`, keeping the
profiles identical apart from `use_sim_time`.

The open-route route smoke remained green with the bound enabled; it produced
the same forward route contract and no branch/backtracking failure.

## NavigateThroughPoses retries

The current multi-pose tree uses one retry in both its planning and
`FollowPath` recovery nodes. The legacy tree uses two in both places.

The existing route and patrol smokes do not provide a way to inject a
transient planner/controller failure and observe recovery attempts. Sending an
invalid goal would be a persistent invalid-input test, not a transient fault;
deactivating lifecycle nodes would test lifecycle disruption rather than the
controller action failure contract. Adding either would introduce artificial
infrastructure outside this cut.

Therefore no retry value was changed. The one-versus-two comparison remains
explicitly **not characterized** and is a follow-up item requiring an existing
fault-injection-capable harness or replay evidence.
