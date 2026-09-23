from math import isclose, nan
from contextlib import nullcontext
from types import SimpleNamespace
import pytest
from salus_navigation.route_model import PreparedRoute, RouteChunk, RouteMission, RoutePhase, RouteWaypoint
from salus_navigation.route_preparation import (
    dispatch_yaws, expand, prepare, resolve_yaws, use_curve_tangent_policy,
)
from salus_navigation.route_preparation import validate_inputs
from salus_navigation.route_anchor import select_anchor
from salus_navigation.route_chunker import (
    build_chunk, limit_chunk_to_horizon, next_start, resolve_dispatch_start,
)
from salus_navigation.route_checkpoint_tracker import (
    CheckpointOccurrence, IntermediateCheckpointTracker,
)
from salus_navigation.route_progress import project
from salus_navigation.route_executor_node import RouteExecutorNode, chunk_goal_request
from salus_navigation.patrol_domain import PatrolMachine, PatrolMissionSpec, PatrolPhase, PatrolRoute

def point(x, index): return RouteWaypoint(0, 0, nan, index, map_x=x, map_y=0)

def test_expansion_marks_synthetic_points_and_resolves_yaw():
    route = resolve_yaws(expand([point(0, 0), point(10, 1)], 2.0, False), False)
    assert len(route) == 6 and route[1].key is False and route[0].yaw_deg == 0.0
    assert route[-1].key is True and route[-1].input_index == 1


def test_expansion_avoids_a_synthetic_almost_on_top_of_a_checkpoint():
    # Captured 12 -> 13 leg: a sample at 35 m left only 1.8 m for Nav2 to
    # change from the incoming heading to the checkpoint's corner tangent.
    route = prepare([
        RouteWaypoint(0, 0, nan, 12, map_x=0.0, map_y=0.0),
        RouteWaypoint(0, 0, nan, 13, map_x=36.8, map_y=0.0),
        RouteWaypoint(0, 0, nan, 14, map_x=66.8, map_y=-20.0),
    ], loop=False, input_count=3, spacing_m=35.0,
        chunk_span_m=120.0, chunk_max_waypoints=5, curve_tangent=True)

    assert [(p.input_index, p.key) for p in route.waypoints] == [
        (12, True), (13, True), (14, True),
    ]
    assert 0.0 > route.waypoints[1].yaw_deg > -45.0


def test_expansion_keeps_useful_synthetics_on_a_long_leg():
    route = prepare([
        point(0.0, 0), point(71.8, 1),
    ], loop=False, input_count=2, spacing_m=35.0,
        chunk_span_m=120.0, chunk_max_waypoints=5)

    assert [(p.input_index, p.key, p.map_x) for p in route.waypoints] == [
        (0, True, 0.0), (0, False, 35.0), (1, True, 71.8),
    ]


def test_long_chunk_uses_reachable_synthetic_as_terminal_without_credit():
    route = prepare([
        point(0.0, 43), point(149.0, 44),
    ], loop=False, input_count=2, spacing_m=35.0,
        chunk_span_m=120.0, chunk_max_waypoints=5)
    original = build_chunk(route, 0, mode="adaptive_dense")

    first = limit_chunk_to_horizon(route, original, robot_xy=(-20.0, 0.0),
                                   max_goal_distance_m=120.0)
    assert first is not None
    assert [point.map_x for point in first.waypoints] == [0.0, 35.0, 70.0]
    assert first.checkpoint_offsets == (0,)
    assert first.waypoints[-1].key is False
    assert next_start(route, first) == 3
    assert len(chunk_goal_request(first, route).lats) == 3

    second = build_chunk(route, next_start(route, first), mode="adaptive_dense")
    assert second.checkpoint_offsets == (1,)
    assert second.waypoints[-1].input_index == 44
    assert limit_chunk_to_horizon(route, second, robot_xy=(70.0, 0.0),
                                  max_goal_distance_m=120.0) is second


def test_horizon_credits_a_reachable_key_then_fails_closed_without_synthetic():
    route = prepare([point(0.0, 0), point(149.0, 1)], loop=False,
                    input_count=2, spacing_m=0.0,
                    chunk_span_m=120.0, chunk_max_waypoints=5)
    chunk = build_chunk(route, 0, mode="adaptive_dense")
    first = limit_chunk_to_horizon(route, chunk, robot_xy=(0.0, 0.0),
                                   max_goal_distance_m=120.0)
    assert first.checkpoint_offsets == (0,)
    assert first.waypoints[-1].input_index == 0
    second = build_chunk(route, next_start(route, first), mode="adaptive_dense")
    assert limit_chunk_to_horizon(route, second, robot_xy=(0.0, 0.0),
                                  max_goal_distance_m=120.0) is None


def test_loop_long_closure_uses_synthetic_then_preserves_next_lap_key():
    route = prepare([
        point(0.0, 0), point(100.0, 1), point(250.0, 2),
    ], loop=True, input_count=3, spacing_m=35.0,
        chunk_span_m=120.0, chunk_max_waypoints=5)
    last_key = max(i for i, p in enumerate(route.waypoints) if p.key)
    original = build_chunk(route, last_key, iteration=0,
                           mode="adaptive_dense")

    first = limit_chunk_to_horizon(
        route, original, robot_xy=(260.0, 0.0),
        max_goal_distance_m=120.0,
    )

    assert first.waypoints[-1].key is False
    assert first.checkpoint_occurrences == ((0, 2, 0),)
    assert first.end < len(route.waypoints) - 1
    remainder = build_chunk(route, next_start(route, first), iteration=0,
                            mode="adaptive_dense")
    assert remainder.waypoints[-1].input_index == 0
    assert remainder.checkpoint_occurrences[-1][2] == 1


def test_open_route_final_automatic_yaw_follows_its_incoming_leg():
    route = resolve_yaws([
        RouteWaypoint(0, 0, nan, 0, map_x=0, map_y=0),
        RouteWaypoint(0, 0, nan, 1, map_x=0, map_y=10),
    ], False)

    assert [point.yaw_deg for point in route] == [90.0, 90.0]


def test_route_tangent_mode_splits_a_right_angle_and_keeps_leg_synthetics():
    route = prepare([
        RouteWaypoint(0, 0, nan, 0, map_x=0.0, map_y=0.0),
        RouteWaypoint(0, 0, nan, 1, map_x=10.0, map_y=0.0),
        RouteWaypoint(0, 0, nan, 2, map_x=10.0, map_y=10.0),
    ], loop=False, input_count=3, spacing_m=5.0,
        chunk_span_m=120.0, chunk_max_waypoints=5, curve_tangent=True)

    corner = next(p for p in route.waypoints if p.key and p.input_index == 1)
    synthetic = next(p for p in route.waypoints if not p.key and p.input_index == 1)
    assert isclose(corner.yaw_deg, 45.0, abs_tol=1e-6)
    assert isclose(synthetic.yaw_deg, 90.0, abs_tol=1e-6)
    chunk = build_chunk(route, 2)
    assert list(chunk_goal_request(chunk, route, approach_xy=(9.0, -1.0)).yaws_deg) == [45.0]


def test_route_tangent_mode_preserves_operator_yaw_and_legacy_option():
    points = [
        RouteWaypoint(0, 0, nan, 0, map_x=0.0, map_y=0.0),
        RouteWaypoint(0, 0, -30.0, 1, map_x=10.0, map_y=0.0, yaw_explicit=True),
        RouteWaypoint(0, 0, nan, 2, map_x=10.0, map_y=10.0),
    ]
    tangent = resolve_yaws(points, False, curve_tangent=True)
    legacy = resolve_yaws(points, False)
    assert tangent[1].yaw_deg == legacy[1].yaw_deg == -30.0
    assert dispatch_yaws(tuple(tangent), curve_tangent=True)[1] == -30.0
    assert legacy[0].yaw_deg == 0.0


def test_unspecified_public_yaw_policy_uses_tangents_for_all_routes():
    assert use_curve_tangent_policy("")
    assert use_curve_tangent_policy("route_tangent")
    assert not use_curve_tangent_policy("legacy")
    with pytest.raises(ValueError, match="auto_yaw_policy"):
        use_curve_tangent_policy("unknown")
    points = [
        RouteWaypoint(0, 0, nan, 0, map_x=0.0, map_y=0.0),
        RouteWaypoint(0, 0, nan, 1, map_x=10.0, map_y=0.0),
        RouteWaypoint(0, 0, nan, 2, map_x=10.0, map_y=10.0),
    ]
    default = prepare(points, loop=False, input_count=3, spacing_m=0.0,
                      chunk_span_m=120.0, chunk_max_waypoints=5,
                      curve_tangent=use_curve_tangent_policy(""))
    assert default.auto_yaw_policy == "route_tangent"
    assert isclose(default.waypoints[1].yaw_deg, 45.0, abs_tol=1e-6)


def test_tangent_dispatch_uses_approach_for_first_automatic_key_only():
    # Patrol's join starts ~6 m in front of the robot. Asking that first
    # through-pose for its later -90 degree corner tangent made Dubins loop.
    keys = (
        RouteWaypoint(0, 0, -90.0, 3, map_x=6.0, map_y=0.0),
        RouteWaypoint(0, 0, 0.0, 0, map_x=20.0, map_y=-14.0),
        RouteWaypoint(0, 0, 90.0, 1, map_x=34.0, map_y=0.0),
    )
    assert dispatch_yaws(keys, approach_xy=(0.0, 0.0), curve_tangent=True) == [
        0.0, 0.0, 90.0,
    ]
    assert dispatch_yaws(keys, approach_xy=(0.0, 0.0),
                         approach_heading_deg=0.0, curve_tangent=True) == [
        0.0, 0.0, 90.0,
    ]
    synthetic = RouteWaypoint(0, 0, -30.0, 3, key=False,
                              map_x=6.0, map_y=0.0)
    assert dispatch_yaws((synthetic, *keys[1:]), approach_xy=(0.0, 0.0),
                         curve_tangent=True)[0] == -30.0
    explicit = RouteWaypoint(0, 0, -90.0, 3, map_x=6.0, map_y=0.0,
                             yaw_explicit=True)
    assert dispatch_yaws((explicit,), approach_xy=(0.0, 0.0),
                         curve_tangent=True) == [-90.0]
    gentle = RouteWaypoint(0, 0, 45.0, 0, map_x=0.0, map_y=6.0)
    assert dispatch_yaws((gentle,), approach_xy=(0.0, 0.0),
                         curve_tangent=True) == [45.0]
    north = RouteWaypoint(0, 0, 90.0, 0, map_x=0.0, map_y=6.0)
    assert dispatch_yaws((north,), approach_xy=(0.0, 0.0),
                         approach_heading_deg=0.0, curve_tangent=True) == [0.0]


def test_finite_chunk_changes_only_an_automatic_terminal_yaw_to_its_incoming_leg():
    chunk = (
        RouteWaypoint(0, 0, 0.0, 0, map_x=0.0, map_y=0.0),
        RouteWaypoint(0, 0, 90.0, 1, map_x=10.0, map_y=0.0),
    )

    assert dispatch_yaws(chunk) == [0.0, 0.0]
    assert chunk[-1].yaw_deg == 90.0


def test_finite_chunk_preserves_an_explicit_terminal_yaw():
    chunk = (
        RouteWaypoint(0, 0, 0.0, 0, map_x=0.0, map_y=0.0),
        RouteWaypoint(0, 0, 90.0, 1, map_x=10.0, map_y=0.0, yaw_explicit=True),
    )

    assert dispatch_yaws(chunk) == [0.0, 90.0]


def test_finite_chunk_uses_approach_heading_for_first_automatic_checkpoint():
    chunk = (
        RouteWaypoint(0, 0, 0.0, 0, map_x=0.0, map_y=0.0),
        RouteWaypoint(0, 0, 90.0, 1, map_x=10.0, map_y=0.0),
    )

    assert dispatch_yaws(chunk, approach_xy=(0.0, -10.0)) == [90.0, 0.0]


def test_finite_chunk_preserves_explicit_first_checkpoint_yaw():
    chunk = (
        RouteWaypoint(0, 0, -30.0, 0, map_x=0.0, map_y=0.0, yaw_explicit=True),
        RouteWaypoint(0, 0, 90.0, 1, map_x=10.0, map_y=0.0),
    )

    assert dispatch_yaws(chunk, approach_xy=(0.0, -10.0)) == [-30.0, 0.0]


def test_chunk_request_uses_terminal_incoming_yaw_for_automatic_checkpoints():
    automatic_route = prepare(
        [
            RouteWaypoint(0, 0, nan, 0, map_x=0.0, map_y=0.0),
            RouteWaypoint(0, 0, nan, 1, map_x=10.0, map_y=0.0),
            RouteWaypoint(0, 0, nan, 2, map_x=10.0, map_y=10.0),
        ],
        loop=True, input_count=3, spacing_m=0,
        chunk_span_m=120, chunk_max_waypoints=5,
    )
    automatic_chunk = build_chunk(automatic_route, 0)

    assert [point.yaw_deg for point in automatic_chunk.waypoints] == [0.0]
    assert list(chunk_goal_request(automatic_chunk, automatic_route).yaws_deg) == [0.0]

    explicit_route = prepare(
        [
            RouteWaypoint(0, 0, nan, 0, map_x=0.0, map_y=0.0),
            RouteWaypoint(0, 0, 90.0, 1, map_x=10.0, map_y=0.0, yaw_explicit=True),
            RouteWaypoint(0, 0, nan, 2, map_x=10.0, map_y=10.0),
        ],
        loop=True, input_count=3, spacing_m=0,
        chunk_span_m=120, chunk_max_waypoints=5,
    )
    explicit_chunk = build_chunk(explicit_route, 0)

    assert list(chunk_goal_request(explicit_chunk, explicit_route).yaws_deg) == [0.0]


def test_chunk_request_does_not_mutate_first_yaw_from_robot_approach():
    route = prepare(
        [
            RouteWaypoint(0, 0, nan, 0, map_x=0.0, map_y=0.0),
            RouteWaypoint(0, 0, nan, 1, map_x=10.0, map_y=0.0),
            RouteWaypoint(0, 0, nan, 2, map_x=10.0, map_y=10.0),
        ],
        loop=True, input_count=3, spacing_m=0,
        chunk_span_m=120, chunk_max_waypoints=5,
    )
    chunk = build_chunk(route, 0)

    assert list(chunk_goal_request(chunk, route).yaws_deg) == [0.0]


def test_chunk_request_uses_current_pose_for_automatic_first_yaw():
    route = prepare(
        [
            RouteWaypoint(0, 0, nan, 0, map_x=0.0, map_y=0.0),
            RouteWaypoint(0, 0, nan, 1, map_x=10.0, map_y=0.0),
        ],
        loop=False, input_count=2, spacing_m=0,
        chunk_span_m=120, chunk_max_waypoints=5,
    )
    chunk = build_chunk(route, 0)

    assert list(chunk_goal_request(
        chunk, route, approach_xy=(0.0, -10.0)
    ).yaws_deg) == [90.0]


def test_chunk_request_preserves_explicit_first_yaw_with_current_pose():
    route = prepare(
        [
            RouteWaypoint(
                0, 0, -30.0, 0, map_x=0.0, map_y=0.0,
                yaw_explicit=True,
            ),
            RouteWaypoint(0, 0, nan, 1, map_x=10.0, map_y=0.0),
        ],
        loop=False, input_count=2, spacing_m=0,
        chunk_span_m=120, chunk_max_waypoints=5,
    )
    chunk = build_chunk(route, 0)

    assert list(chunk_goal_request(
        chunk, route, approach_xy=(0.0, -10.0)
    ).yaws_deg) == [-30.0]


def test_chunk_request_keeps_terminal_incoming_yaw_for_automatic_pose():
    route = prepare(
        [
            RouteWaypoint(0, 0, nan, 0, map_x=0.0, map_y=0.0),
            RouteWaypoint(0, 0, nan, 1, map_x=10.0, map_y=0.0),
        ],
        loop=False, input_count=2, spacing_m=0,
        chunk_span_m=120, chunk_max_waypoints=5,
    )
    chunk = build_chunk(route, 0)

    assert list(chunk_goal_request(chunk, route).yaws_deg)[-1] == 0.0


def test_chunk_request_preserves_explicit_terminal_yaw_with_current_pose():
    route = prepare(
        [
            RouteWaypoint(0, 0, nan, 0, map_x=0.0, map_y=0.0),
            RouteWaypoint(
                0, 0, 90.0, 1, map_x=10.0, map_y=0.0,
                yaw_explicit=True,
            ),
        ],
        loop=False, input_count=2, spacing_m=0,
        chunk_span_m=120, chunk_max_waypoints=5,
    )
    chunk = build_chunk(route, 0)

    assert list(chunk_goal_request(
        chunk, route, approach_xy=(0.0, -10.0)
    ).yaws_deg) == [90.0]


def test_single_automatic_pose_uses_approach_yaw_without_changing_fallback():
    automatic = RouteWaypoint(
        0, 0, 90.0, 1, map_x=10.0, map_y=0.0,
    )

    assert dispatch_yaws((automatic,), approach_xy=(0.0, -10.0)) == [45.0]
    assert dispatch_yaws((automatic,)) == [90.0]


def test_single_explicit_pose_keeps_yaw_with_approach():
    explicit = RouteWaypoint(
        0, 0, 15.0, 1, map_x=10.0, map_y=0.0, yaw_explicit=True,
    )

    assert dispatch_yaws((explicit,), approach_xy=(0.0, -10.0)) == [15.0]


def test_chunk_request_falls_back_when_current_pose_is_invalid():
    route = prepare(
        [
            RouteWaypoint(0, 0, nan, 0, map_x=0.0, map_y=0.0),
            RouteWaypoint(0, 0, nan, 1, map_x=10.0, map_y=0.0),
        ],
        loop=False, input_count=2, spacing_m=0,
        chunk_span_m=120, chunk_max_waypoints=5,
    )
    chunk = build_chunk(route, 0)

    assert list(chunk_goal_request(
        chunk, route, approach_xy=(float("nan"), -10.0)
    ).yaws_deg) == [0.0]

def test_open_anchor_never_moves_backwards():
    route = prepare([point(0,0), point(10,1), point(20,2)], loop=False, input_count=3, spacing_m=0, chunk_span_m=20, chunk_max_waypoints=3)
    assert select_anchor(route, 9.0, 0.2) >= 1


def test_loop_anchor_enters_at_next_waypoint_of_nearby_segment():
    route = PreparedRoute(
        (
            RouteWaypoint(0, 0, 0.0, 0, map_x=0.0, map_y=0.0),
            RouteWaypoint(0, 0, 0.0, 1, map_x=10.0, map_y=0.0),
            RouteWaypoint(0, 0, 0.0, 2, map_x=5.0, map_y=4.0),
        ),
        True, 3, 0.0, 20.0, 5,
    )

    # The robot is midway along 0 -> 1, outside waypoint tolerance from both
    # endpoints. Waypoint 2 is nevertheless the closest vertex; loop
    # incorporation must follow the nearby segment and enter at waypoint 1.
    assert select_anchor(
        route, 5.0, 1.0, reached_tolerance_m=1.2, segment_tolerance_m=1.2,
    ) == 1
    assert select_anchor(
        route, 10.4, 0.0, reached_tolerance_m=1.2, segment_tolerance_m=1.2,
    ) == 2


def test_loop_anchor_respects_configured_segment_tolerance():
    route = PreparedRoute(
        (
            RouteWaypoint(0, 0, 0.0, 0, map_x=0.0, map_y=0.0),
            RouteWaypoint(0, 0, 0.0, 1, map_x=10.0, map_y=0.0),
            RouteWaypoint(0, 0, 0.0, 2, map_x=5.0, map_y=4.0),
        ),
        True, 3, 0.0, 20.0, 5,
    )

    assert select_anchor(
        route, 5.0, 1.0, reached_tolerance_m=1.2, segment_tolerance_m=1.2,
    ) == 1
    assert select_anchor(
        route, 5.0, 1.0, reached_tolerance_m=1.2, segment_tolerance_m=0.5,
    ) == 2


def test_loop_chunk_does_not_contain_a_complete_circuit():
    route = prepare([point(0,0),point(2,1),point(4,2),point(6,3)], loop=True,input_count=4,spacing_m=0,chunk_span_m=100,chunk_max_waypoints=10)
    chunk = build_chunk(route, 0)
    assert len(chunk.waypoints) == 1 and next_start(route, chunk) == 1


def test_adaptive_dense_loop_leaves_one_point_for_the_next_finite_request():
    route = prepare(
        [
            RouteWaypoint(0, 0, nan, 0, map_x=0, map_y=0),
            RouteWaypoint(0, 0, nan, 1, map_x=10, map_y=0),
            RouteWaypoint(0, 0, nan, 2, map_x=10, map_y=10),
            RouteWaypoint(0, 0, nan, 3, map_x=0, map_y=10),
        ],
        loop=True, input_count=4, spacing_m=35,
        chunk_span_m=120, chunk_max_waypoints=5,
    )

    chunk = build_chunk(
        route, 3, mode="adaptive_dense",
        adaptive_dense_leg_max_m=20, adaptive_dense_horizon_m=60,
    )

    assert [waypoint.input_index for waypoint in chunk.waypoints] == [3, 0, 1]
    assert next_start(route, chunk) == 2


def test_each_finite_chunk_has_at_most_one_terminal_key_and_advances():
    route = prepare(
        [point(0, 0), point(10, 1), point(20, 2)], loop=False,
        input_count=3, spacing_m=2, chunk_span_m=3, chunk_max_waypoints=2,
    )
    start = 0
    chunks = []
    while start < len(route.waypoints):
        chunk = build_chunk(route, start)
        chunks.append(chunk)
        start = next_start(route, chunk)

    assert all(
        sum(point.key for point in chunk.waypoints) <= 1
        and chunk.waypoints[-1].key
        for chunk in chunks
    )
    assert [chunk.waypoints[-1].input_index for chunk in chunks] == [0, 1, 2]


def test_chunk_ends_at_next_original_checkpoint_like_legacy():
    route = prepare(
        [point(0, 0), point(3, 1), point(6, 2), point(9, 3), point(12, 4)],
        loop=False, input_count=5, spacing_m=35,
        chunk_span_m=120, chunk_max_waypoints=5,
    )

    chunk = build_chunk(route, 0)

    assert [waypoint.input_index for waypoint in chunk.waypoints] == [0]
    assert chunk.checkpoint_offsets == (0,)
    assert next_start(route, chunk) == 1


def test_chunk_soft_limits_never_promote_synthetic_point_to_boundary():
    route = prepare(
        [point(0, 0), point(10, 1), point(20, 2)], loop=False,
        input_count=3, spacing_m=2, chunk_span_m=3, chunk_max_waypoints=2,
    )
    chunk = build_chunk(route, 1)

    assert chunk.waypoints[-1].key is True
    assert chunk.waypoints[-1].input_index == 1
    assert len(chunk.waypoints) > route.chunk_max_waypoints
    assert chunk.checkpoint_offsets == (len(chunk.waypoints) - 1,)


def test_chunk_started_on_synthetic_geometry_dispatches_only_next_checkpoint():
    route = prepare(
        [point(0, 0), point(10, 1), point(20, 2)], loop=False,
        input_count=3, spacing_m=2, chunk_span_m=3, chunk_max_waypoints=2,
    )
    chunk = build_chunk(route, 2)

    assert chunk.waypoints[0].key is False
    assert chunk.waypoints[-1].key is True
    assert chunk.checkpoint_offsets == (len(chunk.waypoints) - 1,)


def test_synthetic_points_never_count_as_dispatchable_checkpoints():
    route = prepare(
        [point(0, 0), point(12, 1)], loop=False,
        input_count=2, spacing_m=2, chunk_span_m=100, chunk_max_waypoints=20,
    )
    chunk = build_chunk(route, 1)

    dispatched = [chunk.waypoints[index] for index in chunk.checkpoint_offsets]
    assert [waypoint.input_index for waypoint in dispatched] == [1]
    assert all(waypoint.key for waypoint in dispatched)


def test_expanded_loop_chunk_ends_at_checkpoint_without_full_circuit():
    route = prepare(
        [point(0, 0), point(10, 1), point(20, 2)], loop=True,
        input_count=3, spacing_m=2, chunk_span_m=1000, chunk_max_waypoints=100,
    )
    chunk = build_chunk(route, 1)

    assert chunk.waypoints[0].key is False
    assert chunk.waypoints[-1].key is True
    assert len(chunk.waypoints) < len(route.waypoints)
    assert all(chunk.waypoints[offset].key for offset in chunk.checkpoint_offsets)


def test_progress_projects_onto_segment_instead_of_nearest_vertex():
    from salus_navigation.route_model import RouteChunk

    route = prepare(
        [point(0, 0), point(10, 1), point(20, 2)], loop=False,
        input_count=3, spacing_m=0, chunk_span_m=100, chunk_max_waypoints=3,
    )
    chunk = RouteChunk(route.waypoints, 0, 2, 0)

    progress = project(chunk, 5.0, 2.0)

    assert progress.expanded_index == 0
    assert progress.checkpoint_index == 0
    assert progress.ratio == 0.25
    assert progress.cross_track_error_m == 2.0
    assert progress.distance_to_target_m > 5.0


def test_progress_does_not_jump_back_to_an_earlier_segment_at_shared_vertex():
    from salus_navigation.route_model import RouteChunk
    points = (
        RouteWaypoint(0, 0, 0, 0, map_x=0, map_y=0),
        RouteWaypoint(0, 0, 0, 1, map_x=10, map_y=0),
        RouteWaypoint(0, 0, 0, 2, map_x=10, map_y=10),
    )

    progress = project(RouteChunk(points, 0, 2, 0), 10.0, 5.0)

    assert progress.expanded_index == 1
    assert progress.checkpoint_index == 1
    assert progress.ratio == 0.75
    assert progress.cross_track_error_m == 0.0


def test_action_checkpoint_is_a_hard_chunk_boundary():
    points = [point(0, 0), point(10, 1), point(20, 2)]
    points[1] = RouteWaypoint(
        **{**points[1].__dict__, "action_json": '[{"type":"brake_hold","duration_s":1}]'}
    )
    route = prepare(
        points, loop=False, input_count=3, spacing_m=2,
        chunk_span_m=1000, chunk_max_waypoints=100,
    )

    chunk = build_chunk(route, 1)

    assert chunk.waypoints[-1].key
    assert chunk.waypoints[-1].input_index == 1
    assert next_start(route, chunk) < len(route.waypoints)


def test_chunk_request_sends_synthetic_geometry_but_counts_only_checkpoint_boundary():
    route = prepare(
        [point(0, 0), point(12, 1)], loop=False, input_count=2,
        spacing_m=2, chunk_span_m=100, chunk_max_waypoints=20,
    )
    chunk = build_chunk(route, 1)

    request = chunk_goal_request(chunk, route)

    assert list(request.lats) == [point.lat for point in chunk.waypoints]
    assert len(request.lats) > len(chunk.checkpoint_offsets)
    assert chunk.checkpoint_offsets == (len(chunk.waypoints) - 1,)
    assert request.loop is False
    assert request.suppress_success_brake is False


def test_intermediate_chunk_suppresses_success_brake_and_loop_chunk_is_finite():
    open_route = prepare(
        [point(0, 0), point(10, 1), point(20, 2)], loop=False,
        input_count=3, spacing_m=2, chunk_span_m=3, chunk_max_waypoints=2,
    )
    open_chunk = build_chunk(open_route, 0)
    assert chunk_goal_request(open_chunk, open_route).suppress_success_brake

    loop_route = prepare(
        [point(0, 0), point(10, 1), point(20, 2)], loop=True,
        input_count=3, spacing_m=2, chunk_span_m=1000, chunk_max_waypoints=100,
    )
    loop_chunk = build_chunk(loop_route, 0)
    request = chunk_goal_request(loop_chunk, loop_route)
    assert request.loop is False
    assert request.suppress_success_brake
    assert len(request.lats) < len(loop_route.waypoints)


def test_chunk_success_counts_only_original_checkpoints_and_advances_once():
    route = prepare(
        [point(0, 0), point(12, 1)], loop=False, input_count=2,
        spacing_m=2, chunk_span_m=100, chunk_max_waypoints=20,
    )
    chunk = build_chunk(route, 1)
    events = []
    advanced = []
    fake = SimpleNamespace(
        _chunk=chunk,
        _target_offset=chunk.checkpoint_offsets[-1],
        _mission=SimpleNamespace(reached=0, mission_id="mission", chunk_id=0, loop_iteration=0),
        _checkpoint_tracker=None,
        _reached_occurrences=set(),
        _event=lambda *args, **kwargs: events.append((args, kwargs)),
        _start_actions=lambda *_args: None,
        _advance=lambda: advanced.append(True),
    )

    RouteExecutorNode._complete_current_chunk(fake, "nav2_succeeded")

    assert fake._mission.reached == 1
    assert [event[1]["input_index"] for event in events] == [1]
    assert len(events) == len(chunk.checkpoint_offsets)
    assert len(events) <= len(chunk.waypoints)
    assert advanced == [True]


def test_synthetic_subgoal_success_advances_without_checkpoint_event():
    route = prepare([point(0.0, 43), point(149.0, 44)], loop=False,
                    input_count=2, spacing_m=35.0,
                    chunk_span_m=120.0, chunk_max_waypoints=5)
    chunk = limit_chunk_to_horizon(
        route, build_chunk(route, 0, mode="adaptive_dense"),
        robot_xy=(-20.0, 0.0), max_goal_distance_m=120.0,
    )
    events, advanced = [], []
    fake = SimpleNamespace(
        _chunk=chunk,
        _target_offset=len(chunk.waypoints) - 1,
        _mission=SimpleNamespace(prepared=route, target_index=0, reached=1,
                                 mission_id="m", chunk_id=1, loop_iteration=0),
        _checkpoint_tracker=None, _reached_occurrences={("m", 0, 43)},
        _event=lambda *args, **kwargs: events.append((args, kwargs)),
        _advance=lambda: advanced.append(True),
    )

    RouteExecutorNode._complete_current_chunk(fake, "nav2_succeeded")

    assert fake._mission.reached == 1
    assert not events
    assert advanced == [True]


def test_duplicate_terminal_credit_does_not_repeat_actions():
    endpoint = RouteWaypoint(
        0, 0, 0, 2,
        action_json='[{"type":"brake_hold","duration_s":1}]',
        map_x=2.0, map_y=0.0,
    )
    chunk = RouteChunk((endpoint,), 2, 2, 0, (0,))
    actions, advanced, events = [], [], []
    fake = SimpleNamespace(
        _chunk=chunk, _target_offset=0, _checkpoint_tracker=None,
        _mission=SimpleNamespace(reached=1, mission_id="m", chunk_id=2,
                                 loop_iteration=0),
        _reached_occurrences={("m", 0, 2)},
        _event=lambda *args, **kwargs: events.append((args, kwargs)),
        _start_actions=lambda *args: actions.append(args),
        _advance=lambda: advanced.append(True),
    )

    RouteExecutorNode._complete_current_chunk(fake, "nav2_succeeded")

    assert not actions and not events
    assert fake._mission.reached == 1
    assert advanced == [True]


def test_terminal_success_with_pending_soft_checkpoint_fails_closed():
    route = prepare(
        [point(0, 0), point(10, 1)], loop=False, input_count=2,
        spacing_m=0, chunk_span_m=120, chunk_max_waypoints=5,
    )
    chunk = build_chunk(route, 0, mode="legacy_pair")
    tracker = IntermediateCheckpointTracker((
        CheckpointOccurrence(0, 0, 0.0, 0.0),
    ))
    events, paused, brakes = [], [], []

    class PendingFuture:
        def add_done_callback(self, _callback):
            return None

    fake = SimpleNamespace(
        _chunk=chunk,
        _target_offset=chunk.checkpoint_offsets[-1],
        _mission=SimpleNamespace(
            reached=0, mission_id="mission", chunk_id=0, loop_iteration=0),
        _checkpoint_tracker=tracker,
        _reached_occurrences=set(),
        _pause=lambda reason: paused.append(reason),
        _brake=SimpleNamespace(
            call_async=lambda request: (
                brakes.append((request.duration_s, request.brake_pct))
                or PendingFuture()
            )),
        _log_failed_brake=lambda _future: None,
        _event=lambda *args, **kwargs: events.append((args, kwargs)),
    )

    RouteExecutorNode._complete_current_chunk(fake, "nav2_succeeded")

    assert paused == ["ROUTE_CHECKPOINT_SEQUENCE_INCOMPLETE"]
    assert brakes == [(0.25, 100)]
    assert fake._mission.reached == 0
    assert events[-1][0][1] == "ROUTE_CHECKPOINT_SEQUENCE_INCOMPLETE"


def test_retry_of_the_same_chunk_preserves_soft_checkpoint_evidence():
    route = prepare(
        [point(0, 0), point(10, 1)], loop=False, input_count=2,
        spacing_m=0, chunk_span_m=120, chunk_max_waypoints=5,
    )
    chunk = build_chunk(route, 0, mode="legacy_pair")
    fake = SimpleNamespace(
        _chunk=chunk,
        _mission=SimpleNamespace(mission_id="mission", chunk_id=3),
        _checkpoint_tracker=None,
        _checkpoint_tracker_key=None,
        _route_progress_pose_max_age_s=0.5,
    )
    RouteExecutorNode._prepare_checkpoint_tracker(fake)
    tracker = fake._checkpoint_tracker
    tracker.observe(
        SimpleNamespace(
            x=0.0, y=0.0, source_stamp_s=10.0, received_steady_s=10.0),
        now_ros_s=10.1,
        now_steady_s=10.1,
    )

    RouteExecutorNode._prepare_checkpoint_tracker(fake)

    assert fake._checkpoint_tracker is tracker
    assert fake._checkpoint_tracker.complete


def test_recovery_reuses_credited_chunk_suffix_after_costmap_clear():
    points = tuple(point(float(index), index) for index in (5, 0, 1, 2))
    chunk = RouteChunk(points, 5, 2, 0, (0, 1, 1, 1))
    dispatched = []
    events = []

    class ReadyClient:
        def service_is_ready(self):
            return True

        def call_async(self, _request):
            return SimpleNamespace(done=lambda: True, result=lambda: object())

    route = PreparedRoute(tuple(point(float(i), i) for i in range(6)),
                          True, 6, 0.0, 20.0, 6)
    mission = SimpleNamespace(mission_id="m", chunk_id=7, loop_iteration=0,
                              prepared=route, target_index=5)
    fake = SimpleNamespace(
        _lock=nullcontext(), _recovery_checkpoint_reached=False,
        _mission=mission, _chunk=chunk, _reached_occurrences={("m", 0, 5)},
        _clear_costmaps=(ReadyClient(), ReadyClient()), _recovery_clears=None,
        _checkpoint_tracker_key=object(), _steady_now=lambda: 1.0,
        get_parameter=lambda _name: SimpleNamespace(value=3.0),
        _event=lambda *args, **kwargs: events.append((args, kwargs)),
        _publish_paths=lambda: None,
        _send_chunk=lambda: dispatched.append(fake._chunk),
    )
    RouteExecutorNode._begin_recovery_retry(
        fake, SimpleNamespace(reason="NAV_ABORTED", attempt=1))
    RouteExecutorNode._check_recovery_clears(fake)

    assert [p.input_index for p in dispatched[0].waypoints] == [0, 1, 2]
    assert dispatched[0].checkpoint_iterations == (1, 1, 1)
    assert mission.mission_id == "m" and mission.chunk_id == 7
    assert mission.loop_iteration == 1
    assert mission.target_index == 0
    assert fake._reached_occurrences == {("m", 0, 5)}
    assert events[0][0][1] == "ROUTE_BLOCKED_RETRYING"

    fake._reached_occurrences.add(("m", 1, 0))
    RouteExecutorNode._begin_recovery_retry(
        fake, SimpleNamespace(reason="NAV_ABORTED", attempt=2))
    RouteExecutorNode._check_recovery_clears(fake)
    assert [p.input_index for p in dispatched[-1].waypoints] == [1, 2]
    assert mission.target_index == 1 and mission.loop_iteration == 1
    assert mission.chunk_id == 7


def test_pose_callback_accounts_for_time_waiting_before_tracker_evaluation():
    observations = []

    class Tracker:
        def observe(self, sample, *, now_ros_s, now_steady_s):
            observations.append((sample, now_ros_s, now_steady_s))
            return SimpleNamespace(accepted=False, occurrence=None)

    steady_times = iter((10.0, 10.7))
    fake = SimpleNamespace(
        _steady_now=lambda: next(steady_times),
        get_clock=lambda: SimpleNamespace(
            now=lambda: SimpleNamespace(nanoseconds=100_200_000_000)
        ),
        _lock=nullcontext(),
        _mission=SimpleNamespace(phase=RoutePhase.ACTIVE),
        _checkpoint_tracker=Tracker(),
        _goal_request_pending=False,
        _record_checkpoint_reached=lambda *_args, **_kwargs: None,
        _pose=None,
        _pose_sample=None,
    )
    message = SimpleNamespace(
        header=SimpleNamespace(
            stamp=SimpleNamespace(sec=100, nanosec=0),
        ),
        pose=SimpleNamespace(
            pose=SimpleNamespace(position=SimpleNamespace(x=1.0, y=2.0)),
        ),
    )

    RouteExecutorNode._on_pose(fake, message)

    sample, now_ros_s, now_steady_s = observations[0]
    assert sample.received_steady_s == 10.0
    assert now_steady_s == 10.7
    assert now_ros_s == 100.2


def test_cancel_route_mission_transitions_paused_state_to_cancelled():
    class ImmediateFuture:
        def add_done_callback(self, callback):
            callback(self)

        def result(self):
            return SimpleNamespace(ok=True, error="")

    class Client:
        def call_async(self, _request):
            return ImmediateFuture()

    mission = RouteMission(phase=RoutePhase.PAUSED, pause_reason="manual takeover")
    fake = SimpleNamespace(
        _lock=nullcontext(),
        _preparation_epoch=0,
        _preparation=object(),
        _goal_request_pending=True,
        _recovery=SimpleNamespace(reset=lambda: None),
        _recovery_clears=object(),
        _recovery_checkpoint_reached=True,
        _checkpoint_tracker=object(),
        _checkpoint_tracker_key=object(),
        _reached_occurrences={1},
        _action=None,
        _action_future=None,
        _mission=mission,
        _goal_epoch=4,
        _cancel_goal=Client(),
        _brake=Client(),
        _log_failed_brake=lambda _future: None,
        _nav_cancel_timeout_s=0.1,
        _event=lambda *_args, **_kwargs: None,
    )
    response = SimpleNamespace(ok=False, error="")

    RouteExecutorNode._cancel(fake, None, response)

    assert response.ok and response.error == ""
    assert mission.phase is RoutePhase.CANCELLED
    assert mission.pause_reason == "cancelled"


def test_route_input_accepts_hard_role_but_rejects_unknown_roles():
    values = ([0.0], [0.0], [nan], [""])
    assert validate_inputs(*values, ["hard"]) == ""
    assert "normal or hard" in validate_inputs(*values, ["home"])


def test_dispatch_start_skips_passed_synthetics_without_moving_backwards():
    route = prepare(
        [point(0, 0), point(10, 1)], loop=False, input_count=2,
        spacing_m=2, chunk_span_m=100, chunk_max_waypoints=20,
    )
    resolved = resolve_dispatch_start(
        route, 1, robot_xy=(5.0, 0.0), reached_tolerance_m=1.2,
        synthetic_segment_tolerance_m=5.0,
    )
    assert resolved.index == 3
    assert resolved.skipped_synthetic > 0
    assert resolved.index > 1


def test_dispatch_start_never_skips_an_action_checkpoint():
    points = [point(0, 0), point(10, 1)]
    points[1] = RouteWaypoint(
        **{**points[1].__dict__, "action_json": '[{"type":"brake_hold","duration_s":1}]'}
    )
    route = prepare(
        points, loop=False, input_count=2, spacing_m=2,
        chunk_span_m=100, chunk_max_waypoints=20,
    )
    resolved = resolve_dispatch_start(
        route, len(route.waypoints) - 1, robot_xy=(10.0, 0.0),
        reached_tolerance_m=1.2, synthetic_segment_tolerance_m=5.0,
    )
    assert resolved.index == len(route.waypoints) - 1
    assert resolved.skipped_reached == 0


def test_dispatch_start_keeps_reached_checkpoint_as_an_observable_boundary():
    route = prepare(
        [point(0, 0), point(10, 1), point(20, 2)], loop=False,
        input_count=3, spacing_m=0, chunk_span_m=100, chunk_max_waypoints=5,
    )
    resolved = resolve_dispatch_start(
        route, 0, robot_xy=(0.5, 0.0), reached_tolerance_m=1.2,
        synthetic_segment_tolerance_m=5.0,
    )
    assert resolved.index == 0
    assert resolved.skipped_reached == 0


def test_dispatch_start_preserves_exit_checkpoint_event_for_patrol_return():
    loop_points = [
        RouteWaypoint(0, 0, 0.0, 0, map_x=0.0, map_y=0.0),
        RouteWaypoint(0, 0, 0.0, 1, map_x=10.0, map_y=0.0),
        RouteWaypoint(0, 0, 0.0, 2, map_x=20.0, map_y=0.0),
    ]
    route = prepare(
        loop_points, loop=True, input_count=3, spacing_m=0.0,
        chunk_span_m=100.0, chunk_max_waypoints=5,
    )
    spec = PatrolMissionSpec(
        home=loop_points[0],
        loop=PatrolRoute(tuple(loop_points), ("", "", "")),
        depart=PatrolRoute((), ()),
        returning=PatrolRoute((loop_points[1],), ("",)),
        depart_entry_loop_index=0,
        leg_spacing_m=2.0,
        chunk_span_m=100.0,
        chunk_max_waypoints=5,
    )
    patrol = PatrolMachine(spec, "patrol")
    assert patrol.start(at_home=False) is PatrolPhase.JOIN_LOOP
    assert patrol.goal_succeeded() is PatrolPhase.PATROL
    assert patrol.latch_battery_return(at_home=False).phase is PatrolPhase.EXIT_LOOP
    assert patrol.state.return_exit.loop_index == 1

    resolved = resolve_dispatch_start(
        route, 1, robot_xy=(10.0, 0.0), reached_tolerance_m=1.2,
        synthetic_segment_tolerance_m=5.0,
    )

    # A reached original checkpoint is still dispatched so its typed event can
    # advance EXIT_LOOP -> RETURN_HOME; pruning it would leave patrol stuck.
    assert resolved.index == 1
    assert patrol.goal_succeeded(route.waypoints[resolved.index].input_index) is PatrolPhase.RETURN_HOME


def test_yaw_policies_are_compared_and_explicit_yaw_is_unchanged():
    points = (
        RouteWaypoint(0, 0, 0.0, 0, map_x=0.0, map_y=0.0),
        RouteWaypoint(0, 0, 90.0, 1, map_x=10.0, map_y=0.0),
    )
    current = dispatch_yaws(points, approach_xy=(0.0, -10.0))
    prepared_without_mutation = [point.yaw_deg for point in points]
    terminal_incoming = dispatch_yaws(points)
    assert current == [90.0, 0.0]
    assert prepared_without_mutation == [0.0, 90.0]
    assert terminal_incoming == [0.0, 0.0]
    explicit = tuple(
        point if index == 0 else RouteWaypoint(**{**point.__dict__, "yaw_explicit": True})
        for index, point in enumerate(points)
    )
    assert dispatch_yaws(explicit, approach_xy=(0.0, -10.0)) == [90.0, 90.0]


def test_supported_actions_are_accepted_but_unknown_actions_are_rejected():
    base = ([1.0], [2.0], [0.0])
    assert validate_inputs(*base, ['[{"type":"brake_hold","duration_s":1}]'], ["normal"]) == ""
    assert "unsupported" in validate_inputs(*base, ['[{"type":"camera"}]'], ["normal"])
