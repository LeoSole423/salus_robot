from math import nan
from types import SimpleNamespace
from salus_navigation.route_model import RouteWaypoint
from salus_navigation.route_preparation import dispatch_yaws, expand, prepare, resolve_yaws
from salus_navigation.route_preparation import validate_inputs
from salus_navigation.route_anchor import select_anchor
from salus_navigation.route_chunker import build_chunk, next_start
from salus_navigation.route_progress import project
from salus_navigation.route_executor_node import RouteExecutorNode, chunk_goal_request

def point(x, index): return RouteWaypoint(0, 0, nan, index, map_x=x, map_y=0)

def test_expansion_marks_synthetic_points_and_resolves_yaw():
    route = resolve_yaws(expand([point(0, 0), point(10, 1)], 2.0, False), False)
    assert len(route) == 6 and route[1].key is False and route[0].yaw_deg == 0.0
    assert route[-1].key is True and route[-1].input_index == 1


def test_open_route_final_automatic_yaw_follows_its_incoming_leg():
    route = resolve_yaws([
        RouteWaypoint(0, 0, nan, 0, map_x=0, map_y=0),
        RouteWaypoint(0, 0, nan, 1, map_x=0, map_y=10),
    ], False)

    assert [point.yaw_deg for point in route] == [90.0, 90.0]


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


def test_chunk_request_uses_incoming_yaw_only_for_an_automatic_terminal_checkpoint():
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

    assert [point.yaw_deg for point in automatic_chunk.waypoints] == [0.0, 90.0]
    assert list(chunk_goal_request(automatic_chunk, automatic_route).yaws_deg) == [0.0, 0.0]

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

    assert list(chunk_goal_request(explicit_chunk, explicit_route).yaws_deg) == [0.0, 90.0]

def test_open_anchor_never_moves_backwards():
    route = prepare([point(0,0), point(10,1), point(20,2)], loop=False, input_count=3, spacing_m=0, chunk_span_m=20, chunk_max_waypoints=3)
    assert select_anchor(route, 9.0, 0.2) >= 1

def test_loop_chunk_does_not_contain_a_complete_circuit():
    route = prepare([point(0,0),point(2,1),point(4,2),point(6,3)], loop=True,input_count=4,spacing_m=0,chunk_span_m=100,chunk_max_waypoints=10)
    chunk=build_chunk(route,0); assert len(chunk.waypoints)==2 and next_start(route,chunk)==2


def test_chunk_ends_at_next_original_checkpoint_like_legacy():
    route = prepare(
        [point(0, 0), point(3, 1), point(6, 2), point(9, 3), point(12, 4)],
        loop=False, input_count=5, spacing_m=35,
        chunk_span_m=120, chunk_max_waypoints=5,
    )

    chunk = build_chunk(route, 0)

    assert [waypoint.input_index for waypoint in chunk.waypoints] == [0, 1]
    assert chunk.checkpoint_offsets == (0, 1)
    assert next_start(route, chunk) == 2


def test_chunk_soft_limits_never_promote_synthetic_point_to_boundary():
    route = prepare(
        [point(0, 0), point(10, 1), point(20, 2)], loop=False,
        input_count=3, spacing_m=2, chunk_span_m=3, chunk_max_waypoints=2,
    )
    chunk = build_chunk(route, 0)

    assert chunk.waypoints[-1].key is True
    assert chunk.waypoints[-1].input_index == 1
    assert len(chunk.waypoints) > route.chunk_max_waypoints
    assert chunk.checkpoint_offsets == (0, len(chunk.waypoints) - 1)


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
    chunk = build_chunk(route, 0)

    dispatched = [chunk.waypoints[index] for index in chunk.checkpoint_offsets]
    assert [waypoint.input_index for waypoint in dispatched] == [0, 1]
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
    route = prepare(
        [point(0, 0), point(10, 1), point(20, 2)], loop=False,
        input_count=3, spacing_m=0, chunk_span_m=100, chunk_max_waypoints=3,
    )
    chunk = build_chunk(route, 0)

    progress = project(chunk, 5.0, 2.0)

    assert progress.expanded_index == 0
    assert progress.checkpoint_index == 0
    assert progress.ratio == 0.5
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

    chunk = build_chunk(route, 0)

    assert chunk.waypoints[-1].key
    assert chunk.waypoints[-1].input_index == 1
    assert next_start(route, chunk) < len(route.waypoints)


def test_chunk_request_sends_synthetic_geometry_but_counts_only_checkpoint_boundary():
    route = prepare(
        [point(0, 0), point(12, 1)], loop=False, input_count=2,
        spacing_m=2, chunk_span_m=100, chunk_max_waypoints=20,
    )
    chunk = build_chunk(route, 0)

    request = chunk_goal_request(chunk, route)

    assert list(request.lats) == [point.lat for point in chunk.waypoints]
    assert len(request.lats) > len(chunk.checkpoint_offsets)
    assert chunk.checkpoint_offsets == (0, len(chunk.waypoints) - 1)
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
    chunk = build_chunk(route, 0)
    events = []
    advanced = []
    fake = SimpleNamespace(
        _chunk=chunk,
        _target_offset=chunk.checkpoint_offsets[-1],
        _mission=SimpleNamespace(reached=0, mission_id="mission", chunk_id=0, loop_iteration=0),
        _event=lambda *args, **kwargs: events.append((args, kwargs)),
        _start_actions=lambda *_args: None,
        _advance=lambda: advanced.append(True),
    )

    RouteExecutorNode._complete_current_chunk(fake, "nav2_succeeded")

    assert fake._mission.reached == 2
    assert [event[1]["input_index"] for event in events] == [0, 1]
    assert len(events) == len(chunk.checkpoint_offsets)
    assert len(events) < len(chunk.waypoints)
    assert advanced == [True]


def test_supported_actions_are_accepted_but_unknown_actions_are_rejected():
    base = ([1.0], [2.0], [0.0])
    assert validate_inputs(*base, ['[{"type":"brake_hold","duration_s":1}]'], ["normal"]) == ""
    assert "unsupported" in validate_inputs(*base, ['[{"type":"camera"}]'], ["normal"])
