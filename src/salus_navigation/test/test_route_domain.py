from math import nan
from salus_navigation.route_model import RouteWaypoint
from salus_navigation.route_preparation import expand, prepare, resolve_yaws
from salus_navigation.route_anchor import select_anchor
from salus_navigation.route_chunker import build_chunk, next_start

def point(x, index): return RouteWaypoint(0, 0, nan, index, map_x=x, map_y=0)

def test_expansion_marks_synthetic_points_and_resolves_yaw():
    route = resolve_yaws(expand([point(0, 0), point(10, 1)], 2.0, False), False)
    assert len(route) == 6 and route[1].key is False and route[0].yaw_deg == 0.0
    assert route[-1].key is True and route[-1].input_index == 1

def test_open_anchor_never_moves_backwards():
    route = prepare([point(0,0), point(10,1), point(20,2)], loop=False, input_count=3, spacing_m=0, chunk_span_m=20, chunk_max_waypoints=3)
    assert select_anchor(route, 9.0, 0.2) >= 1

def test_loop_chunk_does_not_contain_a_complete_circuit():
    route = prepare([point(0,0),point(2,1),point(4,2),point(6,3)], loop=True,input_count=4,spacing_m=0,chunk_span_m=100,chunk_max_waypoints=10)
    chunk=build_chunk(route,0); assert len(chunk.waypoints)==3 and next_start(route,chunk)==3


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
