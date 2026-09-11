from salus_navigation.route_geometry import path_geometry_metrics
from salus_navigation.route_chunker import build_chunk, next_start
from salus_navigation.route_preparation import prepare
from salus_navigation.route_model import RouteWaypoint


def waypoint(x, y, index):
    return RouteWaypoint(0.0, 0.0, float(index * 45), index, map_x=x, map_y=y)


def test_geometry_metrics_measure_length_detour_and_deviation():
    metrics = path_geometry_metrics(
        [(0.0, 0.0), (1.0, 1.0), (2.0, 0.0)],
        [(0.0, 0.0), (2.0, 0.0)],
    )
    assert metrics.length_m > 2.0
    assert metrics.detour_ratio > 1.0
    assert metrics.max_deviation_m == 1.0
    assert metrics.self_intersections == 0


def test_geometry_metrics_count_non_adjacent_self_intersection():
    metrics = path_geometry_metrics(
        [(0.0, 0.0), (2.0, 2.0), (0.0, 2.0), (2.0, 0.0)],
        [(0.0, 0.0), (2.0, 2.0)],
    )
    assert metrics.self_intersections == 1


def test_geometry_metrics_accept_a_wide_loop_without_false_intersection():
    metrics = path_geometry_metrics(
        [(0.0, 0.0), (8.0, 0.0), (8.0, 8.0), (0.0, 8.0), (0.0, 0.0)],
        [(0.0, 0.0), (8.0, 0.0), (8.0, 8.0), (0.0, 8.0)],
    )
    assert metrics.self_intersections == 0


def test_wide_ninety_degree_route_has_finite_forward_chunks():
    route = prepare(
        [waypoint(0.0, 0.0, 0), waypoint(8.0, 0.0, 1), waypoint(8.0, 8.0, 2)],
        loop=False, input_count=3, spacing_m=0.0,
        chunk_span_m=120.0, chunk_max_waypoints=5,
    )

    first = build_chunk(route, 0)
    second = build_chunk(route, next_start(route, first))

    assert [point.input_index for point in first.waypoints] == [0, 1]
    assert [point.input_index for point in second.waypoints] == [2]
    assert path_geometry_metrics(
        [(point.map_x, point.map_y) for point in first.waypoints],
        [(0.0, 0.0), (8.0, 0.0)],
    ).self_intersections == 0


def test_wide_loop_dispatches_progressively_without_a_second_full_circuit():
    route = prepare(
        [waypoint(0.0, 0.0, 0), waypoint(8.0, 0.0, 1),
         waypoint(8.0, 8.0, 2), waypoint(0.0, 8.0, 3)],
        loop=True, input_count=4, spacing_m=0.0,
        chunk_span_m=120.0, chunk_max_waypoints=5,
    )

    starts = []
    start = 0
    for _ in range(4):
        chunk = build_chunk(route, start)
        starts.append(start)
        start = next_start(route, chunk)

    assert starts == [0, 2, 0, 2]
    assert all(len(build_chunk(route, start).waypoints) <= 2 for start in starts)


def test_intersection_metric_exposes_a_problematic_route_instead_of_hiding_it():
    metrics = path_geometry_metrics(
        [(0.0, 0.0), (8.0, 8.0), (0.0, 8.0), (8.0, 0.0)],
        [(0.0, 0.0), (8.0, 8.0), (0.0, 8.0), (8.0, 0.0)],
    )

    assert metrics.self_intersections == 1
    assert metrics.detour_ratio > 1.0


def test_compact_dubins_incompatible_fixture_is_measured_without_a_new_heuristic():
    route = prepare(
        [waypoint(0.0, 0.0, 0), waypoint(1.0, 0.0, 1),
         waypoint(1.0, 1.0, 2), waypoint(0.0, 1.0, 3)],
        loop=False, input_count=4, spacing_m=0.0,
        chunk_span_m=120.0, chunk_max_waypoints=5,
    )
    chunk = build_chunk(route, 0)
    metrics = path_geometry_metrics(
        [(point.map_x, point.map_y) for point in chunk.waypoints],
        [(point.map_x, point.map_y) for point in chunk.waypoints],
    )

    assert len(chunk.waypoints) == 2
    assert metrics.length_m == 1.0
    assert metrics.self_intersections == 0
