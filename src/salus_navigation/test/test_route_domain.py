from math import nan
from salus_navigation.route_model import RouteWaypoint
from salus_navigation.route_preparation import expand, prepare, resolve_yaws
from salus_navigation.route_anchor import select_anchor
from salus_navigation.route_chunker import build_chunk, next_start

def point(x, index): return RouteWaypoint(0, 0, nan, index, map_x=x, map_y=0)

def test_expansion_marks_synthetic_points_and_resolves_yaw():
    route = resolve_yaws(expand([point(0, 0), point(10, 1)], 2.0, False), False)
    assert len(route) == 5 and route[1].key is False and route[0].yaw_deg == 0.0

def test_open_anchor_never_moves_backwards():
    route = prepare([point(0,0), point(10,1), point(20,2)], loop=False, input_count=3, spacing_m=0, chunk_span_m=20, chunk_max_waypoints=3)
    assert select_anchor(route, 9.0, 0.2) >= 1

def test_loop_chunk_does_not_contain_a_complete_circuit():
    route = prepare([point(0,0),point(2,1),point(4,2),point(6,3)], loop=True,input_count=4,spacing_m=0,chunk_span_m=100,chunk_max_waypoints=10)
    chunk=build_chunk(route,0); assert len(chunk.waypoints)==3 and next_start(route,chunk)==3
