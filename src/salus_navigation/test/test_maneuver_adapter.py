"""Exercise actual service adapter decisions without a ROS executor or hardware."""

from types import SimpleNamespace as NS

from salus_interfaces.msg import PathHealth
from salus_interfaces.srv import EvaluatePathHealth
from salus_navigation.maneuver_guard import ManeuverConfig, ManeuverWait
from salus_navigation.path_health import CostmapView, PathHealthNode, PathHealthPolicy
from test_path_health import make_path
from test_maneuver_guard import arc


def node_fixture(monkeypatch):
    node = object.__new__(PathHealthNode)
    node._policy = PathHealthPolicy()
    node._maneuver_config = ManeuverConfig(wait_s=5)
    node._maneuver_wait = ManeuverWait(node._maneuver_config)
    node._maneuver_key = None
    node._maneuver_progress_m = 0.0
    node._costmap_frame = 'map'
    node._costmap = CostmapView(10., 20, 20, -100., -100., 10., (0,) * 400)
    node._keepout_revision = None
    stamp = PathHealth().stamp
    stamp.sec = 10
    node.get_clock = lambda: NS(now=lambda: NS(nanoseconds=10_000_000_000, to_msg=lambda: stamp))
    node.get_parameter = lambda name: NS(value='base_footprint' if name == 'base_frame' else 1.5)
    node._tf_buffer = NS(lookup_transform_core=lambda *_args: NS(
        header=NS(stamp=NS(sec=10, nanosec=0)),
        transform=NS(translation=NS(x=0., y=0.), rotation=NS(x=0., y=0., z=0., w=1.))))
    published = []
    node._pub = NS(publish=published.append)
    clock = [0.]
    monkeypatch.setattr('salus_navigation.path_health.time.monotonic', lambda: clock[0])
    return node, published, clock


def evaluate(node, points, context=0):
    request = EvaluatePathHealth.Request()
    request.path = make_path(points)
    request.context = context
    return node._on_evaluate(request, EvaluatePathHealth.Response()).health


def test_candidate_veto_holds_old_path_until_accepted_path_becomes_active(monkeypatch):
    node, published, clock = node_fixture(monkeypatch)
    old = ((0., 0.), (15., 0.))
    assert evaluate(node, old).state == PathHealth.KEEP_PATH
    assert evaluate(node, arc(180), 1).state == PathHealth.STOP_AND_WAIT
    assert published[-1].reason == 'maneuver:WAITING_REPLAN:PLANNER_U_TURN'
    clock[0] = 1
    good = ((0., 0.), (12., 1.))
    assert evaluate(node, good, 1).state == PathHealth.KEEP_PATH
    assert published[-1].state == PathHealth.STOP_AND_WAIT
    assert evaluate(node, old).state == PathHealth.STOP_AND_WAIT
    assert evaluate(node, good).state == PathHealth.KEEP_PATH
    assert published[-1].reason == 'path_healthy'


def test_timer_expires_without_requests_and_latches_until_explicit_reset(monkeypatch):
    node, published, clock = node_fixture(monkeypatch)
    evaluate(node, arc(360), 1)
    clock[0] = 5
    node._tick_maneuver_wait()
    assert published[-1].reason == 'maneuver:NEEDS_OPERATOR:PLANNER_LOOP'
    assert evaluate(node, ((0., 0.), (15., 0.)), 1).state == PathHealth.STOP_AND_WAIT
    request = EvaluatePathHealth.Request(context=EvaluatePathHealth.Request.RESET)
    node._on_evaluate(request, EvaluatePathHealth.Response())
    assert published[-1].state == PathHealth.STOP_AND_WAIT
    assert evaluate(node, ((0., 0.), (15., 0.)), 1).state == PathHealth.KEEP_PATH


def test_stale_data_during_wait_cannot_clear_or_renew_deadline(monkeypatch):
    node, published, clock = node_fixture(monkeypatch)
    evaluate(node, arc(180), 1)
    node._costmap = None
    clock[0] = 4
    assert evaluate(node, ((0., 0.), (15., 0.)), 1).state == PathHealth.STOP_AND_WAIT
    assert published[-1].wait_remaining_s == 1
    clock[0] = 5
    node._tick_maneuver_wait()
    assert 'NEEDS_OPERATOR' in published[-1].reason


def test_active_path_becoming_inadmissible_stops_without_waiting_for_a_replan(monkeypatch):
    node, published, clock = node_fixture(monkeypatch)
    result = evaluate(node, arc(180))
    assert result.state == PathHealth.STOP_AND_WAIT
    assert published[-1].reason == 'maneuver:WAITING_REPLAN:PLANNER_U_TURN'
