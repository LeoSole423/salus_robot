"""Keep the actual rejected plan and admission cause reproducible offline."""
import csv

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from salus_interfaces.msg import PathHealth
from salus_evaluation.artifacts import write_artifacts
from salus_evaluation.evaluation_runner import EvaluationRunner


def test_collector_persists_complete_plan_and_health_with_source_stamps(tmp_path, monkeypatch):
    monkeypatch.setattr("salus_evaluation.evaluation_runner._now_s", lambda _: 11.0)
    collector = object.__new__(EvaluationRunner)
    collector.plans = []
    collector.plan_samples = []
    collector.path_health_samples = []
    collector.mode = 'run'
    path = Path()
    path.header.frame_id = 'map'
    path.header.stamp.sec = 10
    path.header.stamp.nanosec = 500_000_000
    for x, y in ((0., 0.), (1., 0.)):
        pose = PoseStamped()
        pose.pose.position.x, pose.pose.position.y = x, y
        pose.pose.orientation.w = 1.
        path.poses.append(pose)
    collector._plan(path)
    collector._plan(path)
    health = PathHealth(state=2, reason='maneuver:WAITING_REPLAN:PLANNER_U_TURN',
                        wait_remaining_s=12.)
    health.stamp = path.header.stamp
    collector._path_health(health)
    write_artifacts(tmp_path, {}, {}, {
        'plans': collector.plan_samples,
        'path_health': collector.path_health_samples,
    })
    rows = list(csv.DictReader((tmp_path / 'plans.csv').open()))
    assert len(rows) == 4
    assert [row['plan_index'] for row in rows] == ['0', '0', '1', '1']
    assert rows[-1]['x_m'] == '1.0' and rows[-1]['stamp_s'] == '10.5'
    assert rows[-1]['frame_id'] == 'map'
    assert rows[-1]['receive_stamp_s'] == '11.0'
    states = list(csv.DictReader((tmp_path / 'path_health.csv').open()))
    assert states[0]['reason'] == health.reason
    assert states[0]['wait_remaining_s'] == '12.0'


def test_ordered_service_rejection_is_terminal_failure():
    from types import SimpleNamespace
    from action_msgs.msg import GoalStatus
    collector = object.__new__(EvaluationRunner)
    collector.terminal_status = None
    collector.get_logger = lambda: SimpleNamespace(error=lambda message: None)
    future = SimpleNamespace(result=lambda: SimpleNamespace(ok=False, error='bad route'))
    collector._ordered_goal_response(future)
    assert collector.terminal_status == GoalStatus.STATUS_ABORTED


def test_repeated_terminal_telemetry_does_not_restart_capture_window():
    from types import SimpleNamespace
    from action_msgs.msg import GoalStatus
    collector = object.__new__(EvaluationRunner)
    collector.goal_event_baseline = 1
    collector.terminal_status = GoalStatus.STATUS_SUCCEEDED
    collector.terminal_received_s = 5.
    collector.success_s = 5.
    collector.get_clock = lambda: SimpleNamespace(
        now=lambda: SimpleNamespace(nanoseconds=9_000_000_000))
    message = SimpleNamespace(nav_result_status=GoalStatus.STATUS_SUCCEEDED,
                              nav_result_event_id=2)
    collector._telemetry(message)
    assert collector.terminal_received_s == 5.
