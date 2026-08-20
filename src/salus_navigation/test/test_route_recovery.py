from salus_interfaces.msg import PathHealth
from salus_navigation.route_model import PreparedRoute, RouteWaypoint
from salus_navigation.route_recovery import (
    BlockedRecoveryPolicy, RecoveryAction, RecoveryObservation, RecoveryState,
    resolve_forward_reanchor,
)
from salus_navigation.nav_command_server import diagnostic_level
from diagnostic_msgs.msg import DiagnosticStatus


def observation(now, **changes):
    values = {"now_s": now}
    values.update(changes)
    return RecoveryObservation(**values)


def route(loop=False):
    points = tuple(RouteWaypoint(0, 0, 0, index, map_x=float(index * 5), map_y=0.0)
                   for index in range(5))
    return PreparedRoute(points, loop, 5, 0.0, 20.0, 5)


def test_transient_collision_never_consumes_retry():
    policy = BlockedRecoveryPolicy(persistence_s=1.5)
    assert policy.observe(observation(0.0, collision_stop=True)).state == RecoveryState.PENDING
    decision = policy.observe(observation(1.0, collision_stop=False))
    assert decision.state == RecoveryState.CLEAR and decision.attempt == 0


def test_persistent_block_waits_then_starts_one_retry():
    policy = BlockedRecoveryPolicy(persistence_s=1.0, retry_wait_s=5.0)
    policy.observe(observation(0.0, nav_failure_code="COSTMAP_CLEAR_TIMEOUT"))
    waiting = policy.observe(observation(1.1, nav_failure_code="COSTMAP_CLEAR_TIMEOUT"))
    assert waiting.state == RecoveryState.WAITING_RETRY
    assert waiting.action == RecoveryAction.CANCEL_AND_BRAKE
    assert policy.observe(observation(6.0)).state == RecoveryState.WAITING_RETRY
    retry = policy.observe(observation(6.1))
    assert retry.state == RecoveryState.RECOVERING
    assert retry.action == RecoveryAction.BEGIN_RETRY and retry.attempt == 1


def test_stale_data_waits_without_cleaning_or_consuming_attempt():
    policy = BlockedRecoveryPolicy()
    stale = policy.observe(observation(2.0, tf_fresh=False))
    assert stale.state == RecoveryState.WAITING_DATA
    assert stale.action == RecoveryAction.NONE and stale.attempt == 0
    recovered = policy.observe(observation(3.0))
    assert recovered.state == RecoveryState.CLEAR
    assert recovered.action == RecoveryAction.RESUME


def test_stale_data_preserves_pending_retry():
    policy = BlockedRecoveryPolicy(persistence_s=0.0, retry_wait_s=5.0)
    policy.observe(observation(0.0, collision_stop=True))
    policy.observe(observation(0.1, collision_stop=True))
    stale = policy.observe(observation(1.0, costmap_fresh=False))
    assert stale.state == RecoveryState.WAITING_DATA
    resumed = policy.observe(observation(2.0))
    assert resumed.state == RecoveryState.WAITING_RETRY
    assert abs(resumed.wait_remaining_s - 3.1) < 1e-6


def test_attempt_limit_requires_operator_without_silent_reset():
    policy = BlockedRecoveryPolicy(persistence_s=0.0, retry_wait_s=0.0, max_attempts=2)
    policy.observe(observation(0.0, collision_stop=True))
    policy.observe(observation(0.1, collision_stop=True))
    assert policy.observe(observation(0.2)).action == RecoveryAction.BEGIN_RETRY
    policy.finish_retry(now_s=0.3, accepted=False)
    assert policy.observe(observation(0.4)).attempt == 2
    final = policy.finish_retry(now_s=0.5, accepted=False)
    assert final.state == RecoveryState.NEEDS_OPERATOR
    assert policy.observe(observation(100.0)).state == RecoveryState.NEEDS_OPERATOR


def test_path_stop_waits_for_data_but_replan_does_not_own_retry():
    policy = BlockedRecoveryPolicy()
    stopped = policy.observe(observation(
        0.0, path_state=PathHealth.STOP_AND_WAIT, costmap_fresh=False))
    assert stopped.state == RecoveryState.WAITING_DATA
    replanning = policy.observe(observation(1.0, path_state=PathHealth.REPLAN))
    assert replanning.state == RecoveryState.CLEAR


def test_open_reanchor_never_moves_backwards():
    resolution = resolve_forward_reanchor(
        route(), current_index=2, robot_x=5.1, robot_y=0.0, tolerance_m=8.0)
    assert resolution.resolved_index == 2 and not resolution.reanchored


def test_reanchor_moves_forward_and_loop_does_not_wrap_early():
    forward = resolve_forward_reanchor(
        route(), current_index=1, robot_x=14.8, robot_y=0.0, tolerance_m=8.0)
    assert forward.resolved_index == 3 and forward.reanchored
    loop = resolve_forward_reanchor(
        route(loop=True), current_index=3, robot_x=0.1, robot_y=0.0, tolerance_m=8.0)
    assert loop.resolved_index == 3 and loop.reason == "no_forward_match"
def test_recovery_event_severity_is_normalized_for_humble() -> None:
    assert diagnostic_level(DiagnosticStatus.WARN) == 1
    assert isinstance(diagnostic_level(DiagnosticStatus.ERROR), int)
