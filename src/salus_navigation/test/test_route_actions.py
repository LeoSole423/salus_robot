import json

from salus_navigation.route_actions import ActionExecution, ActionState, parse_actions


def test_parse_supported_actions_and_canonicalize() -> None:
    actions, canonical, error = parse_actions(
        '[{"type":"brake_hold","duration_s":5,"brake_pct":110},'
        '{"type":"set_navigation_profile","profile":"RURAL"}]', 3)
    assert not error and [item.kind for item in actions] == ["brake_hold", "set_navigation_profile"]
    assert actions[0].brake_pct == 100 and actions[1].profile == "rural"
    assert json.loads(canonical)[1]["profile"] == "rural"


def test_parser_rejects_unknown_invalid_and_unbounded_actions() -> None:
    assert parse_actions("{}", 0)[2]
    assert parse_actions('[{"type":"unknown"}]', 0)[2]
    assert parse_actions('[{"type":"brake_hold","duration_s":0}]', 0)[2]
    assert parse_actions('[{"type":"set_navigation_profile","profile":"forest"}]', 0)[2]


def test_action_execution_is_explicit_and_cancelable() -> None:
    actions, _, _ = parse_actions('[{"type":"brake_hold","duration_s":5}]', 2)
    execution = ActionExecution(2, actions)
    assert execution.start(10.0).kind == "brake_hold"
    assert execution.state == ActionState.RUNNING and execution.remaining_s(12.0) == 3.0
    execution.cancel("manual")
    assert execution.state == ActionState.CANCELLED and execution.error == "manual"
