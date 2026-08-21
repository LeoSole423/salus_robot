from pathlib import Path


ROOT = Path(__file__).parents[1]


def test_patrol_coordinator_exposes_legacy_endpoints_without_nav2_or_velocity_clients():
    source = (ROOT / "salus_navigation" / "patrol_mission_coordinator.py").read_text()
    for endpoint in (
        "/route_executor/set_patrol_mission_ll",
        "/route_executor/cancel_patrol_mission",
        "/route_executor/get_patrol_mission_state",
        "/route_executor/request_return_home",
    ):
        assert endpoint in source
    assert "NavigateToPose" not in source
    assert "cmd_vel" not in source


def test_route_executor_emits_an_unambiguous_checkpoint_event():
    source = (ROOT / "salus_navigation" / "route_executor_node.py").read_text()
    assert '"ROUTE_CHECKPOINT_REACHED"' in source
    assert "input_index=point.input_index" in source
    assert "mission_id=self._mission.mission_id" in source
