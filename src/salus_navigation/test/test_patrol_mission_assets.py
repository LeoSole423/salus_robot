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


def test_patrol_battery_inputs_are_configurable_and_do_not_command_motion():
    source = (ROOT / "salus_navigation" / "patrol_mission_coordinator.py").read_text()
    for parameter, topic in (
        ("battery_guard_topic", "/battery_mission_guard"),
        ("battery_state_topic", "/battery_state"),
        ("battery_guard_timeout_s", "3.0"),
        ("low_battery_threshold_pct", "25.0"),
    ):
        assert parameter in source
        assert topic in source
    callbacks = source[source.index("    def _on_battery_guard"):source.index("    def _set")]
    assert "cmd_vel" not in callbacks
    assert "NavigateToPose" not in callbacks


def test_route_cancel_waits_for_terminal_nav2_before_returning_ok():
    source = (ROOT / "salus_navigation" / "route_executor_node.py").read_text()
    cancel = source[source.index("    def _cancel("):source.index("    def _set_profile")]
    assert '"nav_cancel_timeout_s", 15.0' in source
    assert "self._nav_cancel_group = ReentrantCallbackGroup()" in source
    assert "callback_group=self._nav_cancel_group" in source
    assert "terminal = threading.Event()" in cancel
    assert "terminal.wait(self._nav_cancel_timeout_s)" in cancel
    assert '"ROUTE_CANCEL_TIMEOUT"' in cancel
    assert '"ROUTE_CANCEL_FAILED"' in cancel
    assert "response.ok, response.error = True, \"\"" in cancel


def test_profile_forwarder_outlives_coordinator_transaction_contract():
    root = ROOT / "salus_navigation"
    route = (root / "route_executor_node.py").read_text()
    coordinator = (root / "navigation_profile_coordinator.py").read_text()
    smoke = (ROOT.parents[1] / "tools" / "smoke_navigation_profiles.py").read_text()
    assert '"transaction_timeout_s", 21.0' in coordinator
    assert '"profile_transaction_timeout_s", 24.0' in route
    assert '"profile_coordinator_discovery_timeout_s", 5.0' in route
    assert "28.0" in smoke
    assert "component_deadline = min(" in coordinator


def test_route_goal_results_are_correlated_to_the_current_request_boundary():
    route = (ROOT / "salus_navigation" / "route_executor_node.py").read_text()
    nav = (ROOT / "salus_navigation" / "nav_command_server.py").read_text()
    interface = (
        ROOT.parents[1] / "src" / "salus_interfaces" / "srv" / "SetNavGoalLL.srv"
    ).read_text()

    assert "uint32 goal_event_id" in interface
    assert "response.goal_event_id = self._event(" in nav
    assert "self._goal_request_pending = True" in route
    assert "self._goal_result_event_floor = int(result.goal_event_id)" in route
    assert "terminal_nav_result_is_current(" in route
