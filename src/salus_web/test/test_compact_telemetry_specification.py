import json
from pathlib import Path


FIXTURE = Path(__file__).parent / "fixtures/cockpit_protocol/compact_telemetry.json"


def _specification():
    return json.loads(FIXTURE.read_text(encoding="utf-8"))


def test_profiles_are_explicit_and_compact_is_the_default() -> None:
    specification = _specification()
    assert specification["schema_version"] == 1
    assert specification["default_profile"] == "compact"
    assert set(specification["profiles"]) == {"compact", "full"}
    assert specification["profiles"]["compact"]["max_state_hz"] == 2.0
    assert specification["profiles"]["compact"]["coalescing"] == "latest_wins"
    assert specification["profiles"]["full"]["suppressed_delta_ops"] == []


def test_compact_profile_never_delays_control_or_mission_transitions() -> None:
    specification = _specification()
    immediate = set(specification["immediate_transition_fields"])
    assert {"control_locked", "manual_enabled", "collision_stop_active"} <= immediate
    assert {"nav_result_event_id", "route_mission.state", "patrol_mission.phase"} <= immediate
    assert {"home_phase", "battery_return_home_recommended"} <= immediate
    assert {"ack", "nav_event", "nav_snapshot"} <= set(
        specification["never_rate_limited_ops"]
    )


def test_scan_preview_is_diagnostic_and_cannot_replace_scan_clean() -> None:
    preview = _specification()["scan_preview"]
    assert preview["source_topic"] == "/scan_clean"
    assert preview["output_topic"] == "/scan_preview"
    assert preview["publish_hz"] == 2.0
    assert preview["beam_stride"] == 4
    assert preview["output_range_max_m"] == 12.0
    assert preview["qos"] == {
        "reliability": "best_effort",
        "durability": "volatile",
        "depth": 1,
    }
    assert preview["replaceable"] is True
    assert preview["include_intensities"] is False
    assert {"nav2", "collision_monitor", "nav_snapshot_server"} <= set(
        preview["forbidden_consumers"]
    )
