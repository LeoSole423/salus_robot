import importlib.util
from pathlib import Path
import sys


TOOL = Path(__file__).parents[3] / "tools" / "observe_hardware_contracts.py"
SPEC = importlib.util.spec_from_file_location("observe_hardware_contracts", TOOL)
assert SPEC and SPEC.loader
observer = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = observer
SPEC.loader.exec_module(observer)


def test_topic_list_preserves_types_and_unknown_entries() -> None:
    assert observer.parse_topic_list("/imu/data [sensor_msgs/msg/Imu]\n/raw\n") == {
        "/imu/data": "sensor_msgs/msg/Imu",
        "/raw": None,
    }


def test_topic_info_extracts_counts_and_qos() -> None:
    parsed = observer.parse_topic_info("""Type: sensor_msgs/msg/Imu
Publisher count: 1
Node name: imu_driver
QoS profile:
  Reliability: RELIABLE
  Durability: VOLATILE
Subscription count: 1
Node name: ekf
QoS profile:
  Reliability: BEST_EFFORT
""")
    assert parsed["publisher_count"] == 1
    assert parsed["subscriber_count"] == 1
    assert parsed["publishers"][0]["qos"]["reliability"] == "RELIABLE"
    assert parsed["subscribers"][0]["qos"]["reliability"] == "BEST_EFFORT"


def test_sanitize_sample_excludes_position_and_payload() -> None:
    sanitized = observer.sanitize_sample("""header:
  stamp:
    sec: 12
    nanosec: 34
  frame_id: gps_link
latitude: -31.4
longitude: -64.2
status:
  status: 0
  service: 1
position_covariance:
- 1.0
- 0.0
rtcm: secret-bytes
""")
    assert sanitized["headers"]["header.stamp.sec"] == 12
    assert sanitized["statuses"]["status.status"] == 0
    assert sanitized["covariances"]["position_covariance"] == [1.0, 0.0]
    assert "latitude" not in str(sanitized)
    assert "secret-bytes" not in str(sanitized)


def test_observe_uses_only_read_only_topic_commands() -> None:
    calls = []

    def runner(command, _timeout):
        calls.append(command)
        if command[2:4] == ["list", "-t"]:
            return observer.CommandResult(0, "/imu/data [sensor_msgs/msg/Imu]\n")
        if command[2:4] == ["info", "-v"]:
            return observer.CommandResult(0, "Publisher count: 0\nSubscription count: 0\n")
        return observer.CommandResult(0, "header:\n  frame_id: imu\n")

    report = observer.observe(None, include_samples=True, timeout_s=1.0, runner=runner)
    assert [call[2:4] for call in calls] == [["list", "-t"], ["info", "-v"], ["echo", "--once"]]
    assert report["topics"][0]["type"] == "sensor_msgs/msg/Imu"
    assert report["topics"][0]["sample"] == {
        "headers": {"header.frame_id": "imu"},
        "statuses": {},
        "covariances": {},
    }


def test_observe_never_echoes_non_whitelisted_payload_types() -> None:
    calls = []

    def runner(command, _timeout):
        calls.append(command)
        if command[2:4] == ["list", "-t"]:
            return observer.CommandResult(
                0, "/rtcm [mavros_msgs/msg/RTCM]\n"
            )
        return observer.CommandResult(
            0, "Publisher count: 1\nSubscription count: 1\n"
        )

    report = observer.observe(
        ["/rtcm"], include_samples=True, timeout_s=1.0, runner=runner
    )
    assert all(call[2] != "echo" for call in calls)
    assert report["topics"][0]["sample"] is None
    assert "whitelist" in report["topics"][0]["sample_skipped"]
