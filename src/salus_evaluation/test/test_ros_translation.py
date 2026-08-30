import math

from nav_msgs.msg import Odometry
from salus_interfaces.msg import DriveTelemetry, NavEvent

from salus_evaluation.evaluation_runner import (_status_snapshot,
                                                _telemetry_snapshot, _stamp)


def test_stamp_accepts_header_and_direct_ros_time_layouts():
    odometry = Odometry()
    odometry.header.stamp.sec = 2
    odometry.header.stamp.nanosec = 500_000_000
    event = NavEvent()
    event.stamp.sec = 3
    event.stamp.nanosec = 250_000_000
    assert _stamp(odometry) == 2.5
    assert _stamp(event) == 3.25


def test_drive_telemetry_stamp_and_degree_conversion_are_available_to_runner():
    telemetry = DriveTelemetry()
    telemetry.stamp.sec = 4
    telemetry.stamp.nanosec = 500_000_000
    telemetry.steer_deg_measured = 90.0
    assert _stamp(telemetry) == 4.5
    assert math.radians(telemetry.steer_deg_measured) == math.pi / 2


def test_controller_json_snapshots_require_the_documented_nested_payloads():
    status = _status_snapshot(1.0, {"source": "auto", "fresh": True, "command": {
        "drive_enabled": True, "estop": False, "speed_mps": 1.0,
        "brake_pct": 0, "requested_linear_x_mps": 1.0,
        "requested_angular_z_rps": .2, "requested_steer_rad": .3,
        "applied_steer_rad": .2, "steering_limit_used_rad": .25,
        "steer_saturated": True, "speed_limited": False,
        "min_speed_enforced": False,
    }})
    assert status.steer_saturated and status.applied_steer_rad == .2
    telemetry = _telemetry_snapshot(1.0, {"requested_auto_command": {
        "speed_mps": 1.0, "requested_steer_rad": .3, "applied_steer_rad": .2,
    }, "ackermann_limits": {
        "steering_limit_deg": 30.0, "operational_steering_limit_deg": 18.0,
        "effective_steering_limit_deg": 18.0,
    }})
    assert telemetry.effective_steering_limit_deg == 18.0
    assert _status_snapshot(1.0, {}) is None
    assert _telemetry_snapshot(1.0, {}) is None
    assert _status_snapshot(1.0, {"command": {"speed_mps": 1.0}}) is None
