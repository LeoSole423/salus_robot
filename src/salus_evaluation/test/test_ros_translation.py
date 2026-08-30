import math

import pytest

from nav_msgs.msg import Odometry
from salus_interfaces.msg import DriveTelemetry, NavEvent

from salus_evaluation.evaluation_runner import (_command_chain, _status_snapshot,
                                                _telemetry_snapshot, _stamp,
                                                _trial_json_error_counts)
from salus_evaluation.models import (TimedDriveTelemetry, TimedFinalCommand,
                                     TimedVehicleCommand)


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
    missing_source = {"fresh": True, "command": dict(status.__dict__)}
    missing_source["command"].pop("source", None)
    missing_source["command"].update({
        "drive_enabled": True, "estop": False, "speed_mps": 1.0,
        "brake_pct": 0, "requested_linear_x_mps": 1.0,
        "requested_angular_z_rps": .2, "requested_steer_rad": .3,
        "applied_steer_rad": .2, "steering_limit_used_rad": .25,
        "steer_saturated": True, "speed_limited": False,
        "min_speed_enforced": False,
    })
    assert _status_snapshot(1.0, missing_source) is None


@pytest.mark.parametrize(
    "value", ["abc", "false", float("nan"), float("inf"), True,
              pytest.param(10 ** 10000, id="huge_int")]
)
def test_controller_snapshots_reject_invalid_types_and_non_finite_numbers(value):
    payload = {"source": "auto", "fresh": True, "command": {
        "drive_enabled": True, "estop": False, "speed_mps": value,
        "brake_pct": 0, "requested_linear_x_mps": 1.0,
        "requested_angular_z_rps": .2, "requested_steer_rad": .3,
        "applied_steer_rad": .2, "steering_limit_used_rad": .25,
        "steer_saturated": True, "speed_limited": False,
        "min_speed_enforced": False,
    }}
    assert _status_snapshot(1.0, payload) is None
    payload["command"]["speed_mps"] = 1.0
    payload["command"]["drive_enabled"] = "false"
    assert _status_snapshot(1.0, payload) is None


def test_command_chain_summary_preserves_translation_and_observed_aggregates():
    final = (TimedFinalCommand(1.0, .5, .2, 25, 7),)
    vehicle = (TimedVehicleCommand(1.1, 9, True, False, .25, .5, .1),)
    drive = (TimedDriveTelemetry(1.2, True, True, True, False, True, True,
                                 "auto", .45, .08, 20),)
    status = _status_snapshot(1.1, {"source": "auto", "fresh": True, "command": {
        "drive_enabled": True, "estop": False, "speed_mps": .5,
        "brake_pct": 25, "requested_linear_x_mps": .5,
        "requested_angular_z_rps": .2, "requested_steer_rad": .15,
        "applied_steer_rad": .1, "steering_limit_used_rad": .12,
        "steer_saturated": True, "speed_limited": False,
        "min_speed_enforced": False,
    }})
    telemetry = _telemetry_snapshot(1.15, {"requested_auto_command": {
        "speed_mps": .5, "requested_steer_rad": .15, "applied_steer_rad": .1,
    }, "ackermann_limits": {
        "steering_limit_deg": 30.0, "operational_steering_limit_deg": 18.0,
        "effective_steering_limit_deg": 18.0,
    }})
    chain = _command_chain((), (), final, vehicle, drive, (status,), (telemetry,))
    translation = chain["twist_to_ackermann"][0]
    assert translation["vehicle_source"] == 9
    assert translation["vehicle_brake_ratio"] == .25
    summary = chain["summary"]
    assert summary["cmd_vel_final"]["brake_pct_histogram"] == {"25": 1}
    assert summary["vehicle_command"]["brake_ratio_histogram"] == {"0.25": 1}
    assert summary["ackermann"][
        "requested_to_applied_steer_delta_rad"
    ]["last"] == pytest.approx(-.05)
    assert summary["ackermann_limits"]["effective_steering_limit_deg"]["last"] == 18.0
    assert _trial_json_error_counts((("status", .5), ("status", 1.0),
                                     ("telemetry", 1.1)), 1.0) == {
        "status": 1, "telemetry": 1,
    }
