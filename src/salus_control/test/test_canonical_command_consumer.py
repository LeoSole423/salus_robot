import math

import pytest

from salus_control.canonical_command_consumer import (
    CanonicalCommandConfig,
    CanonicalCommandConsumer,
    CanonicalCommandSample,
)
from salus_control.control_logic import COMMAND_SOURCE_AUTO, COMMAND_SOURCE_SAFETY


def sample(**overrides) -> CanonicalCommandSample:
    values = {
        "stamp_ns": 10_000_000_000,
        "source": COMMAND_SOURCE_AUTO,
        "drive_enabled": True,
        "emergency_stop": False,
        "brake_ratio": 0.0,
        "speed_mps": 2.0,
        "steering_angle_rad": 0.2,
        "steering_angle_velocity_rad_s": 0.0,
        "acceleration_mps2": 0.0,
        "jerk_mps3": 0.0,
        "valid_for_s": 0.7,
    }
    values.update(overrides)
    return CanonicalCommandSample(**values)


def ingest(consumer, command=None, ros_now_ns=10_000_000_000, mono=5.0):
    return consumer.ingest(
        command or sample(), ros_now_ns=ros_now_ns, monotonic_now_s=mono
    )


def test_valid_command_is_accepted_then_expires_monotonically() -> None:
    consumer = CanonicalCommandConsumer(CanonicalCommandConfig())
    assert ingest(consumer).valid
    assert consumer.tick(5.699).valid
    expired = consumer.tick(5.700001)
    assert not expired.valid
    assert expired.reason == "watchdog_timeout"
    assert expired.source == COMMAND_SOURCE_SAFETY
    assert expired.emergency_stop
    assert expired.brake_ratio == 1.0


def test_requested_validity_is_capped_by_local_watchdog() -> None:
    consumer = CanonicalCommandConsumer(
        CanonicalCommandConfig(max_valid_for_s=0.2)
    )
    assert ingest(consumer, sample(valid_for_s=5.0)).valid
    assert consumer.tick(5.199).valid
    assert consumer.tick(5.200001).reason == "watchdog_timeout"


@pytest.mark.parametrize(
    "command, now_ns, reason",
    [
        (sample(source=99), 10_000_000_000, "invalid_source"),
        (sample(stamp_ns=0), 10_000_000_000, "invalid_stamp"),
        (sample(brake_ratio=1.1), 10_000_000_000, "invalid_brake_ratio"),
        (sample(speed_mps=4.1), 10_000_000_000, "speed_out_of_range"),
        (sample(steering_angle_rad=0.6), 10_000_000_000, "steering_out_of_range"),
        (sample(valid_for_s=0.0), 10_000_000_000, "invalid_validity"),
        (sample(speed_mps=math.nan), 10_000_000_000, "nonfinite_command"),
        (sample(stamp_ns=10_200_000_000), 10_000_000_000, "future_stamp"),
        (sample(stamp_ns=9_000_000_000), 10_000_000_000, "stale_on_arrival"),
    ],
)
def test_malformed_commands_fail_safe(command, now_ns: int, reason: str) -> None:
    result = ingest(
        CanonicalCommandConsumer(CanonicalCommandConfig()),
        command,
        ros_now_ns=now_ns,
    )
    assert not result.valid
    assert result.reason == reason
    assert result.emergency_stop


def test_repeated_or_regressive_stamp_fails_safe() -> None:
    consumer = CanonicalCommandConsumer(CanonicalCommandConfig())
    assert ingest(consumer).valid
    assert ingest(consumer, sample(), ros_now_ns=10_000_000_000).reason == (
        "nonmonotonic_stamp"
    )


def test_estop_and_disable_dominate_requested_motion() -> None:
    consumer = CanonicalCommandConsumer(CanonicalCommandConfig())
    estop = ingest(consumer, sample(emergency_stop=True))
    assert estop.valid
    assert not estop.drive_enabled
    assert estop.speed_mps == 0.0
    assert estop.steering_angle_rad == 0.0
    assert estop.brake_ratio == 1.0

    disabled = ingest(
        consumer,
        sample(stamp_ns=10_100_000_000, drive_enabled=False),
        ros_now_ns=10_100_000_000,
    )
    assert disabled.valid
    assert disabled.speed_mps == 0.0
    assert disabled.steering_angle_rad == 0.0


def test_service_brake_suppresses_speed_without_becoming_estop() -> None:
    result = ingest(
        CanonicalCommandConsumer(CanonicalCommandConfig()),
        sample(brake_ratio=0.3),
    )
    assert result.valid
    assert not result.emergency_stop
    assert result.brake_ratio == 0.3
    assert result.speed_mps == 0.0
    assert result.reason == "service_brake"
