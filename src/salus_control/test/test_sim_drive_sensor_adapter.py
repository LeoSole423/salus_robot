import math
from pathlib import Path

import pytest
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState

from salus_control.sim_drive_sensor_adapter import (
    DRIVE_ODOM_TOPIC,
    SIM_JOINT_STATES_TOPIC,
    STEERING_JOINT_NAMES,
    STEERING_RNG_OFFSET,
    WHEEL_RNG_OFFSET,
    ScheduledMeasurement,
    SimDriveSensorProfile,
    SimDriveSensorProcessor,
    _SimTimeQueue,
    resolve_sim_drive_sensor_profile,
)


ROOT = Path(__file__).parents[1]


def _stamp(message, seconds: float) -> None:
    message.header.stamp.sec = int(seconds)
    message.header.stamp.nanosec = int(round((seconds % 1.0) * 1.0e9))


def _profile(**changes) -> SimDriveSensorProfile:
    values = {
        "name": "test",
        "wheel_scale": 1.0,
        "wheel_bias_mps": 0.0,
        "wheel_noise_sigma_mps": 0.0,
        "wheel_dropout_probability": 0.0,
        "wheel_latency_base_s": 0.0,
        "wheel_jitter_sigma_s": 0.0,
        "steering_bias_deg": 0.0,
        "steering_noise_sigma_deg": 0.0,
        "steering_dropout_probability": 0.0,
        "steering_latency_base_s": 0.0,
        "steering_jitter_sigma_s": 0.0,
    }
    values.update(changes)
    return SimDriveSensorProfile(**values)


def _odom(stamp_s: float = 1.0, speed_mps: float = 2.0) -> Odometry:
    message = Odometry()
    _stamp(message, stamp_s)
    message.twist.twist.linear.x = speed_mps
    return message


def _joint_states(stamp_s: float = 1.0, position_rad: float = 0.2) -> JointState:
    message = JointState()
    _stamp(message, stamp_s)
    message.name = list(STEERING_JOINT_NAMES)
    message.position = [position_rad, position_rad]
    return message


def test_clean_profile_preserves_measurement_and_stamp() -> None:
    processor = SimDriveSensorProcessor(resolve_sim_drive_sensor_profile("clean"))
    source = _odom(stamp_s=3.25, speed_mps=-0.7)

    output = processor.process_odom(source)

    assert output is not None
    assert output.release_time_s == pytest.approx(3.25)
    assert output.message.header.stamp == source.header.stamp
    assert output.message.twist.twist.linear.x == pytest.approx(-0.7)
    assert source.twist.twist.linear.x == pytest.approx(-0.7)


def test_independent_nominal_uses_issue_values_and_units() -> None:
    profile = resolve_sim_drive_sensor_profile("independent_nominal")

    assert profile.wheel_scale == pytest.approx(1.005)
    assert profile.wheel_bias_mps == pytest.approx(0.0)
    assert profile.wheel_noise_sigma_mps == pytest.approx(0.015)
    assert profile.wheel_dropout_probability == pytest.approx(0.005)
    assert profile.wheel_latency_base_s == pytest.approx(0.020)
    assert profile.wheel_jitter_sigma_s == pytest.approx(0.005)
    assert profile.steering_bias_deg == pytest.approx(0.15)
    assert profile.steering_noise_sigma_deg == pytest.approx(0.20)
    assert profile.steering_dropout_probability == pytest.approx(0.005)
    assert profile.steering_latency_base_s == pytest.approx(0.020)
    assert profile.steering_jitter_sigma_s == pytest.approx(0.005)


def test_bias_and_scale_are_applied_in_the_documented_units() -> None:
    processor = SimDriveSensorProcessor(
        _profile(
            wheel_scale=1.005,
            wheel_bias_mps=0.1,
            steering_bias_deg=0.15,
        )
    )

    wheel = processor.process_odom(_odom(speed_mps=2.0))
    steering = processor.process_joint_states(_joint_states(position_rad=0.2))

    assert wheel is not None
    assert wheel.message.twist.twist.linear.x == pytest.approx(2.11)
    assert steering is not None
    expected_position = 0.2 + math.radians(0.15)
    assert steering.message.position == pytest.approx([expected_position] * 2)


def test_same_seed_and_input_reproduce_both_independent_streams() -> None:
    profile = resolve_sim_drive_sensor_profile("independent_nominal")
    first = SimDriveSensorProcessor(profile, seed=6400)
    second = SimDriveSensorProcessor(profile, seed=6400)

    first_wheel = first.process_odom(_odom(stamp_s=4.0))
    second_wheel = second.process_odom(_odom(stamp_s=4.0))
    first_steering = first.process_joint_states(_joint_states(stamp_s=4.0))
    second_steering = second.process_joint_states(_joint_states(stamp_s=4.0))

    assert first_wheel is not None and second_wheel is not None
    assert first_steering is not None and second_steering is not None
    assert first_wheel.release_time_s == second_wheel.release_time_s
    assert (
        first_wheel.message.twist.twist.linear.x
        == second_wheel.message.twist.twist.linear.x
    )
    assert first_steering.release_time_s == second_steering.release_time_s
    assert first_steering.message.position == second_steering.message.position


def test_different_seed_changes_perturbation() -> None:
    profile = _profile(wheel_noise_sigma_mps=0.1, wheel_jitter_sigma_s=0.01)
    first = SimDriveSensorProcessor(profile, seed=1).process_odom(_odom())
    second = SimDriveSensorProcessor(profile, seed=2).process_odom(_odom())

    assert first is not None and second is not None
    assert (
        first.message.twist.twist.linear.x != second.message.twist.twist.linear.x
        or first.release_time_s != second.release_time_s
    )


def test_wheel_and_steering_rng_streams_have_stable_distinct_offsets() -> None:
    assert WHEEL_RNG_OFFSET == 101
    assert STEERING_RNG_OFFSET == 211
    assert WHEEL_RNG_OFFSET != STEERING_RNG_OFFSET


def test_dropout_removes_sample_before_it_enters_delivery_queue() -> None:
    processor = SimDriveSensorProcessor(
        _profile(wheel_dropout_probability=1.0, steering_dropout_probability=1.0),
        seed=6400,
    )

    assert processor.process_odom(_odom()) is None
    assert processor.process_joint_states(_joint_states()) is None


def test_delay_uses_sim_time_and_preserves_measurement_stamp() -> None:
    processor = SimDriveSensorProcessor(
        _profile(wheel_latency_base_s=0.2, steering_latency_base_s=0.2),
        seed=6400,
    )

    wheel = processor.process_odom(_odom(stamp_s=5.0))
    steering = processor.process_joint_states(_joint_states(stamp_s=5.0))

    assert wheel is not None and steering is not None
    assert wheel.release_time_s == pytest.approx(5.2)
    assert steering.release_time_s == pytest.approx(5.2)
    assert wheel.message.header.stamp.sec == 5
    assert steering.message.header.stamp.sec == 5


def test_sim_time_queue_is_bounded_and_clears_on_reset() -> None:
    queue = _SimTimeQueue(max_size=2)
    queue.push(ScheduledMeasurement(1.0, "first"))
    queue.push(ScheduledMeasurement(2.0, "second"))
    queue.push(ScheduledMeasurement(3.0, "third"))

    assert queue.pop_ready(1.0) == []
    queue.clear()
    assert queue.pop_ready(10.0) == []


def test_invalid_profile_values_fail_closed() -> None:
    with pytest.raises(ValueError):
        SimDriveSensorProcessor(_profile(wheel_noise_sigma_mps=math.nan))
    with pytest.raises(ValueError):
        SimDriveSensorProcessor(_profile(steering_dropout_probability=1.1))


def test_control_launch_integrates_adapter_only_for_perturbed_profiles() -> None:
    launch = (ROOT / "launch/control_sim.launch.py").read_text(encoding="utf-8")

    assert 'choices=["clean", "independent_nominal", "degraded"]' in launch
    assert 'default_value="6400"' in launch
    assert 'executable="sim_drive_sensor_adapter"' in launch
    assert DRIVE_ODOM_TOPIC in launch
    assert SIM_JOINT_STATES_TOPIC in launch
    assert "'.lower() != 'clean'" in launch


def test_adapter_does_not_publish_tf_or_commands() -> None:
    source = (ROOT / "salus_control/sim_drive_sensor_adapter.py").read_text(
        encoding="utf-8"
    )

    assert "TransformBroadcaster" not in source
    assert "Twist" not in source
    assert "sleep(" not in source
