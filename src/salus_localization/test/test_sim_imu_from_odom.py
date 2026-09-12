import math

from geometry_msgs.msg import Quaternion
from nav_msgs.msg import Odometry
import pytest

from salus_localization.sim_imu_from_odom import (
    IMU_RNG_OFFSET,
    SimImuProfile,
    SimImuProcessor,
    planar_yaw_from_quaternion,
    resolve_imu_profile,
)


def make_odom(stamp_s: float = 10.0, yaw_rate: float = 0.4) -> Odometry:
    message = Odometry()
    message.header.stamp.sec = int(stamp_s)
    message.header.stamp.nanosec = int(round((stamp_s % 1.0) * 1e9))
    message.pose.pose.orientation.w = math.cos(0.4)
    message.pose.pose.orientation.z = math.sin(0.4)
    message.twist.twist.angular.z = yaw_rate
    return message


def test_planar_yaw_from_quaternion() -> None:
    quaternion = Quaternion(w=math.cos(0.4), z=math.sin(0.4))
    assert math.isclose(planar_yaw_from_quaternion(quaternion), 0.8, abs_tol=1.0e-9)


def test_clean_imu_preserves_truth_values_and_stamp() -> None:
    source = make_odom()
    output = SimImuProcessor(resolve_imu_profile("clean"), seed=6400).process(source)
    assert output is not None
    assert output.header.stamp == source.header.stamp
    assert output.orientation == source.pose.pose.orientation
    assert output.angular_velocity.z == source.twist.twist.angular.z


def test_same_seed_replays_the_same_imu_measurement() -> None:
    profile = resolve_imu_profile("independent_nominal")
    first_processor = SimImuProcessor(profile, seed=6400)
    second_processor = SimImuProcessor(profile, seed=6400)
    assert first_processor.process(make_odom(yaw_rate=1.0), now_s=10.0) is None
    assert second_processor.process(make_odom(yaw_rate=1.0), now_s=10.0) is None
    first = first_processor.release(11.0)[0]
    second = second_processor.release(11.0)[0]
    assert first is not None and second is not None
    assert first.angular_velocity.z == second.angular_velocity.z


def test_changing_seed_changes_imu_noise_and_uses_its_own_stream() -> None:
    profile = SimImuProfile("noise", gyro_noise_sigma_rps=1.0)
    first = SimImuProcessor(profile, seed=6400).process(make_odom(yaw_rate=0.0))
    second = SimImuProcessor(profile, seed=6401).process(make_odom(yaw_rate=0.0))
    assert first is not None and second is not None
    assert first.angular_velocity.z != second.angular_velocity.z
    assert SimImuProcessor(profile, seed=6400).stream_seed == 6400 + IMU_RNG_OFFSET


def test_imu_bias_has_si_units_and_dropout_removes_sample() -> None:
    biased = SimImuProfile("bias", gyro_bias_rps=0.25)
    output = SimImuProcessor(biased, seed=1).process(make_odom(yaw_rate=0.5))
    assert output is not None
    assert output.angular_velocity.z == pytest.approx(0.75)

    dropped = SimImuProcessor(SimImuProfile("drop", dropout_probability=1.0), seed=1)
    assert dropped.process(make_odom()) is None
    assert dropped.pending_count == 0


def test_imu_latency_is_sim_time_based_and_preserves_measurement_stamp() -> None:
    profile = SimImuProfile("delayed", latency_s=0.5)
    processor = SimImuProcessor(profile, seed=1)
    source = make_odom(stamp_s=10.0)
    assert processor.process(source, now_s=10.0) is None
    assert processor.release(10.49) == []
    released = processor.release(10.5)
    assert len(released) == 1
    assert released[0].header.stamp == source.header.stamp


def test_imu_clock_rollback_discards_pending_samples() -> None:
    processor = SimImuProcessor(SimImuProfile("delayed", latency_s=1.0), seed=1)
    assert processor.process(make_odom(10.0), now_s=10.0) is None
    assert processor.process(make_odom(0.0), now_s=0.0) is None
    released = processor.release(1.0)
    assert len(released) == 1
    assert released[0].header.stamp.sec == 0


def test_imu_rejects_non_finite_configuration_and_data() -> None:
    with pytest.raises(ValueError):
        SimImuProfile("invalid", gyro_noise_sigma_rps=math.nan)
    source = make_odom()
    source.twist.twist.angular.z = math.nan
    assert SimImuProcessor(resolve_imu_profile("clean")).process(source) is None
