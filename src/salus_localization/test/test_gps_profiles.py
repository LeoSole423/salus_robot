import math

import pytest
from sensor_msgs.msg import NavSatFix, NavSatStatus
from salus_localization.gps_profiles import (
    GPS_RNG_OFFSET,
    SimGpsFixProcessor,
    SimGpsProfile,
    resolve_gps_profile,
)


def make_fix(stamp_s: float = 10.0) -> NavSatFix:
    message = NavSatFix()
    message.header.stamp.sec = int(stamp_s)
    message.header.stamp.nanosec = int(round((stamp_s % 1.0) * 1e9))
    message.latitude = -31.4858037
    message.longitude = -64.2410570
    return message


def test_profiles_have_stable_names() -> None:
    assert tuple(sorted(resolve_gps_profile(name).name for name in ("ideal", "f9p_rtk", "m8n"))) == ("f9p_rtk", "ideal", "m8n")


def test_ideal_profile_preserves_position() -> None:
    source = make_fix(); output = SimGpsFixProcessor(resolve_gps_profile("ideal"), 123).process(source)
    assert output is not None and output.latitude == source.latitude and output.longitude == source.longitude


def test_m8n_profile_throttles_samples() -> None:
    processor = SimGpsFixProcessor(resolve_gps_profile("m8n"), 123)
    assert processor.process(make_fix()) is not None
    second = make_fix(); second.header.stamp.nanosec = 50_000_000
    assert processor.process(second) is None


def test_clean_preserves_the_existing_f9p_receiver_statistics() -> None:
    clean = resolve_gps_profile("clean")
    f9p = resolve_gps_profile("f9p_rtk")
    assert clean.noise_m == f9p.noise_m
    assert clean.vertical_noise_m == f9p.vertical_noise_m
    assert clean.rate_hz == f9p.rate_hz
    assert clean.dropout_probability == 0.0
    assert clean.latency_s == 0.0


def test_same_seed_replays_gnss_and_uses_the_gnss_stream_offset() -> None:
    profile = resolve_gps_profile("independent_nominal")
    first_processor = SimGpsFixProcessor(profile, seed=6400)
    second_processor = SimGpsFixProcessor(profile, seed=6400)
    assert first_processor.process(make_fix(), now_s=10.0) is None
    assert second_processor.process(make_fix(), now_s=10.0) is None
    first = first_processor.release(11.0)[0]
    second = second_processor.release(11.0)[0]
    assert first is not None and second is not None
    assert first.latitude == second.latitude
    assert first.longitude == second.longitude
    assert first_processor.stream_seed == 6400 + GPS_RNG_OFFSET


def test_different_seed_changes_gnss_perturbation() -> None:
    profile = SimGpsProfile("noise", 1.0, 1.0, 0.0, 1.0, "RTK_FIXED", 2)
    first = SimGpsFixProcessor(profile, seed=6400).process(make_fix())
    second = SimGpsFixProcessor(profile, seed=6401).process(make_fix())
    assert first is not None and second is not None
    assert (first.latitude, first.longitude, first.altitude) != (
        second.latitude,
        second.longitude,
        second.altitude,
    )


def test_dropout_removes_exactly_the_sample_and_does_not_mutate_truth() -> None:
    source = make_fix()
    source_copy = (source.latitude, source.longitude, source.altitude)
    processor = SimGpsFixProcessor(
        SimGpsProfile("drop", 0.0, 0.0, 0.0, 1.0, "RTK_FIXED", 2, dropout_probability=1.0),
        seed=1,
    )
    assert processor.process(source) is None
    assert processor.pending_count == 0
    assert (source.latitude, source.longitude, source.altitude) == source_copy


def test_gnss_latency_preserves_measurement_stamp_and_uses_sim_time() -> None:
    profile = SimGpsProfile("delayed", 0.0, 0.0, 0.0, 1.0, "RTK_FIXED", 2, latency_s=0.5)
    processor = SimGpsFixProcessor(profile, seed=1)
    source = make_fix(10.0)
    assert processor.process(source, now_s=10.0) is None
    assert processor.release(10.49) == []
    released = processor.release(10.5)
    assert len(released) == 1
    assert released[0].header.stamp == source.header.stamp


def test_gnss_quality_transition_updates_status_and_covariance() -> None:
    profile = SimGpsProfile(
        "quality",
        0.0,
        0.0,
        0.0,
        0.04,
        "RTK_FIXED",
        2,
        degraded_rtk_status="RTK_FLOAT",
        degraded_navsat_status=NavSatStatus.STATUS_FIX,
        degraded_covariance_m2=4.0,
        degraded_vertical_noise_m=3.0,
        quality_transition_period_s=10.0,
        quality_degraded_duration_s=2.0,
    )
    fixed = make_fix(5.0)
    degraded = make_fix(9.0)
    processor = SimGpsFixProcessor(profile, seed=1)
    fixed_output = processor.process(fixed)
    degraded_output = processor.process(degraded)
    assert fixed_output is not None and degraded_output is not None
    assert fixed_output.status.status == NavSatStatus.STATUS_GBAS_FIX
    assert fixed_output.position_covariance[0] == 0.04
    assert degraded_output.status.status == NavSatStatus.STATUS_FIX
    assert degraded_output.position_covariance[0] == 4.0
    assert processor.quality_for(degraded_output).rtk_status == "RTK_FLOAT"


def test_gnss_clock_rollback_discards_pending_samples() -> None:
    processor = SimGpsFixProcessor(
        SimGpsProfile("delayed", 0.0, 0.0, 0.0, 1.0, "RTK_FIXED", 2, latency_s=1.0),
        seed=1,
    )
    assert processor.process(make_fix(10.0), now_s=10.0) is None
    assert processor.process(make_fix(0.0), now_s=0.0) is None
    released = processor.release(1.0)
    assert len(released) == 1
    assert released[0].header.stamp.sec == 0


def test_gnss_rejects_non_finite_configuration_and_data() -> None:
    with pytest.raises(ValueError):
        SimGpsProfile("invalid", math.nan, 0.0, 0.0, 1.0, "RTK_FIXED", 2)
    source = make_fix()
    source.latitude = math.nan
    assert SimGpsFixProcessor(resolve_gps_profile("clean")).process(source) is None
