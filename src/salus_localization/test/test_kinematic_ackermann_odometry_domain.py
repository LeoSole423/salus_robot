import math

import pytest

from salus_localization.kinematic_ackermann_odometry_domain import (
    FIELD_LINEAR_VELOCITY,
    FIELD_POSITION,
    KinematicOdometryConfig,
    KinematicOdometryState,
    KinematicSample,
    STATUS_OK,
    STEERING_SOURCE_VIRTUAL_CENTER_WHEEL,
    TRACTION_SOURCE_DRIVE_WHEEL,
    accept_steering,
    accept_traction,
)


CONFIG = KinematicOdometryConfig.create(
    traction_source_id="rear_drive_wheel_equivalent", steering_source_id="virtual_center_wheel",
    wheelbase_m=0.94, max_pair_skew_s=0.05, max_dt_s=0.2,
)


def traction(stamp=1.0, **changes):
    values = dict(source_id=CONFIG.traction_source_id, stamp_s=stamp,
                  source_type=TRACTION_SOURCE_DRIVE_WHEEL, status=STATUS_OK,
                  available_fields=FIELD_LINEAR_VELOCITY, measured_fields=FIELD_LINEAR_VELOCITY,
                  calculated_fields=0, inferred_fields=0, value=1.0)
    values.update(changes)
    return KinematicSample(**values)


def steering(stamp=1.0, **changes):
    values = dict(source_id=CONFIG.steering_source_id, stamp_s=stamp,
                  source_type=STEERING_SOURCE_VIRTUAL_CENTER_WHEEL, status=STATUS_OK,
                  available_fields=FIELD_POSITION, measured_fields=FIELD_POSITION,
                  calculated_fields=0, inferred_fields=0, value=0.1)
    values.update(changes)
    return KinematicSample(**values)


def pair(state, stamp=1.0, traction_changes=None, steering_changes=None):
    state = accept_traction(state, traction(stamp, **(traction_changes or {})), CONFIG).state
    return accept_steering(state, steering(stamp, **(steering_changes or {})), CONFIG)


def test_config_requires_positive_finite_limits_and_ids():
    with pytest.raises(ValueError):
        KinematicOdometryConfig.create(traction_source_id="", steering_source_id="s", wheelbase_m=1, max_pair_skew_s=1, max_dt_s=1)
    with pytest.raises(ValueError):
        KinematicOdometryConfig.create(traction_source_id="t", steering_source_id="s", wheelbase_m=math.inf, max_pair_skew_s=1, max_dt_s=1)


def test_first_pair_publishes_zero_pose_and_valid_twist():
    update = pair(KinematicOdometryState(), 10.0)
    assert update.emission is not None
    assert update.emission.stamp_s == 10.0
    assert update.emission.x_m == update.emission.y_m == update.emission.yaw_rad == 0.0
    assert update.emission.speed_mps == 1.0 and update.emission.yaw_rate_rps > 0.0


@pytest.mark.parametrize("changes", [
    {"status": 2}, {"source_type": 1}, {"available_fields": 0},
    {"value": math.nan}, {"measured_fields": 0},
    {"measured_fields": FIELD_LINEAR_VELOCITY, "calculated_fields": FIELD_LINEAR_VELOCITY},
    {"measured_fields": FIELD_LINEAR_VELOCITY * 2},
])
def test_non_consumable_selected_traction_clears_its_pending_sample(changes):
    state = accept_traction(KinematicOdometryState(), traction(), CONFIG).state
    update = accept_traction(state, traction(**changes), CONFIG)
    assert update.state.pending_traction is None and update.emission is None


def test_unselected_input_is_ignored_without_disturbing_pending_sample():
    state = accept_traction(KinematicOdometryState(), traction(), CONFIG).state
    update = accept_steering(state, steering(source_id="another_sensor"), CONFIG)
    assert update.state.pending_traction is not None and update.state.pending_steering is None


def test_disjoint_exhaustive_calculated_or_inferred_provenance_is_accepted():
    update = pair(KinematicOdometryState(), traction_changes={"measured_fields": 0, "calculated_fields": FIELD_LINEAR_VELOCITY}, steering_changes={"measured_fields": 0, "inferred_fields": FIELD_POSITION})
    assert update.emission is not None


def test_skewed_pair_is_discarded_and_neither_sample_is_reused():
    state = accept_traction(KinematicOdometryState(), traction(1.0), CONFIG).state
    update = accept_steering(state, steering(1.1), CONFIG)
    assert update.emission is None
    assert update.state.pending_traction is None and update.state.pending_steering is None
    assert accept_steering(update.state, steering(1.1), CONFIG).emission is None


def test_each_emission_requires_two_new_samples_and_integrates_once():
    first = pair(KinematicOdometryState(), 1.0)
    assert first.emission is not None
    only_traction = accept_traction(first.state, traction(1.1), CONFIG)
    assert only_traction.emission is None
    second = accept_steering(only_traction.state, steering(1.1), CONFIG)
    assert second.emission is not None and second.emission.x_m > 0.0


@pytest.mark.parametrize("second_stamp", [1.0, 0.5])
def test_repeated_or_regressive_time_does_not_publish_or_integrate(second_stamp):
    first = pair(KinematicOdometryState(), 1.0)
    update = pair(first.state, second_stamp)
    assert update.emission is None
    assert update.state.x_m == first.state.x_m and update.state.y_m == first.state.y_m
    expected = 1.0 if second_stamp == 1.0 else None
    assert update.state.baseline_stamp_s == expected


@pytest.mark.parametrize("bad_stamp", [0.0, -1.0, math.nan])
def test_nonpositive_or_nonfinite_timestamp_is_not_consumable(bad_stamp):
    update = accept_traction(KinematicOdometryState(), traction(bad_stamp), CONFIG)
    assert update.emission is None and update.state.pending_traction is None


def test_large_forward_gap_publishes_without_integrating_and_sets_new_baseline():
    first = pair(KinematicOdometryState(), 1.0)
    update = pair(first.state, 1.3)
    assert update.emission is not None
    assert update.emission.x_m == first.state.x_m
    assert update.state.baseline_stamp_s == 1.3


def test_clock_regression_requires_a_new_baseline_before_integrating_again():
    first = pair(KinematicOdometryState(), 1.0)
    regressed = pair(first.state, 0.5)
    baseline = pair(regressed.state, 0.6)
    resumed = pair(baseline.state, 0.7)
    assert regressed.emission is None
    assert baseline.emission is not None and baseline.emission.x_m == first.state.x_m
    assert resumed.emission is not None and resumed.emission.x_m > baseline.state.x_m
