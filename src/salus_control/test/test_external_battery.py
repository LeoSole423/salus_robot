"""Measured SOC and missing data must not revert to an ADC estimate."""
import math

import pytest
from sensor_msgs.msg import BatteryState

from salus_control.external_battery import external_battery_sample
from salus_control.battery_estimator import BatteryEstimator


def test_bms_percentage_is_preserved():
    msg = BatteryState()
    msg.present, msg.voltage, msg.percentage = True, 49.8, 0.93
    sample = external_battery_sample(msg, 10)
    assert sample.percentage == pytest.approx(0.93)
    assert sample.rx_monotonic_s == 10
    assert sample.as_dict()['source'] == 'external_bms'


@pytest.mark.parametrize('voltage,soc,present', [
    (math.nan, .93, True), (49.8, math.nan, True),
    (49.8, 93., True), (49.8, .93, False), (0., .93, True),
])
def test_unusable_measurement_is_rejected(voltage, soc, present):
    msg = BatteryState()
    msg.present, msg.voltage, msg.percentage = present, voltage, soc
    with pytest.raises(ValueError):
        external_battery_sample(msg, 0)


def test_missing_data_cannot_accrue_persistence_or_clear_latch():
    estimator = BatteryEstimator(return_home_persist_s=30)
    estimator.update(46., sample_time_s=0, traction_active=False)
    estimator.update(46., sample_time_s=20, traction_active=False)
    estimator.break_continuity()
    value = estimator.update(46., sample_time_s=100, traction_active=False)
    assert not value.return_home_recommended
    value = estimator.update(46., sample_time_s=130, traction_active=False)
    assert value.return_home_recommended
    estimator.break_continuity()
    value = estimator.update(50., sample_time_s=200, traction_active=False)
    assert value.return_home_recommended


def test_invalid_sample_never_changes_latch_even_with_zero_persistence():
    estimator = BatteryEstimator(return_home_persist_s=0, guard_clear_persist_s=0)
    value = estimator.update(46., sample_time_s=0, traction_active=False, sample_valid=False)
    assert not value.return_home_recommended
    estimator.update(46., sample_time_s=1, traction_active=False)
    value = estimator.update(50., sample_time_s=100, traction_active=False, sample_valid=False)
    assert value.return_home_recommended
