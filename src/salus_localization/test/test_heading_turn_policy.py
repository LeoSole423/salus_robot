import math
import pytest
from salus_localization.heading_turn_policy import HeadingTurnPolicy, TurnConfig


def motion(policy, t, rate, steer=12.0, speed=1.2, valid=True):
    return policy.update(stamp_s=t, now_s=t, speed_mps=speed,
                         steer_deg=steer, valid=valid, yaw_rate_rps=rate)


@pytest.mark.parametrize('sign', [-1, 1])
def test_sustained_turn_and_clean_reacquisition(sign):
    policy = HeadingTurnPolicy()
    for i in range(21):
        motion(policy, i*0.05, sign*0.3, sign*12)
    assert policy.state == 'STRONG_TURN'
    assert policy.generation == 1
    for i in range(21, 31):
        motion(policy, i*0.05, 0, 1)
    assert policy.state == 'REACQUIRE'
    assert policy.generation == 2
    assert not policy.window_valid(0, 1.5)
    for i in range(31, 71):
        motion(policy, i*0.05, 0, 1)
    assert policy.window_valid(1.5, 3.5)


def test_alternating_small_oscillations_keep_window_and_soft_curve_is_separate():
    policy = HeadingTurnPolicy()
    for i in range(101):
        motion(policy, i*0.05, 0.06 if i % 2 else -0.06, 4)
    assert policy.state == 'NORMAL'
    assert policy.generation == 0
    assert policy.window_valid(1, 5)
    for i in range(101, 161):
        motion(policy, i*0.05, 0.12, 6)
    assert policy.state == 'NORMAL'
    assert not policy.window_valid(5, 8)


def test_stationary_steering_and_invalid_samples_do_not_claim_straight():
    policy = HeadingTurnPolicy()
    for i in range(20):
        motion(policy, i*0.05, 0, 30, speed=0)
    assert policy.state == 'NORMAL'
    assert not motion(policy, 1, math.nan)
    assert policy.reason == 'motion_invalid_or_stale'
    assert not motion(policy, 0.1, 0)
    with pytest.raises(ValueError):
        TurnConfig(exit_rate_rps=0.3)
