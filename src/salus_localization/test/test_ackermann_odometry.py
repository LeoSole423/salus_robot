import math

from salus_localization.ackermann_odometry import (
    apply_measured_steer_sign,
    compute_yaw_rate,
    integrate_planar,
    normalize_angle,
)


def test_compute_yaw_rate_returns_zero_for_invalid_wheelbase() -> None:
    assert compute_yaw_rate(1.0, 0.2, 0.0) == 0.0


def test_compute_yaw_rate_matches_ackermann_model() -> None:
    assert compute_yaw_rate(2.0, math.radians(10.0), 0.94) == math.tan(math.radians(10.0)) * 2.0 / 0.94


def test_inverted_measured_steer_flips_yaw_rate_sign() -> None:
    steer = math.radians(apply_measured_steer_sign(-10.0, invert_sign=True))
    assert compute_yaw_rate(0.8, steer, 0.94) > 0.0


def test_integrate_planar_uses_midpoint_heading() -> None:
    x_m, y_m, yaw_rad = integrate_planar(0.0, 0.0, 0.0, 1.0, 0.5, 0.2)
    assert x_m > 0.0 and y_m > 0.0 and yaw_rad > 0.0


def test_normalize_angle_wraps_to_pi_interval() -> None:
    assert math.isclose(normalize_angle(4.0 * math.pi + 0.3), 0.3, abs_tol=1.0e-9)
