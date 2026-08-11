import math
from sensor_msgs.msg import NavSatFix
from salus_localization.gps_course_heading import CourseHeadingEstimator, heading_from_fixes
from salus_localization.global_stationary_gates import is_stationary
from salus_localization.map_gps_absolute_measurement import project_fix


def test_project_fix_uses_enu_axes() -> None:
    x_m, y_m = project_fix(-31.4858037 + 1.0 / 111_320.0, -64.2410570, -31.4858037, -64.2410570)
    assert abs(x_m) < 0.01 and math.isclose(y_m, 1.0, abs_tol=0.01)


def test_gps_heading_is_east_for_increasing_longitude() -> None:
    first = NavSatFix(); first.latitude = -31.48; first.longitude = -64.24
    second = NavSatFix(); second.latitude = first.latitude; second.longitude = first.longitude + 10.0 / (111_320.0 * math.cos(math.radians(first.latitude)))
    yaw, distance = heading_from_fixes(first, second)
    assert math.isclose(yaw, 0.0, abs_tol=1e-6) and distance > 9.9


def test_course_heading_gates_turn_and_holds_last_valid_heading() -> None:
    estimator = CourseHeadingEstimator(min_distance_m=2.0, min_speed_mps=0.8, invalid_hold_s=0.8)
    estimator.add_fix(-31.0, -64.0, 1.0)
    estimator.add_fix(-31.0, -64.0 + 3.0 / (111_320.0 * math.cos(math.radians(-31.0))), 2.0)
    valid = estimator.estimate(now_s=2.0, speed_mps=3.0, steer_deg=0.0, steer_valid=True, yaw_rate_rps=0.0)
    held = estimator.estimate(now_s=2.4, speed_mps=3.0, steer_deg=8.0, steer_valid=True, yaw_rate_rps=0.0)
    assert valid.valid and valid.reason == "ok" and held.valid and held.reason == "hold_steer_too_high"


def test_course_heading_rejects_stale_fix_and_stationary_gate_requires_fresh_telemetry() -> None:
    estimator = CourseHeadingEstimator()
    estimator.add_fix(-31.0, -64.0, 1.0)
    assert estimator.estimate(now_s=2.0, speed_mps=1.0, steer_deg=0.0, steer_valid=True, yaw_rate_rps=0.0).reason == "stale_fix"
    assert is_stationary(0.02, True, True) and not is_stationary(0.02, False, True)
