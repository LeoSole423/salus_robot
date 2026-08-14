import importlib.util
import math
from pathlib import Path
import sys
import time

from nav_msgs.msg import Odometry


RUNTIME = Path(__file__).parents[3] / "tools" / "smoke_runtime.py"
SPEC = importlib.util.spec_from_file_location("smoke_runtime", RUNTIME)
assert SPEC and SPEC.loader
runtime = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = runtime
SPEC.loader.exec_module(runtime)


def odom(stamp: int = 1) -> Odometry:
    message = Odometry()
    message.header.stamp.sec = stamp
    message.pose.pose.orientation.w = 1.0
    return message


def test_increasing_stamps_require_multiple_progressive_messages() -> None:
    assert not runtime.has_increasing_stamps([])
    assert not runtime.has_increasing_stamps([odom(1), odom(1)])
    assert runtime.has_increasing_stamps([odom(1), odom(2)])


def test_finite_odometry_rejects_invalid_values_and_quaternion() -> None:
    assert runtime.finite_odometry(odom())
    message = odom()
    message.pose.pose.position.x = math.nan
    assert not runtime.finite_odometry(message)
    message = odom()
    message.pose.pose.orientation.w = 0.0
    assert not runtime.finite_odometry(message)


def test_phase_states_are_explicit() -> None:
    assert {state.value for state in runtime.PhaseState} == {
        "WAITING", "RUNNING", "PASSED", "FAILED", "TIMED_OUT"
    }


def test_timeout_is_bounded_and_report_is_written(tmp_path, monkeypatch) -> None:
    monkeypatch.setattr(runtime.rclpy, "spin_once", lambda *_args, **_kwargs: None)
    report_path = tmp_path / "timeout.json"
    probe = runtime.SmokeRuntime(object(), "timeout-fixture", report_path, global_timeout_s=0.1)
    started = time.monotonic()
    try:
        probe.wait("output never appears", lambda: False, 0.05, observe=lambda: {"count": 0})
    except runtime.SmokeTimeout as exc:
        assert "output never appears" in str(exc)
        probe.finish(False, error=exc)
    else:
        assert False, "missing functional output did not time out"
    assert time.monotonic() - started < 0.5
    assert report_path.exists()
    assert '"state": "TIMED_OUT"' in report_path.read_text(encoding="utf-8")
