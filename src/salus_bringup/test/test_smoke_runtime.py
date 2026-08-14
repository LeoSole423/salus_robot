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


class FakeFuture:
    def __init__(self, result=None) -> None:
        self._done = result is not None
        self._result = result
        self.cancelled = False

    def done(self):
        return self._done

    def result(self):
        return self._result

    def cancel(self):
        self.cancelled = True


class FakeClient:
    def __init__(self, future, ready=True) -> None:
        self.future = future
        self.ready = ready
        self.calls = 0

    def service_is_ready(self):
        return self.ready

    def call_async(self, _request):
        self.calls += 1
        return self.future


def test_service_poller_allows_only_one_request_in_flight() -> None:
    future = FakeFuture()
    client = FakeClient(future)
    poller = runtime.AsyncServicePoller(client, object, interval_s=0.0, response_timeout_s=5.0)
    poller.poll()
    poller.poll()
    poller.poll()
    assert client.calls == 1
    assert poller.evidence()["request_in_flight"]


def test_service_poller_records_response_without_request_flood() -> None:
    response = object()
    client = FakeClient(FakeFuture(response))
    poller = runtime.AsyncServicePoller(client, object, interval_s=10.0)
    poller.poll()
    poller.poll()
    assert poller.latest is response
    assert poller.responses == 1
    assert client.calls == 1


def test_service_poller_waits_for_service_and_bounds_stalled_response(monkeypatch) -> None:
    now = [10.0]
    monkeypatch.setattr(runtime.time, "monotonic", lambda: now[0])
    future = FakeFuture()
    client = FakeClient(future, ready=False)
    poller = runtime.AsyncServicePoller(client, object, interval_s=0.0, response_timeout_s=2.0)
    poller.poll()
    assert client.calls == 0
    client.ready = True
    poller.poll()
    now[0] = 12.1
    poller.poll()
    assert future.cancelled
    assert poller.timeouts == 1
    assert poller.future is None
