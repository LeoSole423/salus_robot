import importlib.util
from pathlib import Path
import sys

from lifecycle_msgs.msg import State


PROBE = Path(__file__).parents[3] / "tools" / "lifecycle_readiness_probe.py"
SPEC = importlib.util.spec_from_file_location("lifecycle_readiness_probe", PROBE)
assert SPEC and SPEC.loader
probe = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = probe
SPEC.loader.exec_module(probe)


def _evidence() -> probe.LifecycleEvidence:
    return probe.LifecycleEvidence(node="/test", service="/test/get_state")


def test_active_state_is_accepted() -> None:
    evidence = _evidence()
    evidence.service_available = True
    evidence.record_state(State.PRIMARY_STATE_ACTIVE, "active", 0.2)
    evidence.finish()
    assert evidence.active
    assert evidence.failure == ""


def test_inactive_and_unconfigured_states_are_reported() -> None:
    states = (
        (State.PRIMARY_STATE_INACTIVE, "inactive"),
        (State.PRIMARY_STATE_UNCONFIGURED, "unconfigured"),
    )
    for state_id, label in states:
        evidence = _evidence()
        evidence.service_available = True
        evidence.record_state(state_id, label, 0.2)
        evidence.finish()
        assert not evidence.active
        assert label in evidence.failure


def test_absent_and_nonresponsive_services_are_distinguished() -> None:
    absent = _evidence()
    absent.finish()
    assert absent.failure == "lifecycle service unavailable"
    nonresponsive = _evidence()
    nonresponsive.service_available = True
    nonresponsive.finish()
    assert nonresponsive.failure == "lifecycle service did not respond"


def test_invalid_state_is_not_active() -> None:
    evidence = _evidence()
    evidence.service_available = True
    evidence.record_state(99, "invalid", 0.2)
    evidence.finish()
    assert not evidence.active
    assert "invalid" in evidence.failure


class _Future:
    def __init__(self, done=False, response=None) -> None:
        self._done = done
        self._response = response
        self.cancelled = False

    def done(self):
        return self._done

    def result(self):
        return self._response

    def cancel(self):
        self.cancelled = True


class _Client:
    def __init__(self, futures) -> None:
        self.futures = list(futures)
        self.calls = 0

    def service_is_ready(self):
        return True

    def call_async(self, _request):
        future = self.futures[self.calls]
        self.calls += 1
        return future


def _bare_probe(client, *, request_timeout=2.0, backoff=0.5):
    instance = object.__new__(probe.LifecycleReadinessProbe)
    instance.started_at = 0.0
    instance.request_timeout_s = request_timeout
    instance.retry_backoff_s = backoff
    instance.lifecycle_clients = {"/test": client}
    instance.pending = {}
    instance.evidence = {"/test": _evidence()}
    return instance


def test_pending_request_has_independent_deadline() -> None:
    request = probe.PendingRequest(_Future(), sent_at_s=10.0)
    assert not request.expired(11.9, 2.0)
    assert request.expired(12.0, 2.0)


def test_lost_request_is_cancelled_and_retried(monkeypatch) -> None:
    lost = _Future()
    active = type("Response", (), {
        "current_state": type("State", (), {
            "id": State.PRIMARY_STATE_ACTIVE,
            "label": "active",
        })(),
    })()
    client = _Client([lost, _Future(done=True, response=active)])
    instance = _bare_probe(client, request_timeout=2.0, backoff=0.5)
    clock = iter((10.0, 12.1, 12.4, 12.6, 12.7))
    monkeypatch.setattr(probe.time, "monotonic", lambda: next(clock))

    instance.poll()  # Send the first request.
    instance.poll()  # Expire it.
    instance.poll()  # Backoff has not elapsed.
    instance.poll()  # Send the retry.
    instance.poll()  # Consume the active response.

    evidence = instance.evidence["/test"]
    assert lost.cancelled
    assert evidence.attempts == 2
    assert evidence.request_timeouts == 1
    assert evidence.responses == 1
    assert evidence.active
