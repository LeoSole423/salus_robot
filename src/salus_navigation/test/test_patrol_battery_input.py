import pytest

from salus_navigation.patrol_battery_input import PatrolBatteryInputPolicy


def guard(policy, *, recommended, state="WATCHING", ready=True, fresh=True, now=0.0):
    return policy.ingest_guard(
        ready=ready, fresh=fresh, state=state,
        return_home_recommended=recommended, now_s=now)


def test_valid_guard_recommendation_latches_once_idempotently():
    policy = PatrolBatteryInputPolicy()
    first = guard(policy, recommended=True)
    duplicate = guard(policy, recommended=True, now=0.1)
    assert (first.newly_latched, first.latched, first.source) == (True, True, "mission_guard")
    assert (duplicate.newly_latched, duplicate.latched) == (False, True)


@pytest.mark.parametrize("state,ready,fresh", [
    ("STALE", True, True), ("SUSPECT", True, True),
    ("UNAVAILABLE", True, True), ("WATCHING", False, True),
    ("WATCHING", True, False),
])
def test_invalid_guard_never_triggers_or_claims_guard_precedence(state, ready, fresh):
    policy = PatrolBatteryInputPolicy()
    decision = guard(policy, recommended=True, state=state, ready=ready, fresh=fresh)
    snapshot = policy.snapshot(now_s=0.0)
    assert not decision.latched
    assert not snapshot.valid_guard_seen
    assert not snapshot.guard_valid


def test_valid_non_recommending_guard_disables_soc_fallback():
    policy = PatrolBatteryInputPolicy()
    decision = guard(policy, recommended=False)
    fallback = policy.ingest_soc(present=True, percentage=10.0, now_s=0.1)
    assert decision.source == "mission_guard"
    assert policy.snapshot(now_s=0.1).valid_guard_seen
    assert not fallback.latched
    assert fallback.source == "mission_guard"


def test_soc_fallback_accepts_fractional_percentage_before_guard_is_valid():
    policy = PatrolBatteryInputPolicy(low_battery_threshold_pct=25.0)
    decision = policy.ingest_soc(present=True, percentage=0.24, now_s=0.0)
    assert (decision.newly_latched, decision.source) == (True, "soc_fallback")
    assert policy.snapshot(now_s=0.0).soc_pct == 24.0


def test_guard_latch_persists_through_recovery_and_only_resets_per_mission():
    policy = PatrolBatteryInputPolicy()
    guard(policy, recommended=True)
    recovered = guard(policy, recommended=False, now=0.1)
    assert recovered.latched
    policy.begin_mission()
    assert not policy.snapshot(now_s=0.1).return_latched
    assert policy.evaluate(now_s=0.1).source == "mission_guard"


def test_stale_samples_do_not_remain_usable():
    policy = PatrolBatteryInputPolicy(guard_timeout_s=3.0)
    guard(policy, recommended=True)
    assert policy.snapshot(now_s=3.1).guard_valid is False


@pytest.mark.parametrize("kwargs", [
    {"guard_timeout_s": 0.0}, {"guard_timeout_s": float("nan")},
    {"low_battery_threshold_pct": -1.0}, {"low_battery_threshold_pct": 101.0},
])
def test_policy_rejects_invalid_configuration(kwargs):
    with pytest.raises(ValueError):
        PatrolBatteryInputPolicy(**kwargs)
