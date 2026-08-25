"""Functional gates and baseline comparison policy."""

from dataclasses import dataclass
from enum import Enum


class GateState(str, Enum):
    """Machine-readable evaluation state."""

    PASS = "pass"
    FAIL = "fail"
    CALIBRATING = "calibrating"


@dataclass(frozen=True)
class GateResult:
    """One gate outcome with an actionable reason."""

    name: str
    state: GateState
    reason: str


def functional_gates(*, finite_data, plan_present, terminal_success,
                     final_distance_m, tolerance_m, sign_metrics,
                     reverse_observed, reverse_allowed):
    """Evaluate causal invariants suitable for CI from the first run."""

    checks = [
        ("finite_data", finite_data, "all required samples must be finite"),
        ("plan_present", plan_present, "a non-empty plan must be observed"),
        ("terminal_success", terminal_success, "Nav2 must report success"),
        ("arrival", final_distance_m <= tolerance_m, "final pose must be within tolerance"),
        ("turn_sign", sign_metrics.eligible_count > 0 and sign_metrics.mismatch_count == 0,
         "eligible steering commands must produce the same yaw-rate sign"),
        ("no_reverse", reverse_allowed or not reverse_observed,
         "reverse motion is forbidden by the scenario"),
    ]
    return tuple(GateResult(name, GateState.PASS if passed else GateState.FAIL, reason)
                 for name, passed, reason in checks)


def performance_gate(name, candidate_p95, baseline_p95=None, floor=0.0):
    """Gate a lower-is-better P95 only after a baseline has been calibrated."""

    if baseline_p95 is None:
        return GateResult(name, GateState.CALIBRATING,
                          "no calibrated baseline; metric is report-only")
    limit = max(baseline_p95 * 1.20, baseline_p95 + floor)
    passed = candidate_p95 <= limit
    return GateResult(name, GateState.PASS if passed else GateState.FAIL,
                      f"candidate={candidate_p95:.6g}, limit={limit:.6g}")
