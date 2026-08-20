"""Pure transactional policy for coordinated navigation profiles."""
from dataclasses import dataclass
from enum import Enum


@dataclass(frozen=True)
class ProfileValues:
    ground_tolerance_m: float
    local_inflation_radius: float
    local_cost_scaling: float
    global_inflation_radius: float
    global_cost_scaling: float
    desired_linear_vel: float


PROFILES = {
    "urban": ProfileValues(0.20, 1.4, 1.3, 1.5, 1.4, 1.6),
    "rural": ProfileValues(0.25, 0.8, 3.0, 0.8, 3.0, 1.6),
}


class TransactionState(str, Enum):
    IDLE = "IDLE"
    APPLYING = "APPLYING"
    ROLLING_BACK = "ROLLING_BACK"
    SUCCEEDED = "SUCCEEDED"
    FAILED = "FAILED"


COMPONENTS = ("ground_filter", "local_inflation", "global_inflation", "controller")


class NavigationProfileTransaction:
    """Track component confirmations without performing ROS calls."""

    def __init__(self, active_profile: str = "urban") -> None:
        self.active_profile = active_profile
        self.previous_profile = active_profile
        self.target_profile = active_profile
        self.state = TransactionState.IDLE
        self.pending: list[str] = []
        self.applied: list[str] = []
        self.failed_component = ""
        self.error = ""

    def begin(self, target: str) -> str:
        target = str(target).strip().lower()
        if target not in PROFILES:
            return "profile must be 'urban' or 'rural'"
        if self.state in (TransactionState.APPLYING, TransactionState.ROLLING_BACK):
            return "profile transaction already active"
        self.previous_profile, self.target_profile = self.active_profile, target
        self.pending, self.applied = list(COMPONENTS), []
        self.failed_component = self.error = ""
        self.state = TransactionState.APPLYING
        return ""

    def confirm(self, component: str, ok: bool, error: str = "") -> None:
        if self.state != TransactionState.APPLYING or component not in self.pending:
            raise ValueError(f"unexpected profile confirmation: {component}")
        self.pending.remove(component)
        if not ok:
            self.failed_component, self.error = component, error or "update rejected"
            self.pending = list(reversed(self.applied))
            self.state = TransactionState.ROLLING_BACK
            if not self.pending:
                self.state = TransactionState.FAILED
            return
        self.applied.append(component)
        if not self.pending:
            self.active_profile = self.target_profile
            self.state = TransactionState.SUCCEEDED

    def confirm_rollback(self, component: str, ok: bool, error: str = "") -> None:
        if self.state != TransactionState.ROLLING_BACK or component not in self.pending:
            raise ValueError(f"unexpected rollback confirmation: {component}")
        self.pending.remove(component)
        if not ok:
            suffix = error or "rollback rejected"
            self.error = f"{self.error}; rollback {component}: {suffix}".strip("; ")
        if not self.pending:
            self.active_profile = self.previous_profile
            self.state = TransactionState.FAILED
