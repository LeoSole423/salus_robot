"""Correlate Nav2 planner diagnostics with an active navigation goal."""

from dataclasses import dataclass


@dataclass
class PlannerFailureEvidence:
    goal_epoch: int = -1
    observed_at_s: float | None = None

    def begin_goal(self, epoch: int) -> None:
        self.goal_epoch = epoch
        self.observed_at_s = None

    def observe_log(self, *, name: str, message: str, now_s: float) -> None:
        if self.goal_epoch < 0 or not name.rsplit("/", 1)[-1].startswith("planner_server"):
            return
        lower = message.lower()
        if ("failed to create plan" in lower or
                "failed to generate a valid path" in lower):
            self.observed_at_s = now_s

    def aborted_for_no_path(self, *, epoch: int, now_s: float) -> bool:
        return (self.goal_epoch == epoch and self.observed_at_s is not None
                and 0.0 <= now_s - self.observed_at_s <= 12.0)


def operator_block_reason(code: str) -> str:
    """Describe the recovery state without guessing why a generic abort occurred."""
    if code == "NO_VALID_PATH":
        return ("No se encontró un camino seguro al siguiente punto. "
                "Revisá los obstáculos o elegí otra ruta.")
    if code == "NAV_ABORTED":
        return ("La navegación se interrumpió. Revisá el camino y los "
                "diagnósticos antes de volver a intentar.")
    return code
