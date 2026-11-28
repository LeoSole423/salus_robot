"""Trigger classification independent of ROS and storage."""

NS = 10**9


class SteeringJump:
    def __init__(self, threshold=0.18):
        self.threshold = threshold
        self.previous = None

    def observe(self, stamp, source, linear, angular):
        previous = self.previous
        self.previous = (stamp, abs(angular / linear)) if source == 1 and linear >= 0.5 else None
        return bool(
            previous and self.previous and 0 < stamp - previous[0] <= NS // 5
            and self.previous[1] - previous[1] >= self.threshold
        )


def navigation_reason(code, details):
    if code in {"GOAL_RESULT_ABORTED", "ROUTE_BLOCKED_WAITING", "ROUTE_BLOCKED_NEEDS_OPERATOR"}:
        candidates = {details.get(key) for key in ("failure_code", "reason_code", "reason")}
        for reason in ("NO_VALID_PATH", "PLANNER_U_TURN"):
            if reason in candidates:
                return reason
    return None
