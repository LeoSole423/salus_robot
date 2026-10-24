"""Pure recording intent, independent of ROS and media I/O."""
from dataclasses import dataclass


@dataclass
class RecordingPolicy:
    manual: bool = False
    mission: bool = False
    tail_until: float = 0.0
    known: bool = False
    stale: bool = True

    def observe(self, active: bool | None, now: float) -> None:
        self.stale = active is None
        if active is None:
            return
        if self.mission and not active:
            self.tail_until = now + 30.0
        if active:
            self.tail_until = 0.0
        self.mission = active
        self.known = True

    def wanted(self, now: float) -> bool:
        # Never turn an existing capture off on loss of mission authority.
        return self.manual or self.mission or (self.tail_until > now) or (
            self.stale and self.tail_until > 0
        )


def mission_signal(states):
    """A paused route and an active HOME leg still require video.

    One known active source is enough; idle requires all authorities available.
    """
    valid = [state for state in states if state is not None and state.get("ok") is True]
    if any(state.get("active") or state.get("paused") or state.get("return_home_active")
           for state in valid):
        return True
    return False if len(valid) == len(states) else None
