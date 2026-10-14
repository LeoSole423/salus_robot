"""Timestamp-driven maneuver classification; no ROS or absolute yaw authority."""
from dataclasses import dataclass
import math


@dataclass(frozen=True)
class TurnConfig:
    enter_rate_rps: float = 0.18
    exit_rate_rps: float = 0.08
    enter_duration_s: float = 0.25
    exit_duration_s: float = 0.35
    min_speed_mps: float = 0.3
    min_steer_deg: float = 5.0
    max_motion_age_s: float = 0.5
    max_window_excursion_rad: float = math.radians(5.0)

    def __post_init__(self):
        values = tuple(self.__dict__.values())
        if not all(math.isfinite(v) and v > 0 for v in values):
            raise ValueError('turn parameters must be finite and positive')
        if self.exit_rate_rps >= self.enter_rate_rps:
            raise ValueError('exit rate must be below enter rate')


class HeadingTurnPolicy:
    def __init__(self, config=TurnConfig()):
        self.config = config
        self.state = 'NORMAL'
        self.reason = 'no_motion'
        self.last_stamp = None
        self.confirm_since = None
        self.angle = 0.0
        self.history = []
        self.generation = 0

    def update(self, *, stamp_s, now_s, speed_mps, steer_deg, valid, yaw_rate_rps):
        c = self.config
        finite = all(math.isfinite(v) for v in (stamp_s, now_s, speed_mps, steer_deg, yaw_rate_rps))
        if (not valid or not finite or not 0 <= now_s-stamp_s <= c.max_motion_age_s
                or (self.last_stamp is not None and stamp_s <= self.last_stamp)):
            if self.history:
                self.history.clear()
                self.generation += 1
            self.reason = 'motion_invalid_or_stale'
            self.confirm_since = None
            return False
        if self.last_stamp is not None:
            dt = stamp_s-self.last_stamp
            if dt > c.max_motion_age_s:
                self.history.clear()
                self.confirm_since = None
                self.generation += 1
            else:
                self.angle += yaw_rate_rps*dt
        self.last_stamp = stamp_s
        self.history.append((stamp_s, self.angle))
        self.history = [p for p in self.history if p[0] >= stamp_s-13.0]
        moving = abs(speed_mps) >= c.min_speed_mps
        strong = moving and abs(yaw_rate_rps) >= c.enter_rate_rps and abs(steer_deg) >= c.min_steer_deg
        quiet = abs(yaw_rate_rps) <= c.exit_rate_rps
        condition = quiet if self.state == 'STRONG_TURN' else strong
        if condition:
            if self.confirm_since is None:
                self.confirm_since = stamp_s
            duration = c.exit_duration_s if self.state == 'STRONG_TURN' else c.enter_duration_s
            if stamp_s-self.confirm_since >= duration:
                self.state = 'REACQUIRE' if self.state == 'STRONG_TURN' else 'STRONG_TURN'
                self.confirm_since = None
                self.history = [(stamp_s, self.angle)]
                self.generation += 1
        else:
            self.confirm_since = None
        self.reason = ('motion_contradictory' if moving and abs(yaw_rate_rps) >= c.enter_rate_rps
                       and abs(steer_deg) < c.min_steer_deg else self.state.lower())
        return True

    def window_valid(self, start_s, end_s):
        # Include the causal sample at/before the first fix and require coverage.
        before = [p for p in self.history if p[0] <= start_s]
        inside = [p for p in self.history if start_s < p[0] <= end_s]
        if not before or not inside or end_s-inside[-1][0] > self.config.max_motion_age_s:
            return False
        angles = [before[-1][1]] + [p[1] for p in inside]
        return max(angles)-min(angles) <= self.config.max_window_excursion_rad
