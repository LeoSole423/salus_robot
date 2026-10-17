"""ROS-free admission of planner returns, with a non-renewable wait budget."""

from dataclasses import dataclass
import math

Point = tuple[float, float]


@dataclass(frozen=True)
class ManeuverConfig:
    return_m: float = 1.17
    opposing_rad: float = 3 * math.pi / 4
    loop_rad: float = 3 * math.pi / 2
    goal_radius_m: float = 1.2
    wait_s: float = 15.0

    def __post_init__(self):
        for value in (self.return_m, self.goal_radius_m, self.wait_s):
            if not math.isfinite(value) or value <= 0:
                raise ValueError('distances and wait must be finite and positive')
        if not math.pi / 2 < self.opposing_rad < math.pi:
            raise ValueError('opposing angle must be between pi/2 and pi')
        if not math.pi < self.loop_rad <= 2 * math.pi:
            raise ValueError('loop angle must be in (pi, 2*pi]')


@dataclass(frozen=True)
class ManeuverResult:
    reason: str = ''
    turn_rad: float = 0.0
    return_m: float = 0.0


def angle_delta(a, b):
    return math.atan2(math.sin(a - b), math.cos(a - b))


def inspect_leg(points, heading, config):
    """Combine signed turning and longitudinal regression, never angle alone."""
    vertices = [points[0]]
    for point in points[1:]:
        if math.dist(vertices[-1], point) > 1e-6:
            vertices.append(point)
    tangents = [math.atan2(b[1] - a[1], b[0] - a[0])
                for a, b in zip(vertices, vertices[1:])]
    if not tangents:
        return ManeuverResult()
    # A loop requires both a large signed winding and return to an earlier
    # sector. Turning alone cannot distinguish an open arc from a planner loop.
    signed = high = low = 0.0
    sectors = {}
    previous = tangents[0]
    for i, point in enumerate(vertices):
        tangent = tangents[min(i, len(tangents) - 1)]
        signed += angle_delta(tangent, previous)
        previous = tangent
        high, low = max(high, signed), min(low, signed)
        col, row = (math.floor(v / config.return_m) for v in point)
        for dc in (-1, 0, 1):
            for dr in (-1, 0, 1):
                for earlier, turn in sectors.get((col + dc, row + dr), ()):
                    if (abs(signed - turn) >= config.loop_rad
                            and math.dist(point, earlier) <= config.return_m):
                        return ManeuverResult('PLANNER_LOOP', high - low)
        sectors.setdefault((col, row), []).append((point, signed))
    peak_turn = high - low
    references = [math.atan2(points[-1][1] - points[0][1],
                             points[-1][0] - points[0][0])]
    if heading is not None:
        references.append(heading)
    for reference in references:
        ux, uy = math.cos(reference), math.sin(reference)
        projections = [x * ux + y * uy for x, y in points]
        high = projections[0]
        regression = 0.0
        for value in projections:
            regression = max(regression, high - value)
            high = max(high, value)
        opposing = max(abs(angle_delta(t, reference)) for t in tangents)
        if regression >= config.return_m and opposing >= config.opposing_rad:
            return ManeuverResult('PLANNER_U_TURN', peak_turn, regression)
    return ManeuverResult('', peak_turn)


def inspect_maneuver(points, *, robot_yaw, goals=(), config=ManeuverConfig()):
    """Partition by ordered requested visits; an intended circuit is not a loop defect.

    Inputs must be the untraversed path. Requested poses carry intent only;
    this function cannot credit checkpoints or change navigation goals.
    """
    if not math.isfinite(robot_yaw) or any(
            not math.isfinite(v) for p in (*points, *goals) for v in p):
        return ManeuverResult('MANEUVER_GEOMETRY_INVALID')
    if len(points) < 2:
        return ManeuverResult()
    start = 0
    legs = []
    for goal in goals:
        # First ordered entry into a requested region, never the globally
        # nearest point (which could jump across a self-intersection).
        hit = next((i for i in range(start + 1, len(points))
                    if math.dist(points[i], goal) <= config.goal_radius_m), None)
        if hit is None:
            # Missing intent cannot exempt a suspicious path.
            break
        legs.append(points[start:hit + 1])
        start = hit
    if start < len(points) - 1:
        legs.append(points[start:])
    for i, leg in enumerate(legs):
        result = inspect_leg(leg, robot_yaw if i == 0 else None, config)
        if result.reason:
            return result
    return ManeuverResult()


@dataclass(frozen=True)
class GuardDecision:
    state: str
    reason: str = ''
    remaining_s: float = 0.0


class ManeuverWait:
    """Only adoption clears a wait; validation cannot release the old controller."""

    def __init__(self, config=ManeuverConfig()):
        self.config = config
        self.reset()

    def reset(self):
        self.deadline = None
        self.cause = ''
        self.approved = None
        self.terminal = False

    def observe(self, *, now_s, reason='', key=None, candidate=False, valid=True):
        if self.deadline is not None and now_s >= self.deadline:
            self.terminal = True
        if self.terminal:
            return self.snapshot(now_s)
        if reason and self.deadline is None:
            self.deadline = now_s + self.config.wait_s
            self.cause = reason
        if self.deadline is not None:
            if candidate and valid and not reason:
                self.approved = key
            elif not candidate and valid and not reason and key == self.approved and key is not None:
                self.reset()
        return self.snapshot(now_s)

    def snapshot(self, now_s):
        if self.deadline is None:
            return GuardDecision('CLEAR')
        if now_s >= self.deadline:
            self.terminal = True
        return GuardDecision('NEEDS_OPERATOR' if self.terminal else 'WAITING_REPLAN',
                             self.cause, max(0.0, self.deadline - now_s))


def guard_reason(decision):
    return f'maneuver:{decision.state}:{decision.reason}'


def forward_suffix(points, robot, progress_m=0.0, search_m=4.0):
    """Project only into a bounded forward window, never across a future loop.

    The 4 m search matches the controller's max_robot_pose_search_dist. A
    candidate must be checked in full, without any nearest-point pruning.
    """
    walked = 0.0
    best = None
    for i, (a, b) in enumerate(zip(points, points[1:])):
        length = math.dist(a, b)
        if length <= 1e-6:
            continue
        if walked + length >= progress_m and walked <= progress_m + search_m:
            lower = max(0.0, (progress_m - walked) / length)
            upper = min(1.0, (progress_m + search_m - walked) / length)
            ratio = ((robot[0] - a[0]) * (b[0] - a[0])
                     + (robot[1] - a[1]) * (b[1] - a[1])) / length ** 2
            ratio = min(upper, max(lower, ratio))
            xy = (a[0] + ratio * (b[0] - a[0]), a[1] + ratio * (b[1] - a[1]))
            distance = math.dist(robot, xy)
            if best is None or distance < best[0]:
                best = (distance, walked + ratio * length, i, xy)
        walked += length
    if best is None:
        return points, progress_m
    return (best[3], *points[best[2] + 1:]), best[1]
