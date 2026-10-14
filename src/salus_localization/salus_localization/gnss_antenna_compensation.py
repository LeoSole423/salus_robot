"""Planar antenna geometry and timestamp alignment, independent of ROS."""
from dataclasses import dataclass
import math


def wrapped_angle(value: float) -> float:
    return math.atan2(math.sin(value), math.cos(value))


@dataclass(frozen=True)
class MapHeading:
    stamp_s: float
    yaw_rad: float
    variance_rad2: float


class MapHeadingHistory:
    def __init__(self, max_gap_s: float = 0.2, history_s: float = 3.0):
        if not all(math.isfinite(v) and v > 0 for v in (max_gap_s, history_s)):
            raise ValueError('heading history bounds must be finite and positive')
        self.max_gap_s, self.history_s = max_gap_s, history_s
        self.samples: list[MapHeading] = []

    def add(self, sample: MapHeading) -> bool:
        if not all(math.isfinite(v) for v in (sample.stamp_s, sample.yaw_rad, sample.variance_rad2)) or sample.variance_rad2 < 0:
            return False
        if self.samples and sample.stamp_s <= self.samples[-1].stamp_s:
            return False
        self.samples.append(sample)
        self.samples = [p for p in self.samples if p.stamp_s >= sample.stamp_s-self.history_s]
        return True

    def at(self, stamp_s: float) -> MapHeading | None:
        if not math.isfinite(stamp_s):
            return None
        for sample in self.samples:
            if sample.stamp_s == stamp_s:
                return sample
        for a,b in zip(self.samples,self.samples[1:]):
            if a.stamp_s < stamp_s < b.stamp_s:
                if b.stamp_s-a.stamp_s > self.max_gap_s:
                    return None
                fraction = (stamp_s-a.stamp_s)/(b.stamp_s-a.stamp_s)
                yaw = wrapped_angle(a.yaw_rad+fraction*wrapped_angle(b.yaw_rad-a.yaw_rad))
                # Endpoints belong to one correlated estimate, not independent measurements.
                return MapHeading(stamp_s,yaw,max(a.variance_rad2,b.variance_rad2))
        return None  # Never extrapolate or retag an old orientation.


def validate_covariance(xx: float, xy: float, yy: float) -> None:
    if not all(math.isfinite(v) for v in (xx,xy,yy)) or xx < 0 or yy < 0 or xy*xy > xx*yy+1e-12:
        raise ValueError('horizontal covariance must be finite and positive semidefinite')


def rotate_covariance(xx: float, xy: float, yy: float, angle_rad: float) -> tuple[float,float,float]:
    validate_covariance(xx,xy,yy)
    if not math.isfinite(angle_rad):
        raise ValueError('rotation must be finite')
    c,s = math.cos(angle_rad),math.sin(angle_rad)
    return (c*c*xx-2*c*s*xy+s*s*yy,
            c*s*(xx-yy)+(c*c-s*s)*xy,
            s*s*xx+2*c*s*xy+c*c*yy)


def compensate_antenna(
    x_m: float, y_m: float, heading: MapHeading,
    lever_x_m: float, lever_y_m: float,
    covariance_xy: tuple[float,float,float], mount_variance_m2: float = 0.0,
) -> tuple[float,float,tuple[float,float,float]]:
    values = (x_m,y_m,heading.yaw_rad,heading.variance_rad2,lever_x_m,lever_y_m,mount_variance_m2)
    if not all(math.isfinite(v) for v in values) or heading.variance_rad2 < 0 or mount_variance_m2 < 0:
        raise ValueError('invalid antenna geometry or uncertainty')
    xx,xy,yy = covariance_xy
    validate_covariance(xx,xy,yy)
    c,s = math.cos(heading.yaw_rad),math.sin(heading.yaw_rad)
    offset_x,offset_y = c*lever_x_m-s*lever_y_m,s*lever_x_m+c*lever_y_m
    # d(base position)/d(yaw) = (offset_y, -offset_x).
    dx,dy = offset_y,-offset_x
    variance = heading.variance_rad2
    covariance = (xx+variance*dx*dx+mount_variance_m2,
                  xy+variance*dx*dy,
                  yy+variance*dy*dy+mount_variance_m2)
    return x_m-offset_x,y_m-offset_y,covariance
