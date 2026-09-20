"""Evaluation-only geometry quality metrics and valid circular fillets."""

from __future__ import annotations

import math


DEFAULT_RESAMPLE_SPACING_M = 0.25
DEFAULT_CURVATURE_DEADBAND_PER_M = 0.02
DEFAULT_LATERAL_DEADBAND_M = 0.01
ACKERMANN_WHEELBASE_M = 0.94


def _finite_xy(points):
    """Return finite two-dimensional points, rejecting malformed input."""
    result = []
    for point in points:
        if len(point) != 2:
            raise ValueError("points must contain x/y pairs")
        values = (float(point[0]), float(point[1]))
        if not all(math.isfinite(value) for value in values):
            raise ValueError("points must be finite")
        result.append(values)
    return tuple(result)


def _distance(first, second):
    """Return Euclidean distance between two XY points."""
    return math.hypot(second[0] - first[0], second[1] - first[1])


def polyline_length(points):
    """Return the length of an XY polyline."""
    return sum(_distance(first, second) for first, second in zip(points, points[1:]))


def resample_polyline(points, spacing_m=DEFAULT_RESAMPLE_SPACING_M):
    """Resample a polyline at a deterministic approximate distance spacing."""
    points = _finite_xy(points)
    spacing_m = float(spacing_m)
    if not math.isfinite(spacing_m) or spacing_m <= 0.0:
        raise ValueError("spacing_m must be finite and positive")
    if len(points) < 2:
        return points
    total = polyline_length(points)
    if total == 0.0:
        return (points[0],)
    targets = [0.0]
    target = spacing_m
    while target < total:
        targets.append(target)
        target += spacing_m
    targets.append(total)
    result = []
    segment_start = 0
    distance_before = 0.0
    for target in targets:
        while (segment_start + 1 < len(points)
               and distance_before + _distance(
                   points[segment_start], points[segment_start + 1]
               ) < target - 1.0e-12):
            distance_before += _distance(
                points[segment_start], points[segment_start + 1]
            )
            segment_start += 1
        if segment_start + 1 >= len(points):
            result.append(points[-1])
            continue
        first, second = points[segment_start], points[segment_start + 1]
        segment_length = _distance(first, second)
        fraction = ((target - distance_before) / segment_length
                    if segment_length else 0.0)
        result.append((
            first[0] + fraction * (second[0] - first[0]),
            first[1] + fraction * (second[1] - first[1]),
        ))
    return tuple(result)


def _percentile(values, fraction):
    """Return a linearly interpolated percentile."""
    ordered = sorted(values)
    if not ordered:
        return None
    index = (len(ordered) - 1) * fraction
    low, high = math.floor(index), math.ceil(index)
    return ordered[low] + (ordered[high] - ordered[low]) * (index - low)


def _heading_delta(first, second):
    """Return the signed shortest angular difference first-second."""
    return math.atan2(math.sin(first - second), math.cos(first - second))


def _nearest_signed_lateral(point, reference):
    """Return signed distance from a point to the nearest reference segment."""
    best = None
    for first, second in zip(reference, reference[1:]):
        dx, dy = second[0] - first[0], second[1] - first[1]
        denominator = dx * dx + dy * dy
        if denominator == 0.0:
            continue
        fraction = max(0.0, min(1.0, (
            (point[0] - first[0]) * dx + (point[1] - first[1]) * dy
        ) / denominator))
        nearest = (first[0] + fraction * dx, first[1] + fraction * dy)
        distance = _distance(point, nearest)
        signed = ((dx * (point[1] - nearest[1])
                   - dy * (point[0] - nearest[0])) / math.sqrt(denominator))
        candidate = (distance, signed)
        if best is None or candidate[0] < best[0]:
            best = candidate
    return best


def _sign_changes(values, deadband):
    """Count sign changes after ignoring values inside a deadband."""
    signs = [1 if value > deadband else -1 if value < -deadband else 0
             for value in values]
    signs = [value for value in signs if value]
    return sum(first != second for first, second in zip(signs, signs[1:]))


def quality_metrics(points, reference=None, *, spacing_m=DEFAULT_RESAMPLE_SPACING_M,
                    curvature_deadband_per_m=DEFAULT_CURVATURE_DEADBAND_PER_M,
                    lateral_deadband_m=DEFAULT_LATERAL_DEADBAND_M,
                    wheelbase_m=ACKERMANN_WHEELBASE_M):
    """Measure heading/curvature and lateral quality of one independent plan."""
    points = _finite_xy(points)
    if reference is not None:
        reference = _finite_xy(reference)
        if len(reference) < 2:
            raise ValueError("reference requires at least two points")
    if len(points) < 2:
        return {"available": False, "reason": "plan requires at least two points"}
    sampled = resample_polyline(points, spacing_m)
    if len(sampled) < 2:
        return {"available": False, "reason": "plan has zero length"}
    segments = tuple(zip(sampled, sampled[1:]))
    lengths = tuple(_distance(first, second) for first, second in segments)
    headings = tuple(math.atan2(second[1] - first[1], second[0] - first[0])
                     for first, second in segments)
    heading_deltas = tuple(_heading_delta(second, first)
                           for first, second in zip(headings, headings[1:]))
    curvatures = tuple(
        delta / ((lengths[index] + lengths[index + 1]) / 2.0)
        for index, delta in enumerate(heading_deltas)
        if lengths[index] > 0.0 and lengths[index + 1] > 0.0
    )
    abs_curvatures = tuple(abs(value) for value in curvatures)
    equivalent_steering = tuple(
        abs(math.atan(wheelbase_m * value)) for value in curvatures
    )
    result = {
        "available": True,
        "resampled_count": len(sampled),
        "resample_spacing_m": spacing_m,
        "total_heading_variation_rad": sum(abs(value) for value in heading_deltas),
        "curvature_sign_changes": _sign_changes(
            curvatures, float(curvature_deadband_per_m)
        ),
        "max_abs_curvature_per_m": max(abs_curvatures, default=0.0),
        "p95_abs_curvature_per_m": _percentile(abs_curvatures, 0.95),
        "max_equivalent_steering_rad": max(equivalent_steering, default=0.0),
        "p95_equivalent_steering_rad": _percentile(equivalent_steering, 0.95),
    }
    if reference is None:
        result.update({
            "lateral_error_rms_m": None,
            "max_abs_lateral_error_m": None,
            "lateral_error_sign_changes": None,
        })
        return result
    lateral = tuple(
        match[1] for point in sampled
        if (match := _nearest_signed_lateral(point, reference)) is not None
    )
    result.update({
        "lateral_error_rms_m": (
            math.sqrt(sum(value * value for value in lateral) / len(lateral))
            if lateral else None
        ),
        "max_abs_lateral_error_m": max((abs(value) for value in lateral), default=None),
        "lateral_error_sign_changes": _sign_changes(
            lateral, float(lateral_deadband_m)
        ),
    })
    return result


def valid_fillet_r4(p0, vertex, p2, *, radius_m=4.0, samples=16):
    """Derive a circular tangent fillet for a corner, for evaluation only."""
    p0, vertex, p2 = _finite_xy((p0, vertex, p2))
    radius_m = float(radius_m)
    if not math.isfinite(radius_m) or radius_m <= 0.0:
        return {"valid": False, "reason": "radius must be finite and positive"}
    incoming_length, outgoing_length = _distance(p0, vertex), _distance(vertex, p2)
    if incoming_length <= 1.0e-9 or outgoing_length <= 1.0e-9:
        return {"valid": False, "reason": "degenerate corner leg"}
    incoming = ((vertex[0] - p0[0]) / incoming_length,
                (vertex[1] - p0[1]) / incoming_length)
    outgoing = ((p2[0] - vertex[0]) / outgoing_length,
                (p2[1] - vertex[1]) / outgoing_length)
    deflection = math.atan2(
        incoming[0] * outgoing[1] - incoming[1] * outgoing[0],
        incoming[0] * outgoing[0] + incoming[1] * outgoing[1],
    )
    if abs(deflection) <= 1.0e-9:
        return {"valid": False, "reason": "no corner deflection"}
    if abs(abs(deflection) - math.pi) <= 1.0e-6:
        return {"valid": False, "reason": "U-turn is not filletable"}
    tangent_distance = radius_m * math.tan(abs(deflection) / 2.0)
    if tangent_distance >= min(incoming_length, outgoing_length):
        return {"valid": False, "reason": "corner legs are too short"}
    entry = (vertex[0] - tangent_distance * incoming[0],
             vertex[1] - tangent_distance * incoming[1])
    exit_point = (vertex[0] + tangent_distance * outgoing[0],
                  vertex[1] + tangent_distance * outgoing[1])
    turn_sign = 1.0 if deflection > 0.0 else -1.0
    normal = (-incoming[1], incoming[0])
    center = (entry[0] + turn_sign * radius_m * normal[0],
              entry[1] + turn_sign * radius_m * normal[1])
    start_angle = math.atan2(entry[1] - center[1], entry[0] - center[0])
    samples = int(samples)
    if samples < 2:
        return {"valid": False, "reason": "samples must be at least two"}
    arc_points = tuple(
        (center[0] + radius_m * math.cos(start_angle + turn_sign * abs(deflection)
                                         * index / (samples - 1)),
         center[1] + radius_m * math.sin(start_angle + turn_sign * abs(deflection)
                                         * index / (samples - 1)))
        for index in range(samples)
    )
    arc_headings = tuple(
        start_angle + turn_sign * abs(deflection) * index / (samples - 1)
        + turn_sign * math.pi / 2.0
        for index in range(samples)
    )
    return {
        "valid": True,
        "radius_m": radius_m,
        "deflection_rad": deflection,
        "tangent_distance_m": tangent_distance,
        "tangent_entry": entry,
        "tangent_exit": exit_point,
        "center": center,
        "arc_points": arc_points,
        "arc_headings_rad": arc_headings,
    }
