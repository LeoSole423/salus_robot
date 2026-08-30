"""Pure geometry and navigation metric calculations."""

import math

from .models import (ArrivalMetrics, ExpectedTurn, LocalizationMetrics, Pose2D,
                     SignMetrics, TrackingMetrics)


COMMAND_CHAIN_MAX_ALIGNMENT_GAP_S = 0.2
COMMAND_CHAIN_EPSILON = 1.0e-6


def angle_delta(a, b):
    """Return signed shortest angular difference a-b."""
    return math.atan2(math.sin(a - b), math.cos(a - b))


def absolute_goal(spawn, goal):
    """Transform a relative goal from vehicle coordinates into world coordinates."""
    cosine, sine = math.cos(spawn.yaw_rad), math.sin(spawn.yaw_rad)
    return Pose2D(spawn.x_m + cosine * goal.forward_m - sine * goal.lateral_m,
                  spawn.y_m + sine * goal.forward_m + cosine * goal.lateral_m,
                  spawn.yaw_rad + goal.yaw_offset_rad)


def expected_turn_from_path(start, path, lookahead_m=1.0,
                            lateral_deadband_m=0.05):
    """Infer the initial requested turn from a plan and starting pose."""
    if len(path) < 2:
        raise ValueError("turn inference requires at least two plan points")
    nearest = min(
        range(len(path)),
        key=lambda index: math.hypot(
            path[index].x_m - start.x_m, path[index].y_m - start.y_m
        ),
    )
    target = path[-1]
    traveled = 0.0
    previous = path[nearest]
    for candidate in path[nearest + 1:]:
        traveled += math.hypot(
            candidate.x_m - previous.x_m, candidate.y_m - previous.y_m
        )
        target, previous = candidate, candidate
        if traveled >= lookahead_m:
            break
    dx, dy = target.x_m - start.x_m, target.y_m - start.y_m
    lateral = -math.sin(start.yaw_rad) * dx + math.cos(start.yaw_rad) * dy
    if lateral > lateral_deadband_m:
        return ExpectedTurn.LEFT
    if lateral < -lateral_deadband_m:
        return ExpectedTurn.RIGHT
    return ExpectedTurn.STRAIGHT


def trial_data_finite(goal, pose_streams, commands, plan):
    """Validate every numeric input required by functional evaluation."""
    if goal is None:
        return False
    values = [goal.x_m, goal.y_m, goal.yaw_rad]
    for stream in pose_streams:
        for item in stream:
            values.extend((item.stamp_s, item.pose.x_m, item.pose.y_m,
                           item.pose.yaw_rad, item.linear_x_mps,
                           item.angular_z_rps))
    for item in commands:
        values.extend((item.stamp_s, item.linear_x_mps, item.angular_z_rps))
    for item in plan:
        values.extend((item.x_m, item.y_m, item.yaw_rad))
    return all(math.isfinite(value) for value in values)


def _percentile(values, fraction):
    ordered = sorted(values)
    if not ordered:
        raise ValueError("metric requires samples")
    index = (len(ordered) - 1) * fraction
    low, high = int(math.floor(index)), int(math.ceil(index))
    return ordered[low] + (ordered[high] - ordered[low]) * (index - low)


def _nearest(point, path):
    best = None
    for start, end in zip(path, path[1:]):
        dx, dy = end.x_m - start.x_m, end.y_m - start.y_m
        denominator = dx * dx + dy * dy
        projection = (
            (point.x_m - start.x_m) * dx + (point.y_m - start.y_m) * dy
        )
        t = 0.0 if denominator == 0 else max(
            0.0, min(1.0, projection / denominator)
        )
        x, y = start.x_m + t * dx, start.y_m + t * dy
        candidate = (math.hypot(point.x_m - x, point.y_m - y), math.atan2(dy, dx))
        if best is None or candidate[0] < best[0]:
            best = candidate
    if best is None:
        raise ValueError("path requires at least two points")
    return best


def tracking_metrics(poses, path):
    """Measure tracking against a polyline without ROS dependencies."""
    if not poses:
        raise ValueError("tracking requires poses")
    distances, headings = zip(*(_nearest(item.pose, path) for item in poses))
    heading_errors = [abs(angle_delta(item.pose.yaw_rad, tangent))
                      for item, tangent in zip(poses, headings)]
    traveled = sum(math.hypot(b.pose.x_m - a.pose.x_m, b.pose.y_m - a.pose.y_m)
                   for a, b in zip(poses, poses[1:]))
    direct = math.hypot(poses[-1].pose.x_m - poses[0].pose.x_m,
                        poses[-1].pose.y_m - poses[0].pose.y_m)
    return TrackingMetrics(len(poses), math.sqrt(sum(x*x for x in distances) / len(distances)),
                           _percentile(distances, .95), max(distances),
                           _percentile(heading_errors, .95), traveled,
                           direct / traveled if traveled else 1.0)


def command_response_sign(commands, poses, linear_min=0.1, angular_min=0.02,
                          response_delay_s=0.2):
    """Compare each eligible command with the nearest observed yaw-rate sample."""
    eligible = [cmd for cmd in commands if cmd.linear_x_mps > linear_min and
                abs(cmd.angular_z_rps) > angular_min]
    mismatches = 0
    first_response_sign = 0
    for command in eligible:
        response_stamp = command.stamp_s + response_delay_s
        response = min(poses, key=lambda pose: abs(pose.stamp_s - response_stamp))
        if first_response_sign == 0 and abs(response.angular_z_rps) > angular_min:
            first_response_sign = 1 if response.angular_z_rps > 0 else -1
        if command.angular_z_rps * response.angular_z_rps < 0:
            mismatches += 1
    first_sign = (1 if eligible[0].angular_z_rps > 0 else -1) if eligible else 0
    return SignMetrics(len(eligible), mismatches,
                       mismatches / len(eligible) if eligible else None,
                       first_sign, first_response_sign)


def arrival_metrics(poses, goal, tolerance_m, success_s=None):
    """Measure first entry, subsequent exits, overshoot and post-success motion."""
    distances = [math.hypot(p.pose.x_m-goal.x_m, p.pose.y_m-goal.y_m) for p in poses]
    epsilon = max(1e-12, tolerance_m * 1e-9)
    entries = [i for i, distance in enumerate(distances)
               if distance <= tolerance_m + epsilon]
    first = entries[0] if entries else None
    exits = 0
    if first is not None:
        inside = True
        for distance in distances[first + 1:]:
            if inside and distance > tolerance_m + epsilon:
                exits += 1
            inside = distance <= tolerance_m + epsilon
    post = None
    if success_s is not None:
        post_poses = [p for p in poses if success_s <= p.stamp_s <= success_s + 1.0]
        post = sum(math.hypot(b.pose.x_m-a.pose.x_m, b.pose.y_m-a.pose.y_m)
                   for a, b in zip(post_poses, post_poses[1:]))
    along = [((p.pose.x_m-goal.x_m) * math.cos(goal.yaw_rad) +
              (p.pose.y_m-goal.y_m) * math.sin(goal.yaw_rad)) for p in poses]
    return ArrivalMetrics(poses[first].stamp_s if first is not None else None, exits,
                          min(distances), distances[-1], post, max(0.0, max(along)))


def localization_metrics(ground_truth, estimates, max_alignment_gap_s=0.2):
    """Compare estimates with nearest ground-truth samples."""
    if not estimates or not ground_truth:
        raise ValueError("localization requires both streams")
    errors, yaw_errors = [], []
    for estimate in estimates:
        truth = min(ground_truth, key=lambda item: abs(item.stamp_s-estimate.stamp_s))
        gap_s = abs(truth.stamp_s - estimate.stamp_s)
        if gap_s > max_alignment_gap_s:
            raise ValueError(
                f"localization alignment gap {gap_s:.3f}s exceeds "
                f"{max_alignment_gap_s:.3f}s"
            )
        errors.append(math.hypot(estimate.pose.x_m-truth.pose.x_m,
                                 estimate.pose.y_m-truth.pose.y_m))
        yaw_errors.append(abs(angle_delta(estimate.pose.yaw_rad, truth.pose.yaw_rad)))
    return LocalizationMetrics(len(errors), math.sqrt(sum(x*x for x in errors)/len(errors)),
                               _percentile(errors, .95),
                               math.sqrt(sum(x*x for x in yaw_errors)/len(yaw_errors)),
                               errors[-1])


def latest_prior(samples, stamp_s, max_gap_s=COMMAND_CHAIN_MAX_ALIGNMENT_GAP_S):
    """Return the latest causally prior sample and its auditable alignment gap."""
    eligible = [item for item in samples if item.stamp_s <= stamp_s]
    if not eligible:
        return None, None
    sample = max(eligible, key=lambda item: item.stamp_s)
    gap_s = stamp_s - sample.stamp_s
    return (sample, gap_s) if gap_s <= max_gap_s else (None, gap_s)


def command_stage_alignments(input_stage, output_stage, *, epsilon=COMMAND_CHAIN_EPSILON):
    """Compare velocity stages using only causal prior samples."""
    alignments = []
    for output in output_stage:
        incoming, gap_s = latest_prior(input_stage, output.stamp_s)
        if incoming is None:
            alignments.append({
                "input_stage": input_stage[0].stage if input_stage else "unavailable",
                "output_stage": output.stage,
                "output_stamp_s": output.stamp_s,
                "alignment_gap_s": gap_s,
                "available": False,
            })
            continue
        linear_delta = output.linear_x_mps - incoming.linear_x_mps
        angular_delta = output.angular_z_rps - incoming.angular_z_rps
        alignments.append({
            "input_stage": incoming.stage,
            "output_stage": output.stage,
            "input_stamp_s": incoming.stamp_s,
            "output_stamp_s": output.stamp_s,
            "alignment_gap_s": gap_s,
            "linear_delta_mps": linear_delta,
            "angular_delta_rps": angular_delta,
            "divergent": (
                abs(linear_delta) > epsilon or abs(angular_delta) > epsilon
            ),
            "available": True,
        })
    return tuple(alignments)


def saturation_intervals(statuses, max_gap_s=COMMAND_CHAIN_MAX_ALIGNMENT_GAP_S):
    """Count observed saturation intervals without extrapolating data gaps."""
    intervals, duration_s = 0, 0.0
    previous = None
    for current in sorted(statuses, key=lambda item: item.stamp_s):
        if current.steer_saturated:
            contiguous = (
                previous is not None
                and previous.steer_saturated
                and 0.0 <= current.stamp_s - previous.stamp_s <= max_gap_s
            )
            if contiguous:
                duration_s += current.stamp_s - previous.stamp_s
            else:
                intervals += 1
        previous = current
    return {"interval_count": intervals, "observed_duration_s": duration_s}


def steering_margin_summary(statuses):
    """Summarize remaining autonomous steering authority from status samples."""
    margins = [
        max(0.0, item.steering_limit_used_rad - abs(item.applied_steer_rad))
        for item in statuses
        if item.steering_limit_used_rad > 0.0
    ]
    if not margins:
        return {
            "sample_count": 0, "minimum_rad": None, "p05_rad": None,
            "above_90pct_limit_fraction": None,
        }
    near_limit = sum(
        abs(item.applied_steer_rad) >= .9 * item.steering_limit_used_rad
        for item in statuses if item.steering_limit_used_rad > 0.0
    )
    return {
        "sample_count": len(margins), "minimum_rad": min(margins),
        "p05_rad": _percentile(margins, .05),
        "above_90pct_limit_fraction": near_limit / len(margins),
    }


def first_divergent_stage(*alignment_groups):
    """Return the first observed command boundary that changed a Twist command."""
    for group in alignment_groups:
        for alignment in group:
            if alignment.get("divergent"):
                return alignment["output_stage"]
    return None
