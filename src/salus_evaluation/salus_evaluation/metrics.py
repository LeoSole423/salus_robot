"""Pure geometry and navigation metric calculations."""

import math

from .models import (ArrivalMetrics, LocalizationMetrics, Pose2D, SignMetrics,
                     TrackingMetrics)


def angle_delta(a, b):
    """Return signed shortest angular difference a-b."""
    return math.atan2(math.sin(a - b), math.cos(a - b))


def absolute_goal(spawn, goal):
    """Transform a relative goal from vehicle coordinates into world coordinates."""
    cosine, sine = math.cos(spawn.yaw_rad), math.sin(spawn.yaw_rad)
    return Pose2D(spawn.x_m + cosine * goal.forward_m - sine * goal.lateral_m,
                  spawn.y_m + sine * goal.forward_m + cosine * goal.lateral_m,
                  spawn.yaw_rad + goal.yaw_offset_rad)


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
    for command in eligible:
        response_stamp = command.stamp_s + response_delay_s
        response = min(poses, key=lambda pose: abs(pose.stamp_s - response_stamp))
        if command.angular_z_rps * response.angular_z_rps < 0:
            mismatches += 1
    first_sign = (1 if eligible[0].angular_z_rps > 0 else -1) if eligible else 0
    return SignMetrics(len(eligible), mismatches,
                       mismatches / len(eligible) if eligible else None,
                       first_sign)


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


def localization_metrics(ground_truth, estimates):
    """Compare estimates with nearest ground-truth samples."""
    if not estimates or not ground_truth:
        raise ValueError("localization requires both streams")
    errors, yaw_errors = [], []
    for estimate in estimates:
        truth = min(ground_truth, key=lambda item: abs(item.stamp_s-estimate.stamp_s))
        errors.append(math.hypot(estimate.pose.x_m-truth.pose.x_m,
                                 estimate.pose.y_m-truth.pose.y_m))
        yaw_errors.append(abs(angle_delta(estimate.pose.yaw_rad, truth.pose.yaw_rad)))
    return LocalizationMetrics(len(errors), math.sqrt(sum(x*x for x in errors)/len(errors)),
                               _percentile(errors, .95),
                               math.sqrt(sum(x*x for x in yaw_errors)/len(yaw_errors)),
                               errors[-1])
