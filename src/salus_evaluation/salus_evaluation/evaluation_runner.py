"""Thin ROS collector and runner for the pure navigation evaluation domain."""

from __future__ import annotations

import math
import json
from pathlib import Path

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Point, PoseStamped, Twist
from nav2_msgs.action import NavigateThroughPoses
from nav_msgs.msg import Odometry, Path as NavPath
from rclpy.node import Node
from salus_interfaces.msg import (CmdVelFinal, DriveTelemetry, NavEvent,
                                  NavTelemetry, VehicleCommand)
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray
from salus_navigation.route_geometry import path_geometry_metrics

from .artifacts import write_artifacts
from .chunk_continuity_runner import (
    _transition_metrics as _chunk_transition_metrics,
    _wide_turn_local_route,
)
from .gates import GateState, functional_gates, performance_gate
from .geometry_quality import quality_metrics, valid_fillet_r4
from .metrics import (absolute_goal, arrival_metrics, command_response_sign,
                      command_stage_alignments, expected_turn_from_path,
                      first_divergent_stage, latest_prior,
                      covariance_summary, localization_metrics, angle_delta,
                      saturation_intervals,
                      steering_margin_summary,
                      tracking_metrics, trial_data_finite)
from .models import (ExpectedTurn, Pose2D, TimedCommand,
                     TimedControllerStatus, TimedControllerTelemetry,
                     TimedDriveTelemetry, TimedFinalCommand,
                     TimedPose, TimedVehicleCommand)
from .schema import load_scenario


def _stamp(message):
    stamp = message.header.stamp if hasattr(message, "header") else message.stamp
    return float(stamp.sec) + float(stamp.nanosec) / 1e9


def _yaw(quaternion):
    return math.atan2(2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
                      1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z))


def _timed_odometry(message):
    pose = message.pose.pose
    covariance = message.pose.covariance
    return TimedPose(_stamp(message), Pose2D(pose.position.x, pose.position.y,
                                             _yaw(pose.orientation)),
                     message.twist.twist.linear.x, message.twist.twist.angular.z,
                     covariance[0] if len(covariance) > 0 else None,
                     covariance[7] if len(covariance) > 7 else None,
                     covariance[35] if len(covariance) > 35 else None)


def _now_s(node):
    return node.get_clock().now().nanoseconds / 1e9


def _map_xy(spawn, point):
    """Transform an evaluation-local XY point into the scenario map frame."""
    cosine, sine = math.cos(spawn.yaw_rad), math.sin(spawn.yaw_rad)
    return (spawn.x_m + cosine * point[0] - sine * point[1],
            spawn.y_m + sine * point[0] + cosine * point[1])


def _map_yaw(spawn, yaw_rad):
    """Transform an evaluation-local heading into the scenario map frame."""
    return spawn.yaw_rad + yaw_rad


def _heading(first, second):
    """Return the heading from one finite evaluation-local point to another."""
    return math.atan2(second[1] - first[1], second[0] - first[0])


def _track3_geometry(spawn, variant):
    """Build the frozen TRACK3 T1 requests from the route-level fixture."""
    p0, p1, p2, p3 = _wide_turn_local_route()
    incoming_yaw = _heading(p0, p1)
    outgoing_yaw = _heading(p1, p2)
    fillet = valid_fillet_r4(p0, p1, p2, radius_m=4.0)
    if not fillet["valid"]:
        raise ValueError(f"TRACK3 R4 fillet is invalid: {fillet['reason']}")
    entry = fillet["tangent_entry"]
    exit_point = fillet["tangent_exit"]
    # Exact first synthetic from route_preparation.expand for spacing 1.0 m.
    leg_length = math.hypot(p2[0] - p1[0], p2[1] - p1[1])
    spacing = 1.0
    s1 = (
        p1[0] + spacing * (p2[0] - p1[0]) / leg_length,
        p1[1] + spacing * (p2[1] - p1[1]) / leg_length,
    )
    if variant == "track3_current_boundary":
        request_local_poses = (
            ((entry, incoming_yaw), (p1, incoming_yaw)),
            ((s1, outgoing_yaw), (p2, outgoing_yaw)),
        )
    elif variant == "track3_sparse_exit":
        request_local_poses = (
            ((entry, incoming_yaw), (exit_point, outgoing_yaw)),
            ((p2, outgoing_yaw),),
        )
    else:
        raise ValueError(f"unknown TRACK3 variant: {variant}")
    request_poses = tuple(tuple(
        (_map_xy(spawn, point), _map_yaw(spawn, heading))
        for point, heading in request
    ) for request in request_local_poses)

    def mapped(point):
        xy = _map_xy(spawn, point)
        return {"x_m": xy[0], "y_m": xy[1]}
    logical_points = {
        name: {**mapped(point), "yaw_rad": _map_yaw(spawn, heading)}
        for name, point, heading in (
            ("P0", p0, incoming_yaw), ("P1", p1, outgoing_yaw),
            ("P2", p2, outgoing_yaw), ("P3", p3, outgoing_yaw),
            ("E1", entry, incoming_yaw), ("X1", exit_point, outgoing_yaw),
            ("S1", s1, outgoing_yaw),
        )
    }
    return {
        "request_poses": request_poses,
        "poses": tuple(item for request in request_poses for item in request),
        "common_reference": tuple(_map_xy(spawn, point) for point in (p0, p1, p2, p3)),
        "arm_reference": tuple(_map_xy(spawn, point) for point in (p0, p1, p2, p3)),
        "fillet": fillet,
        "track3_nominal_radius_m": 8.0,
        "planner_minimum_turning_radius_m": 4.0,
        "logical_points": logical_points,
    }


def _exact_productive_replay_geometry():
    """Load captured productive poses without regenerating their geometry."""
    replay = (
        Path(get_package_share_directory("salus_evaluation"))
        / "config" / "replays" / "issue244_t0_rep02_chunk_b.json"
    )
    payload = json.loads(replay.read_text(encoding="utf-8"))
    requests = tuple(
        tuple(
            ((float(point[0]), float(point[1])), math.radians(float(yaw)))
            for point, yaw in zip(item["poses_xy"], item["yaws_deg"])
        )
        for item in payload["requests"]
    )
    return {
        "request_poses": requests,
        "poses": tuple(point for request in requests for point in request),
        "common_reference": tuple(
            point for request in requests for point, _yaw in request
        ),
        "arm_reference": tuple(
            point for request in requests for point, _yaw in request
        ),
        "track3_nominal_radius_m": None,
        "planner_minimum_turning_radius_m": 4.0,
        "logical_points": None,
        "replay_source": payload,
    }


def _request_variant_from_replay(variant):
    """Build one allowed B delta while preserving replay request A exactly."""
    replay = _exact_productive_replay_geometry()
    source_requests = replay["request_poses"]
    assert len(source_requests) == 2
    request_a, full5 = source_requests
    assert len(request_a) == 6 and len(full5) == 5
    if variant == "track3_request_b_full5":
        indices = (0, 1, 2, 3, 4)
        request_b = full5
        changed_fields = ()
    elif variant == "track3_request_b_endpoints_only":
        indices = (0, 4)
        request_b = (full5[0], full5[4])
        changed_fields = ("B1-B3 removed",)
    elif variant == "track3_request_b_normalize_b0_yaw":
        indices = (0, 1, 2, 3, 4)
        request_b = ((full5[0][0], full5[1][1]),) + full5[1:]
        changed_fields = ("B0.yaw := B1.yaw",)
    elif variant == "track3_request_b_drop_b0":
        indices = (1, 2, 3, 4)
        request_b = full5[1:]
        changed_fields = ("B0 removed",)
    else:
        raise ValueError(f"unknown productive replay delta variant: {variant}")
    assert request_a == source_requests[0]
    if variant == "track3_request_b_endpoints_only":
        assert request_b == (full5[0], full5[4])
    elif variant == "track3_request_b_normalize_b0_yaw":
        assert tuple(item[0] for item in request_b) == tuple(item[0] for item in full5)
        assert request_b[0][1] == full5[1][1]
        assert request_b[1:] == full5[1:]
    elif variant == "track3_request_b_drop_b0":
        assert request_b == full5[1:]
    else:
        assert request_b == full5
    full5_diff = []
    for variant_index, pose in enumerate(request_b):
        full5_index = indices[variant_index]
        source_pose = full5[full5_index]
        full5_diff.append({
            "full5_index": full5_index,
            "variant_index": variant_index,
            "xy_equal": pose[0] == source_pose[0],
            "yaw_equal": pose[1] == source_pose[1],
            "full5_xy": source_pose[0],
            "variant_xy": pose[0],
            "full5_yaw_rad": source_pose[1],
            "variant_yaw_rad": pose[1],
        })
    retained = set(indices)
    removed = [index for index in range(len(full5)) if index not in retained]
    contract = {
        "source": "issue244_t0_rep02_chunk_b.json",
        "request_a_exact": True,
        "full5_pose_count": len(full5),
        "variant": variant,
        "full5_indices_retained": list(indices),
        "full5_indices_removed": removed,
        "full5_structured_diff": full5_diff,
        "changed_fields": list(changed_fields),
        "full5_xy_yaw_unchanged_except_allowed": True,
        "nav2_controller_safety_parameters_changed": False,
    }
    return (request_a, request_b), contract


def _experiment_geometry(spawn, goal_spec, variant):
    """Build sparse evaluation geometry and its optional request partition."""
    if variant == "track3_exact_productive_replay":
        return _exact_productive_replay_geometry()
    if variant.startswith("track3_request_b_"):
        requests, contract = _request_variant_from_replay(variant)
        geometry = _exact_productive_replay_geometry()
        geometry["request_poses"] = requests
        geometry["poses"] = tuple(point for request in requests for point in request)
        geometry["delta_debug_contract"] = contract
        return geometry
    if variant in ("track3_current_boundary", "track3_sparse_exit"):
        return _track3_geometry(spawn, variant)
    p0 = (0.0, 0.0)
    vertex = (goal_spec.forward_m, 0.0)
    p2 = (goal_spec.forward_m, goal_spec.lateral_m)
    fillet = valid_fillet_r4(p0, vertex, p2, radius_m=4.0)
    if not fillet["valid"]:
        raise ValueError(f"VALID_FILLET_R4 cannot represent goal: {fillet['reason']}")
    tangent_entry = fillet["tangent_entry"]
    tangent_exit = fillet["tangent_exit"]
    center = fillet["center"]
    start_angle = math.atan2(
        tangent_entry[1] - center[1], tangent_entry[0] - center[0]
    )
    turn_sign = 1.0 if fillet["deflection_rad"] > 0.0 else -1.0
    midpoint_angle = start_angle + turn_sign * abs(fillet["deflection_rad"]) / 2.0
    midpoint = (
        center[0] + fillet["radius_m"] * math.cos(midpoint_angle),
        center[1] + fillet["radius_m"] * math.sin(midpoint_angle),
    )
    midpoint_yaw = midpoint_angle + turn_sign * math.pi / 2.0
    final_yaw = goal_spec.yaw_offset_rad
    if variant == "hard_vertex_current":
        local_poses = (
            (tangent_entry, 0.0),
            (vertex, 0.0),
            (p2, final_yaw),
        )
        arm_reference = (p0, tangent_entry, vertex, p2)
        request_local_poses = (local_poses,)
    elif variant in ("sparse_fillet_r4", "sparse_single_3"):
        local_poses = (
            (tangent_entry, 0.0),
            (tangent_exit, final_yaw),
            (p2, final_yaw),
        )
        arm_reference = (p0, *fillet["arc_points"], p2)
        request_local_poses = (local_poses,)
    elif variant == "sparse_single_4":
        local_poses = (
            (tangent_entry, 0.0),
            (midpoint, midpoint_yaw),
            (tangent_exit, final_yaw),
            (p2, final_yaw),
        )
        arm_reference = (p0, *fillet["arc_points"], p2)
        request_local_poses = (local_poses,)
    elif variant == "sparse_boundary_exit":
        local_poses = (
            (tangent_entry, 0.0),
            (tangent_exit, final_yaw),
            (p2, final_yaw),
        )
        arm_reference = (p0, *fillet["arc_points"], p2)
        request_local_poses = (local_poses[:2], (local_poses[2],))
    elif variant == "sparse_boundary_midarc":
        local_poses = (
            (tangent_entry, 0.0),
            (midpoint, midpoint_yaw),
            (tangent_exit, final_yaw),
            (p2, final_yaw),
        )
        arm_reference = (p0, *fillet["arc_points"], p2)
        request_local_poses = (local_poses[:2], local_poses[2:])
    else:
        raise ValueError(f"unknown matched geometry variant: {variant}")
    request_poses = tuple(tuple(
        (_map_xy(spawn, point), _map_yaw(spawn, heading))
        for point, heading in request
    ) for request in request_local_poses)
    poses = tuple(item for request in request_poses for item in request)
    common_reference = tuple(_map_xy(spawn, point) for point in (p0, vertex, p2))
    arm_reference = tuple(_map_xy(spawn, point) for point in arm_reference)
    return {
        "poses": poses,
        "request_poses": request_poses,
        "common_reference": common_reference,
        "arm_reference": arm_reference,
        "fillet": fillet,
        "midpoint": (_map_xy(spawn, midpoint), _map_yaw(spawn, midpoint_yaw)),
    }


def _finite_float(value):
    """Accept only finite JSON numbers; malformed data stays observable as absent."""
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return None
    try:
        result = float(value)
    except OverflowError:
        return None
    return result if math.isfinite(result) else None


def _boundary_transition_metrics(request_records, plan_records, reference):
    """Measure the two-request boundary without joining independent plans."""
    plans_by_request = {}
    for record in plan_records:
        request_index = record.get("request_index")
        if request_index is not None:
            plans_by_request.setdefault(request_index, []).append(record)
    plans_a = plans_by_request.get(0, ())
    plans_b = plans_by_request.get(1, ())
    if not plans_a or not plans_b:
        return {"available": False, "reason": "both request plans were not observed"}
    first_plan = tuple(
        (item.x_m, item.y_m) for item in plans_a[-1]["points"]
    )
    second_plan = tuple(
        (item.x_m, item.y_m) for item in plans_b[0]["points"]
    )
    result = _chunk_transition_metrics(first_plan, second_plan, reference)
    request_b = next(
        (item for item in request_records if item["request_index"] == 1), None
    )
    robot_pose = None if request_b is None else request_b.get("robot_pose_at_dispatch")
    first_constraint = None if request_b is None else request_b["poses"][0]
    if robot_pose is not None and first_constraint is not None:
        result["robot_to_first_constraint_B_m"] = math.hypot(
            robot_pose["x_m"] - first_constraint["x_m"],
            robot_pose["y_m"] - first_constraint["y_m"],
        )
        result["robot_yaw_at_dispatch_B_rad"] = robot_pose["yaw_rad"]
        result["heading_final_A_to_robot_rad"] = abs(angle_delta(
            result["heading_final_A_rad"], robot_pose["yaw_rad"]
        ))
        result["heading_robot_to_initial_B_rad"] = abs(angle_delta(
            robot_pose["yaw_rad"], result["heading_initial_B_rad"]
        ))
    else:
        result["robot_to_first_constraint_B_m"] = None
        result["robot_yaw_at_dispatch_B_rad"] = None
        result["heading_final_A_to_robot_rad"] = None
        result["heading_robot_to_initial_B_rad"] = None
    result["request_A_result_stamp_s"] = next(
        (item.get("result_stamp_s") for item in request_records
         if item["request_index"] == 0), None
    )
    result["request_B_dispatch_stamp_s"] = next(
        (item.get("dispatch_stamp_s") for item in request_records
         if item["request_index"] == 1), None
    )
    if (result["request_A_result_stamp_s"] is not None and
            result["request_B_dispatch_stamp_s"] is not None):
        result["terminal_A_to_dispatch_B_s"] = (
            result["request_B_dispatch_stamp_s"]
            - result["request_A_result_stamp_s"]
        )
    else:
        result["terminal_A_to_dispatch_B_s"] = None
    result["plan_count_A"] = len(plans_a)
    result["plan_count_B"] = len(plans_b)
    return result


def _status_snapshot(stamp_s, payload):
    command = payload.get("command")
    required = {
        "drive_enabled", "estop", "speed_mps", "brake_pct",
        "requested_linear_x_mps", "requested_angular_z_rps",
        "requested_steer_rad", "applied_steer_rad", "steering_limit_used_rad",
        "steer_saturated", "speed_limited", "min_speed_enforced",
    }
    if not isinstance(command, dict) or not required.issubset(command):
        return None
    booleans = (payload.get("fresh"), command.get("drive_enabled"),
                command.get("estop"), command.get("steer_saturated"),
                command.get("speed_limited"), command.get("min_speed_enforced"))
    numeric_keys = required - {
        "drive_enabled", "estop", "steer_saturated", "speed_limited",
        "min_speed_enforced",
    }
    numeric = {key: _finite_float(command[key]) for key in numeric_keys}
    if not all(isinstance(value, bool) for value in booleans) or any(
            value is None for value in numeric.values()):
        return None
    source = payload.get("source")
    if not isinstance(source, str) or not (
            isinstance(command["brake_pct"], int) and
            not isinstance(command["brake_pct"], bool)):
        return None
    return TimedControllerStatus(
        stamp_s=stamp_s,
        source=source,
        fresh=payload["fresh"], drive_enabled=command["drive_enabled"],
        estop=command["estop"], speed_mps=numeric["speed_mps"],
        brake_pct=int(numeric["brake_pct"]),
        requested_linear_x_mps=numeric["requested_linear_x_mps"],
        requested_angular_z_rps=numeric["requested_angular_z_rps"],
        requested_steer_rad=numeric["requested_steer_rad"],
        applied_steer_rad=numeric["applied_steer_rad"],
        steering_limit_used_rad=numeric["steering_limit_used_rad"],
        steer_saturated=command["steer_saturated"],
        speed_limited=command["speed_limited"],
        min_speed_enforced=command["min_speed_enforced"],
    )


def _telemetry_snapshot(stamp_s, payload):
    command = payload.get("requested_auto_command")
    limits = payload.get("ackermann_limits")
    required_command = {"speed_mps", "requested_steer_rad", "applied_steer_rad"}
    required_limits = {
        "steering_limit_deg", "operational_steering_limit_deg",
        "effective_steering_limit_deg",
    }
    if (
        not isinstance(command, dict)
        or not isinstance(limits, dict)
        or not required_command.issubset(command)
        or not required_limits.issubset(limits)
    ):
        return None
    numeric = {
        key: _finite_float(command[key]) for key in required_command
    }
    numeric.update({key: _finite_float(limits[key]) for key in required_limits})
    if any(value is None for value in numeric.values()):
        return None
    return TimedControllerTelemetry(
        stamp_s=stamp_s,
        requested_speed_mps=numeric["speed_mps"],
        requested_steer_rad=numeric["requested_steer_rad"],
        applied_steer_rad=numeric["applied_steer_rad"],
        steering_limit_deg=numeric["steering_limit_deg"],
        operational_steering_limit_deg=numeric["operational_steering_limit_deg"],
        effective_steering_limit_deg=numeric["effective_steering_limit_deg"],
    )


def _source_counts(samples):
    counts = {}
    for sample in samples:
        key = str(sample.source)
        counts[key] = counts.get(key, 0) + 1
    return counts


def _value_summary(rows, key):
    values = [row[key] for row in rows if row.get(key) is not None]
    if not values:
        return {"count": 0, "last": None, "min": None, "max": None}
    return {"count": len(values), "last": values[-1], "min": min(values), "max": max(values)}


def _alignment_summary(rows):
    available = [row for row in rows if row.get("available")]
    stale = [row for row in rows if not row.get("available") and
             row.get("alignment_gap_s") is not None]
    divergent = [row for row in available if row.get("divergent")]
    return {
        "total": len(rows), "correlated": len(available),
        "unavailable": len(rows) - len(available), "stale": len(stale),
        "divergent": len(divergent),
        "linear_delta_mps": _value_summary(available, "linear_delta_mps"),
        "angular_delta_rps": _value_summary(available, "angular_delta_rps"),
    }


def _histogram(samples, key):
    counts = {}
    for item in samples:
        value = str(getattr(item, key))
        counts[value] = counts.get(value, 0) + 1
    return counts


def _trial_json_error_counts(errors, goal_stamp_s):
    return {
        source: sum(error_source == source and stamp_s >= goal_stamp_s
                    for error_source, stamp_s in errors)
        for source in ("status", "telemetry")
    }


def _command_chain(raw, safe, final, vehicle, drive, status, telemetry):
    """Build observer-only command-chain evidence and derived causal pairings."""
    final_twist = tuple(
        TimedCommand(item.stamp_s, item.linear_x_mps, item.angular_z_rps, "cmd_vel_final")
        for item in final
    )
    raw_safe = command_stage_alignments(raw, safe)
    safe_final = command_stage_alignments(safe, final_twist)
    translations, applied_measurements = [], []
    for item in vehicle:
        previous, gap_s = latest_prior(final_twist, item.stamp_s)
        translations.append({
            "stage": "twist_to_ackermann",
            "stamp_s": item.stamp_s,
            "alignment_gap_s": gap_s,
            "available": previous is not None,
            "final_linear_x_mps": previous.linear_x_mps if previous else None,
            "final_angular_z_rps": previous.angular_z_rps if previous else None,
            "vehicle_speed_mps": item.speed_mps,
            "vehicle_steering_angle_rad": item.steering_angle_rad,
            "vehicle_source": item.source,
            "vehicle_drive_enabled": item.drive_enabled,
            "vehicle_emergency_stop": item.emergency_stop,
            "vehicle_brake_ratio": item.brake_ratio,
        })
    for item in drive:
        command, command_gap_s = latest_prior(status, item.stamp_s)
        requested, requested_gap_s = latest_prior(telemetry, item.stamp_s)
        applied_measurements.append({
            "stage": "ackermann_to_measured",
            "stamp_s": item.stamp_s,
            "status_alignment_gap_s": command_gap_s,
            "telemetry_alignment_gap_s": requested_gap_s,
            "available": command is not None or requested is not None,
            "status_speed_mps": command.speed_mps if command else None,
            "status_requested_steer_rad": command.requested_steer_rad if command else None,
            "status_applied_steer_rad": command.applied_steer_rad if command else None,
            "status_requested_to_applied_steer_delta_rad": (
                command.applied_steer_rad - command.requested_steer_rad
                if command else None
            ),
            "steer_saturated": command.steer_saturated if command else None,
            "telemetry_requested_speed_mps": requested.requested_speed_mps if requested else None,
            "telemetry_requested_steer_rad": requested.requested_steer_rad if requested else None,
            "telemetry_applied_steer_rad": requested.applied_steer_rad if requested else None,
            "effective_steering_limit_deg": (
                requested.effective_steering_limit_deg if requested else None
            ),
            "speed_mps_measured": item.speed_mps_measured if item.speed_valid else None,
            "steer_rad_measured": item.steer_rad_measured if item.steer_valid else None,
            "status_speed_to_measured_delta_mps": (
                item.speed_mps_measured - command.speed_mps
                if command and item.speed_valid else None
            ),
            "status_applied_to_measured_steer_delta_rad": (
                item.steer_rad_measured - command.applied_steer_rad
                if command and item.steer_valid else None
            ),
            "telemetry_requested_to_measured_speed_delta_mps": (
                item.speed_mps_measured - requested.requested_speed_mps
                if requested and item.speed_valid else None
            ),
            "telemetry_applied_to_measured_steer_delta_rad": (
                item.steer_rad_measured - requested.applied_steer_rad
                if requested and item.steer_valid else None
            ),
            "brake_applied_pct": item.brake_applied_pct,
        })
    return {
        "raw_safe": raw_safe,
        "safe_final": safe_final,
        "twist_to_ackermann": tuple(translations),
        "ackermann_to_measured": tuple(applied_measurements),
        "summary": {
            "first_divergent_stage": first_divergent_stage(raw_safe, safe_final),
            "sample_counts": {
                "cmd_vel": len(raw), "cmd_vel_safe": len(safe),
                "cmd_vel_final": len(final), "vehicle_command": len(vehicle),
                "drive_telemetry": len(drive), "controller_status": len(status),
                "controller_telemetry": len(telemetry),
            },
            "cmd_vel_final": {
                "source_counts": _source_counts(final),
                "brake_sample_count": sum(item.brake_pct > 0 for item in final),
                "brake_pct_histogram": _histogram(final, "brake_pct"),
            },
            "vehicle_command": {
                "source_counts": _source_counts(vehicle),
                "drive_enabled_count": sum(item.drive_enabled for item in vehicle),
                "emergency_stop_count": sum(item.emergency_stop for item in vehicle),
                "brake_ratio_histogram": _histogram(vehicle, "brake_ratio"),
            },
            "steering_saturation": saturation_intervals(status),
            "steering_margin": steering_margin_summary(status),
            "alignment": {
                "cmd_vel_to_cmd_vel_safe": _alignment_summary(raw_safe),
                "cmd_vel_safe_to_cmd_vel_final": _alignment_summary(safe_final),
                "twist_to_ackermann": {
                    "total": len(translations),
                    "correlated": sum(row["available"] for row in translations),
                    "unavailable": sum(not row["available"] for row in translations),
                    "stale": sum(
                        not row["available"] and row["alignment_gap_s"] is not None
                        for row in translations
                    ),
                },
                "ackermann_to_measured": {
                    "total": len(applied_measurements),
                    "status_unavailable": sum(
                        row["status_alignment_gap_s"] is None or
                        row["status_speed_mps"] is None for row in applied_measurements
                    ),
                    "telemetry_unavailable": sum(
                        row["telemetry_alignment_gap_s"] is None or
                        row["telemetry_requested_speed_mps"] is None
                        for row in applied_measurements
                    ),
                    "status_stale": sum(
                        row["status_alignment_gap_s"] is not None and
                        row["status_speed_mps"] is None
                        for row in applied_measurements
                    ),
                    "telemetry_stale": sum(
                        row["telemetry_alignment_gap_s"] is not None and
                        row["telemetry_requested_speed_mps"] is None
                        for row in applied_measurements
                    ),
                },
            },
            "ackermann": {
                "requested_to_applied_steer_delta_rad": _value_summary(
                    applied_measurements, "status_requested_to_applied_steer_delta_rad"
                ),
                "status_speed_to_measured_delta_mps": _value_summary(
                    applied_measurements, "status_speed_to_measured_delta_mps"
                ),
                "status_applied_to_measured_steer_delta_rad": _value_summary(
                    applied_measurements, "status_applied_to_measured_steer_delta_rad"
                ),
                "telemetry_requested_to_measured_speed_delta_mps": _value_summary(
                    applied_measurements,
                    "telemetry_requested_to_measured_speed_delta_mps",
                ),
                "telemetry_applied_to_measured_steer_delta_rad": _value_summary(
                    applied_measurements,
                    "telemetry_applied_to_measured_steer_delta_rad",
                ),
            },
            "ackermann_limits": {
                "steering_limit_deg": _value_summary(
                    [vars(item) for item in telemetry], "steering_limit_deg"
                ),
                "operational_steering_limit_deg": _value_summary(
                    [vars(item) for item in telemetry], "operational_steering_limit_deg"
                ),
                "effective_steering_limit_deg": _value_summary(
                    [vars(item) for item in telemetry], "effective_steering_limit_deg"
                ),
            },
        },
    }


class EvaluationRunner(Node):
    """Observe standard ROS topics and persist a reproducible trial result."""

    def __init__(self):
        super().__init__("navigation_evaluation")
        self.declare_parameter("scenario", "")
        self.declare_parameter("output_dir", "")
        self.declare_parameter("mode", "run")
        self.declare_parameter("goal_tolerance_m", 1.2)
        self.declare_parameter("precision_target_m", 0.25)
        self.declare_parameter("observe_timeout_s", 90.0)
        self.declare_parameter("geometry_variant", "hard_vertex_current")
        self.scenario_path = str(self.get_parameter("scenario").value)
        self.output_dir = str(self.get_parameter("output_dir").value)
        self.mode = str(self.get_parameter("mode").value)
        self.tolerance = float(self.get_parameter("goal_tolerance_m").value)
        self.precision_target = float(self.get_parameter("precision_target_m").value)
        self.timeout_s = float(self.get_parameter("observe_timeout_s").value)
        self.geometry_variant = str(self.get_parameter("geometry_variant").value).strip()
        if not self.output_dir:
            raise ValueError("output_dir is required")
        if self.mode not in ("run", "observe"):
            raise ValueError("mode must be run or observe")
        if self.geometry_variant not in (
                "hard_vertex_current", "sparse_fillet_r4", "sparse_single_3",
                "sparse_single_4", "sparse_boundary_exit",
                "sparse_boundary_midarc", "track3_current_boundary",
                "track3_sparse_exit", "track3_exact_productive_replay",
                "track3_request_b_full5", "track3_request_b_endpoints_only",
                "track3_request_b_normalize_b0_yaw", "track3_request_b_drop_b0"):
            raise ValueError(
                "unsupported evaluation geometry variant"
            )
        if self.mode == "run" and not self.scenario_path:
            raise ValueError("scenario is required in run mode")
        if self.tolerance <= 0.0 or self.precision_target <= 0.0:
            raise ValueError("arrival tolerances must be positive")
        self.global_poses, self.raw_poses, self.local_poses = [], [], []
        self.commands, self.safe_commands, self.final_commands = [], [], []
        self.vehicle_commands, self.drive_telemetry = [], []
        self.controller_status, self.controller_telemetry = [], []
        self.controller_json_errors = []
        self.plans, self.events = [], []
        self.goal = None
        self.start_pose = None
        self.goal_sent_s = None
        self.success_s = None
        self.expected_turn = ExpectedTurn.ANY
        self.reverse_allowed = False
        self.terminal_status = None
        self.terminal_received_s = None
        self.last_marker_s = None
        self.telemetry = None
        self.goal_event_baseline = None
        self.geometry_reference = None
        self.common_geometry_reference = None
        self.arm_geometry_reference = None
        self.logical_points = None
        self.track3_nominal_radius_m = None
        self.planner_minimum_turning_radius_m = None
        self.replay_source = None
        self.delta_debug_contract = None
        self.dispatched_poses = ()
        self.request_records = []
        self.plan_records = []
        self._request_pose_sets = ()
        self._active_request_index = None
        self._active_goal_generation = None
        self._direct_goal_client = (
            ActionClient(self, NavigateThroughPoses, "/navigate_through_poses")
            if self.mode == "run" else None
        )
        self._finished = False
        self.exit_code = 1
        self.goal_pub = self.create_publisher(PoseStamped, "/goal_pose", 10)
        self.markers = self.create_publisher(MarkerArray, "/navigation_evaluation/markers", 10)
        self.create_subscription(Odometry, "/odometry/global", self._global, 50)
        self.create_subscription(Odometry, "/odom_raw", self._raw, 50)
        self.create_subscription(Odometry, "/odometry/local", self._local, 50)
        self.create_subscription(Twist, "/cmd_vel", self._command, 50)
        self.create_subscription(Twist, "/cmd_vel_safe", self._safe_command, 50)
        self.create_subscription(CmdVelFinal, "/cmd_vel_final", self._final_command, 50)
        self.create_subscription(
            VehicleCommand, "/vehicle/command_shadow", self._vehicle_command, 50
        )
        self.create_subscription(
            DriveTelemetry, "/controller/drive_telemetry", self._drive_telemetry, 50
        )
        self.create_subscription(String, "/controller/status", self._controller_status, 20)
        self.create_subscription(
            String, "/controller/telemetry", self._controller_telemetry, 20
        )
        self.create_subscription(NavPath, "/plan", self._plan, 10)
        self.create_subscription(NavEvent, "/nav_command_server/events", self._event, 20)
        self.create_subscription(
            NavTelemetry, "/nav_command_server/telemetry", self._telemetry, 20
        )
        self.create_subscription(PoseStamped, "/goal_pose", self._observed_goal, 10)
        self.create_timer(0.1, self._tick)

    def _global(self, message):
        self.global_poses.append(_timed_odometry(message))

    def _raw(self, message):
        self.raw_poses.append(_timed_odometry(message))

    def _local(self, message):
        self.local_poses.append(_timed_odometry(message))

    def _command(self, message):
        self.commands.append(TimedCommand(_now_s(self), message.linear.x, message.angular.z))

    def _safe_command(self, message):
        self.safe_commands.append(
            TimedCommand(_now_s(self), message.linear.x, message.angular.z, "cmd_vel_safe")
        )

    def _final_command(self, message):
        self.final_commands.append(TimedFinalCommand(
            _now_s(self), message.twist.linear.x, message.twist.angular.z,
            int(message.brake_pct), int(message.source),
        ))

    def _vehicle_command(self, message):
        self.vehicle_commands.append(TimedVehicleCommand(
            _stamp(message), int(message.source), bool(message.drive_enabled),
            bool(message.emergency_stop), float(message.brake_ratio),
            float(message.drive.speed), float(message.drive.steering_angle),
        ))

    def _drive_telemetry(self, message):
        self.drive_telemetry.append(TimedDriveTelemetry(
            _stamp(message), bool(message.ready), bool(message.fresh),
            bool(message.drive_enabled), bool(message.estop),
            bool(message.speed_valid), bool(message.steer_valid),
            str(message.control_source), float(message.speed_mps_measured),
            math.radians(float(message.steer_deg_measured)),
            int(message.brake_applied_pct),
        ))

    def _controller_status(self, message):
        stamp_s = _now_s(self)
        try:
            payload = json.loads(message.data)
        except (TypeError, json.JSONDecodeError):
            self.controller_json_errors.append(("status", stamp_s))
            return
        snapshot = _status_snapshot(stamp_s, payload) if isinstance(payload, dict) else None
        if snapshot is None:
            self.controller_json_errors.append(("status", stamp_s))
        else:
            self.controller_status.append(snapshot)

    def _controller_telemetry(self, message):
        stamp_s = _now_s(self)
        try:
            payload = json.loads(message.data)
        except (TypeError, json.JSONDecodeError):
            self.controller_json_errors.append(("telemetry", stamp_s))
            return
        snapshot = (
            _telemetry_snapshot(stamp_s, payload)
            if isinstance(payload, dict) else None
        )
        if snapshot is None:
            self.controller_json_errors.append(("telemetry", stamp_s))
        else:
            self.controller_telemetry.append(snapshot)

    def _plan(self, message):
        points = tuple(Pose2D(item.pose.position.x, item.pose.position.y,
                              _yaw(item.pose.orientation))
                       for item in message.poses)
        if points:
            self.plans.append(points)
            self.plan_records.append({
                "stamp_s": _stamp(message),
                "request_index": self._active_request_index,
                "goal_generation": self._active_goal_generation,
                "points": points,
            })
            if (self.mode == "observe" and self.goal is not None and
                    self.expected_turn == ExpectedTurn.ANY and
                    self.start_pose is not None):
                try:
                    self.expected_turn = expected_turn_from_path(
                        self.start_pose, points
                    )
                except ValueError:
                    pass

    def _event(self, message):
        self.events.append((message.code, _stamp(message)))
        if self._direct_goal_client is not None:
            return
        if message.code == "GOAL_RESULT_SUCCEEDED":
            self.success_s, self.terminal_status = _stamp(message), GoalStatus.STATUS_SUCCEEDED
        elif message.code in ("GOAL_RESULT_ABORTED", "GOAL_CANCELLED"):
            self.terminal_status = (GoalStatus.STATUS_ABORTED if message.code.endswith("ABORTED")
                                    else GoalStatus.STATUS_CANCELED)
        if self.terminal_status is not None:
            self.terminal_received_s = self.get_clock().now().nanoseconds / 1e9

    def _telemetry(self, message):
        self.telemetry = message
        terminal = int(message.nav_result_status)
        is_new_result = (
            self.goal_event_baseline is not None and
            int(message.nav_result_event_id) > self.goal_event_baseline
        )
        if is_new_result and terminal in (
                GoalStatus.STATUS_SUCCEEDED, GoalStatus.STATUS_ABORTED,
                GoalStatus.STATUS_CANCELED):
            self.terminal_status = terminal
            now = self.get_clock().now().nanoseconds / 1e9
            self.terminal_received_s = now
            if terminal == GoalStatus.STATUS_SUCCEEDED and self.success_s is None:
                self.success_s = now

    def _observed_goal(self, message):
        if self.mode != "observe" or self.goal is not None:
            return
        if message.header.frame_id.lstrip("/") != "map":
            self.get_logger().warn("ignoring evaluation goal outside map frame")
            return
        self.goal = Pose2D(message.pose.position.x, message.pose.position.y,
                           _yaw(message.pose.orientation))
        self.start_pose = self.global_poses[-1].pose if self.global_poses else None
        self.terminal_status = None
        self.terminal_received_s = None
        self.goal_event_baseline = (
            int(self.telemetry.nav_result_event_id) if self.telemetry else None
        )
        self.goal_sent_s = self.get_clock().now().nanoseconds / 1e9
        self.get_logger().info("observing RViz goal")

    def _send_scenario_goal(self):
        scenario = load_scenario(self.scenario_path)
        if len(scenario.goals) != 1:
            raise ValueError("v1 runner supports exactly one goal per trial")
        goal_spec = scenario.goals[0]
        self.goal = absolute_goal(scenario.spawn, goal_spec)
        self.expected_turn = goal_spec.expected_turn
        self.reverse_allowed = goal_spec.reverse_allowed
        self.timeout_s = goal_spec.timeout_s
        self.terminal_status = None
        self.terminal_received_s = None
        geometry = _experiment_geometry(
            scenario.spawn, goal_spec, self.geometry_variant
        )
        if self.geometry_variant.startswith("track3_request_b_") or (
                self.geometry_variant == "track3_exact_productive_replay"):
            final_pose = geometry["request_poses"][-1][-1]
            self.goal = Pose2D(final_pose[0][0], final_pose[0][1], final_pose[1])
        self.common_geometry_reference = geometry["common_reference"]
        self.arm_geometry_reference = geometry["arm_reference"]
        self.geometry_reference = self.arm_geometry_reference
        self.logical_points = geometry.get("logical_points")
        self.track3_nominal_radius_m = geometry.get("track3_nominal_radius_m")
        self.planner_minimum_turning_radius_m = geometry.get(
            "planner_minimum_turning_radius_m"
        )
        self.replay_source = geometry.get("replay_source")
        self.delta_debug_contract = geometry.get("delta_debug_contract")
        if not self._direct_goal_client.wait_for_server(timeout_sec=5.0):
            raise RuntimeError("NavigateThroughPoses action server is unavailable")
        self._request_pose_sets = geometry["request_poses"]
        self.dispatched_poses = tuple(
            {"x_m": point[0], "y_m": point[1], "yaw_rad": yaw}
            for request in self._request_pose_sets for point, yaw in request
        )
        self.goal_sent_s = _now_s(self)
        self._send_request(0)

    @staticmethod
    def _pose_record(pose):
        if pose is None:
            return None
        return {
            "stamp_s": pose.stamp_s,
            "x_m": pose.pose.x_m,
            "y_m": pose.pose.y_m,
            "yaw_rad": pose.pose.yaw_rad,
        }

    def _send_request(self, request_index):
        """Send one evaluation request, preserving its causal provenance."""
        poses = self._request_pose_sets[request_index]
        generation = request_index + 1
        dispatch_stamp = _now_s(self)
        self._active_request_index = request_index
        self._active_goal_generation = generation
        self.request_records.append({
            "request_index": request_index,
            "goal_generation": generation,
            "dispatch_stamp_s": dispatch_stamp,
            "poses": tuple(
                {"x_m": point[0], "y_m": point[1], "yaw_rad": yaw}
                for point, yaw in poses
            ),
            "robot_pose_at_dispatch": self._pose_record(
                self.global_poses[-1] if self.global_poses else None
            ),
            "result_status": None,
        })
        action_goal = NavigateThroughPoses.Goal()
        stamp = self.get_clock().now().to_msg()
        for (x_y, yaw) in poses:
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.header.stamp = stamp
            pose.pose.position.x, pose.pose.position.y = x_y
            pose.pose.orientation.z = math.sin(yaw / 2.0)
            pose.pose.orientation.w = math.cos(yaw / 2.0)
            action_goal.poses.append(pose)
        self._direct_goal_client.send_goal_async(
            action_goal
        ).add_done_callback(
            lambda future: self._on_direct_goal_response(future, request_index)
        )

    def _on_direct_goal_response(self, future, request_index):
        """Record the terminal result of the evaluation-only through-poses goal."""
        try:
            handle = future.result()
        except Exception as exc:  # pragma: no cover - exercised by ROS runtime
            self.get_logger().error(f"NavigateThroughPoses request failed: {exc}")
            self._finish_request(request_index, GoalStatus.STATUS_ABORTED)
            return
        if not handle.accepted:
            self.get_logger().error("NavigateThroughPoses goal rejected")
            self._finish_request(request_index, GoalStatus.STATUS_ABORTED)
            return
        handle.get_result_async().add_done_callback(
            lambda result: self._on_direct_goal_result(result, request_index)
        )

    def _finish_request(self, request_index, status):
        for record in reversed(self.request_records):
            if record["request_index"] == request_index:
                record["result_status"] = int(status)
                record["result_stamp_s"] = _now_s(self)
                break
        if status == GoalStatus.STATUS_SUCCEEDED and request_index + 1 < len(
                self._request_pose_sets):
            self._send_request(request_index + 1)
            return
        self.terminal_status = int(status)
        self.terminal_received_s = _now_s(self)
        if self.terminal_status == GoalStatus.STATUS_SUCCEEDED:
            self.success_s = self.terminal_received_s
        self._active_request_index = None
        self._active_goal_generation = None

    def _on_direct_goal_result(self, future, request_index):
        """Record a direct Nav2 action result without relying on gateway telemetry."""
        try:
            response = future.result()
            status = int(response.status)
        except Exception as exc:  # pragma: no cover - exercised by ROS runtime
            self.get_logger().error(f"NavigateThroughPoses result failed: {exc}")
            status = GoalStatus.STATUS_ABORTED
        self._finish_request(request_index, status)

    def _tick(self):
        if self._finished:
            return
        if (self.mode == "run" and self.goal is None and self.global_poses and
                self.telemetry is not None):
            self._send_scenario_goal()
            return
        if self.goal is None or self.goal_sent_s is None:
            return
        now = self.get_clock().now().nanoseconds / 1e9
        if self.last_marker_s is None or now - self.last_marker_s >= 0.5:
            self._publish_markers()
            self.last_marker_s = now
        success_window_complete = (self.terminal_status == GoalStatus.STATUS_SUCCEEDED and
                                   self.terminal_received_s is not None and
                                   now - self.terminal_received_s >= 1.0)
        failed_terminal = self.terminal_status in (GoalStatus.STATUS_ABORTED,
                                                   GoalStatus.STATUS_CANCELED)
        if success_window_complete or failed_terminal or now - self.goal_sent_s > self.timeout_s:
            self._finish("timeout" if self.terminal_status is None else "terminal")

    def _publish_markers(self):
        markers = MarkerArray()
        path = Marker()
        path.header.frame_id = "map"
        path.ns, path.id = "evaluation", 0
        path.type, path.action = Marker.LINE_STRIP, Marker.ADD
        path.scale.x, path.color.a, path.color.g = .03, 1.0, 1.0
        for item in self.global_poses:
            path.points.append(Point(x=item.pose.x_m, y=item.pose.y_m, z=.05))
        markers.markers.append(path)
        self.markers.publish(markers)

    def _finish(self, reason):
        self._finished = True
        global_poses = tuple(
            item for item in self.global_poses if item.stamp_s >= self.goal_sent_s
        )
        raw_poses = tuple(item for item in self.raw_poses if item.stamp_s >= self.goal_sent_s)
        local_poses = tuple(item for item in self.local_poses if item.stamp_s >= self.goal_sent_s)
        commands = tuple(item for item in self.commands if item.stamp_s >= self.goal_sent_s)
        safe_commands = tuple(
            item for item in self.safe_commands if item.stamp_s >= self.goal_sent_s
        )
        final_commands = tuple(
            item for item in self.final_commands if item.stamp_s >= self.goal_sent_s
        )
        vehicle_commands = tuple(
            item for item in self.vehicle_commands if item.stamp_s >= self.goal_sent_s
        )
        drive_telemetry = tuple(
            item for item in self.drive_telemetry if item.stamp_s >= self.goal_sent_s
        )
        controller_status = tuple(
            item for item in self.controller_status if item.stamp_s >= self.goal_sent_s
        )
        controller_telemetry = tuple(
            item for item in self.controller_telemetry if item.stamp_s >= self.goal_sent_s
        )
        controller_json_errors = _trial_json_error_counts(
            self.controller_json_errors, self.goal_sent_s
        )
        observed_plan_records = [
            item for item in self.plan_records
            if item["stamp_s"] >= self.goal_sent_s
        ]
        plan = self.plans[-1] if self.plans else ()
        finite = trial_data_finite(
            self.goal, (global_poses, raw_poses, local_poses), commands, plan
        )
        metrics, arrival, localization = None, None, None
        localization_covariance = covariance_summary(local_poses)
        signs = command_response_sign(commands, raw_poses) if raw_poses else None
        errors = []
        try:
            if global_poses and plan:
                metrics = tracking_metrics(global_poses, plan)
            if global_poses and self.goal:
                arrival = arrival_metrics(global_poses, self.goal, self.tolerance, self.success_s)
            if raw_poses and global_poses:
                localization = localization_metrics(raw_poses, global_poses)
        except ValueError as exc:
            errors.append(str(exc))
        gates = functional_gates(
            finite_data=finite and not errors, plan_present=bool(plan),
            terminal_success=self.terminal_status == GoalStatus.STATUS_SUCCEEDED,
            final_distance_m=arrival.final_distance_m if arrival else float("inf"),
            tolerance_m=self.tolerance, sign_metrics=signs or command_response_sign((), ()),
            reverse_observed=any(command.linear_x_mps < -.01 for command in commands),
            reverse_allowed=self.reverse_allowed, expected_turn=self.expected_turn,
            require_turn_expectation=self.mode == "observe",
        )
        precision = {
            "target_m": self.precision_target,
            "final_error_m": arrival.final_distance_m if arrival else None,
            "target_met": (
                arrival is not None and
                arrival.final_distance_m <= self.precision_target
            ),
            "state": "calibrating",
        }
        logical_p1 = None if self.logical_points is None else self.logical_points.get("P1")
        min_robot_distance_to_logical_p1_m = None
        if logical_p1 is not None and global_poses:
            min_robot_distance_to_logical_p1_m = min(
                math.hypot(item.pose.x_m - logical_p1["x_m"],
                           item.pose.y_m - logical_p1["y_m"])
                for item in global_poses
            )
        command_chain = _command_chain(
            commands, safe_commands, final_commands, vehicle_commands,
            drive_telemetry, controller_status, controller_telemetry,
        )
        reference_start = self.start_pose
        if reference_start is None and global_poses:
            reference_start = global_poses[0].pose
        reference = None
        if self.geometry_reference is not None:
            reference = self.geometry_reference
        elif reference_start is not None and self.goal is not None:
            reference = (
                (reference_start.x_m, reference_start.y_m),
                (self.goal.x_m, self.goal.y_m),
            )
        geometry_quality = []
        common_geometry_quality = []
        arm_plan_geometry = []
        common_plan_geometry = []
        plan_geometry = []
        for index, candidate in enumerate(self.plans):
            provenance = (
                observed_plan_records[index]
                if index < len(observed_plan_records) else {}
            )
            points = tuple((item.x_m, item.y_m) for item in candidate)
            common_quality = quality_metrics(points, self.common_geometry_reference)
            arm_quality = quality_metrics(points, self.arm_geometry_reference)
            geometry_quality.append({
                "plan_index": index,
                "request_index": provenance.get("request_index"),
                "goal_generation": provenance.get("goal_generation"),
                **arm_quality,
            })
            common_geometry_quality.append({
                "plan_index": index,
                **common_quality,
            })
            common_geometry = path_geometry_metrics(
                points, self.common_geometry_reference or reference or points
            )
            arm_geometry = path_geometry_metrics(
                points, self.arm_geometry_reference or reference or points
            )
            common_row = {
                "plan_index": index,
                "request_index": provenance.get("request_index"),
                "goal_generation": provenance.get("goal_generation"),
                "length_m": common_geometry.length_m,
                "direct_distance_m": common_geometry.direct_distance_m,
                "detour_ratio": common_geometry.detour_ratio,
                "max_deviation_m": common_geometry.max_deviation_m,
                "self_intersections": common_geometry.self_intersections,
            }
            common_plan_geometry.append(common_row)
            arm_plan_geometry.append({
                "plan_index": index,
                "request_index": provenance.get("request_index"),
                "goal_generation": provenance.get("goal_generation"),
                "length_m": arm_geometry.length_m,
                "direct_distance_m": arm_geometry.direct_distance_m,
                "detour_ratio": arm_geometry.detour_ratio,
                "max_deviation_m": arm_geometry.max_deviation_m,
                "self_intersections": arm_geometry.self_intersections,
            })
            plan_geometry.append(common_row)
        summary = {"schema_version": 2, "reason": reason,
                   "geometry_variant": self.geometry_variant,
                   "geometry_contract": {
                       "logical_points": self.logical_points,
                       "track3_nominal_radius_m": self.track3_nominal_radius_m,
                       "planner_minimum_turning_radius_m": (
                           self.planner_minimum_turning_radius_m
                       ),
                       "replay_source": self.replay_source,
                       "delta_debug_contract": self.delta_debug_contract,
                   },
                   "min_robot_distance_to_logical_P1_m": (
                       min_robot_distance_to_logical_p1_m
                   ),
                   "terminal_status": self.terminal_status,
                   "goal": self.goal, "metrics": metrics, "arrival": arrival,
                   "operational_tolerance_m": self.tolerance,
                   "precision": precision,
                   "localization": localization,
                   "localization_covariance": localization_covariance,
                   "geometry_quality": geometry_quality,
                   "common_geometry_quality": common_geometry_quality,
                   "plan_geometry": plan_geometry,
                   "common_plan_geometry": common_plan_geometry,
                   "arm_plan_geometry": arm_plan_geometry,
                   "dispatched_poses": self.dispatched_poses,
                   "request_records": self.request_records,
                   "plan_records": observed_plan_records,
                   "boundary_transition": _boundary_transition_metrics(
                       self.request_records, observed_plan_records,
                       self.arm_geometry_reference or reference or ()
                   ),
                   "sign": signs, "gates": gates,
                   "performance": [performance_gate(
                       "cross_track_p95_m",
                       metrics.cross_track_p95_m if metrics else float("inf"),
                   )],
                   "errors": errors, "replans": max(0, len(self.plans) - 1),
                   "command_chain": command_chain["summary"],
                   "controller_json_errors": controller_json_errors}
        manifest = {
            "schema_version": 2, "goal_stamp_s": self.goal_sent_s,
            "mode": self.mode, "scenario": self.scenario_path,
            "streams": [
                "odometry_global", "odometry_raw", "odometry_local", "commands",
                "commands_safe", "commands_final", "vehicle_commands",
                "drive_telemetry", "controller_status", "controller_telemetry",
                "command_chain_alignment",
            ],
            "topics": [
                "/plan", "/cmd_vel", "/cmd_vel_safe", "/cmd_vel_final",
                "/vehicle/command_shadow", "/controller/drive_telemetry",
                "/controller/status", "/controller/telemetry", "/odom_raw",
                "/odometry/local", "/odometry/global",
            ],
            "geometry_variant": self.geometry_variant,
            "geometry_contract": {
                "logical_points": self.logical_points,
                "track3_nominal_radius_m": self.track3_nominal_radius_m,
                "planner_minimum_turning_radius_m": (
                    self.planner_minimum_turning_radius_m
                ),
                "replay_source": self.replay_source,
                "delta_debug_contract": self.delta_debug_contract,
            },
            "min_robot_distance_to_logical_P1_m": min_robot_distance_to_logical_p1_m,
            "dispatched_poses": self.dispatched_poses,
            "request_count": len(self.request_records),
            "request_records": self.request_records,
            "reference_families": ("common_mission", "arm_target"),
        }
        streams = {"odometry_global": global_poses, "odometry_raw": raw_poses,
                   "odometry_local": local_poses, "commands": commands,
                   "commands_safe": safe_commands, "commands_final": final_commands,
                   "vehicle_commands": vehicle_commands,
                   "drive_telemetry": drive_telemetry,
                   "controller_status": controller_status,
                   "controller_telemetry": controller_telemetry,
                   "request_records": self.request_records,
                   "plan_records": observed_plan_records,
                   "command_chain_alignment": (
                       command_chain["raw_safe"] + command_chain["safe_final"]
                       + command_chain["twist_to_ackermann"]
                       + command_chain["ackermann_to_measured"]
                   )}
        write_artifacts(self.output_dir, manifest, summary, streams)
        self.exit_code = int(any(gate.state == GateState.FAIL for gate in gates))
        self._publish_markers()
        self.get_logger().info(f"evaluation complete: {Path(self.output_dir) / 'summary.json'}")


def main():
    rclpy.init()
    node = EvaluationRunner()
    try:
        while rclpy.ok() and not node._finished:
            rclpy.spin_once(node)
    finally:
        exit_code = node.exit_code
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return exit_code
