#!/usr/bin/env python3
"""Capture one bounded, read-only timestamp/receipt window from the ROS graph.

The probe records compact metadata only.  It never publishes, calls a service
or action, broadcasts TF, or starts a launch.  Stamped messages retain their
source timestamp; every row also contains ROS time, wall time and steady time
at receipt.  Large payloads such as point clouds are represented by shape and
frame metadata so a physical window stays small.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import os
import re
import signal
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Iterable, Sequence


CSV_FIELDS = (
    "sequence",
    "topic",
    "message_kind",
    "source_stamp_ns",
    "receipt_ros_ns",
    "receipt_wall_ns",
    "receipt_steady_ns",
    "steady_since_start_s",
    "age_s",
    "source_frame",
    "child_frame",
    "tf_parent",
    "tf_child",
    "x_m",
    "y_m",
    "yaw_rad",
    "linear_x_mps",
    "angular_z_rps",
    "brake_pct",
    "command_source",
    "heading_valid",
    "heading_reason",
    "heading_source",
    "event_code",
    "event_component",
    "event_message",
    "event_id",
    "event_details",
    "causal_signal",
    "log_stamp_ns",
    "log_node",
    "log_level",
    "log_message",
    "goal_active",
    "active_action",
    "nav_result_status",
    "nav_result_text",
    "failure_code",
    "failure_component",
    "cmd_vel_safe_fresh",
    "cmd_vel_safe_age_s",
    "collision_stop_active",
    "plan_poses",
    "action_name",
    "action_statuses",
    "lifecycle_node",
    "lifecycle_start",
    "lifecycle_goal",
    "sample_count",
    "path_signature",
)

EXPECTED_TOPICS = (
    "/scan_3d",
    "/obstacles_cloud",
    "/scan",
    "/scan_clean",
    "/cmd_vel_safe",
    "/cmd_vel_final",
    "/gps/course_heading/debug",
    "/localization/orientation_selection/debug",
    "/gps/course_heading",
    "/localization/orientation",
    "/odometry/global",
    "/tf",
    "/tf_static",
    "/plan",
    "/nav_command_server/events",
    "/nav_command_server/telemetry",
    "/navigate_to_pose/_action/status",
    "/navigate_through_poses/_action/status",
    "/planner_server/transition_event",
    "/controller_server/transition_event",
    "/bt_navigator/transition_event",
    "/behavior_server/transition_event",
    "/rosout",
)

POINTCLOUD_TOPICS = ("/scan_3d", "/obstacles_cloud")
COLLISION_MONITOR_TIMESTAMP_IGNORE_SIGNAL = (
    "collision_monitor_source_ignored_by_timestamp"
)
_COLLISION_MONITOR_TIMESTAMP_IGNORE = re.compile(
    r"(?:latest\s+source.*current\s+collision\s+monitor\s+node\s+"
    r"timestamps?\s+differ.*ignoring\s+the\s+source|"
    r"(?:source|observation).{0,120}(?:timestamp|time).{0,120}"
    r"(?:ignor\w*|drop\w*|discard\w*|stale|timeout))",
    re.IGNORECASE | re.DOTALL,
)


def is_collision_monitor_timestamp_ignore_signal(
    node_name: str, message: str
) -> bool:
    """Match Nav2 Humble's direct source-timestamp ignore warning only."""
    normalized_node = str(node_name).strip("/").split("/")[-1]
    return normalized_node == "collision_monitor" and bool(
        _COLLISION_MONITOR_TIMESTAMP_IGNORE.search(str(message))
    )


def resolve_pointcloud_topics(
    mode: str, requested_topics: Sequence[str] | None = None
) -> tuple[str, ...]:
    """Resolve staged PointCloud2 capture without changing the ROS runtime.

    ``none`` is the low-impact baseline, ``selected`` defaults to the primary
    normalized cloud, and ``all`` is an explicit full-capture opt-in.
    """
    requested = tuple(requested_topics or ())
    unknown = set(requested) - set(POINTCLOUD_TOPICS)
    if unknown:
        raise ValueError(f"unsupported PointCloud2 topic(s): {sorted(unknown)}")
    if mode == "none":
        if requested:
            raise ValueError("--pointcloud-topic requires --pointcloud-mode selected")
        return ()
    if mode == "all":
        if requested:
            raise ValueError("--pointcloud-topic cannot be combined with --pointcloud-mode all")
        return POINTCLOUD_TOPICS
    if mode == "selected":
        return tuple(dict.fromkeys(requested or ("/scan_3d",)))
    raise ValueError(f"unsupported PointCloud2 capture mode: {mode}")


def age_seconds(receipt_ros_ns: int | None, source_stamp_ns: int | None) -> float | None:
    """Return signed ROS-time age, or ``None`` for unstamped/zero-time data."""
    if receipt_ros_ns is None or source_stamp_ns is None:
        return None
    if int(receipt_ros_ns) <= 0 or int(source_stamp_ns) <= 0:
        return None
    return (int(receipt_ros_ns) - int(source_stamp_ns)) / 1_000_000_000.0


def timestamp_gap_seconds(previous_ns: int | None, current_ns: int | None) -> float | None:
    """Return a signed gap between two samples without hiding regressions."""
    if previous_ns is None or current_ns is None:
        return None
    return (int(current_ns) - int(previous_ns)) / 1_000_000_000.0


def interpolate_scalar(
    samples: Sequence[tuple[int, float]], target_stamp_ns: int
) -> float | None:
    """Linearly interpolate a scalar inside a stamped sample window.

    Exact samples are returned directly.  Extrapolation and non-finite input
    are rejected, which keeps offline correlations from inventing state before
    the first or after the last observed sample.
    """
    if not samples or not math.isfinite(float(target_stamp_ns)):
        return None
    previous: tuple[int, float] | None = None
    for stamp_ns, value in samples:
        stamp_ns, value = int(stamp_ns), float(value)
        if stamp_ns <= 0 or not math.isfinite(value):
            return None
        if stamp_ns == int(target_stamp_ns):
            return value
        if previous is not None:
            previous_stamp, previous_value = previous
            if previous_stamp > stamp_ns:
                return None
            if previous_stamp < int(target_stamp_ns) < stamp_ns:
                fraction = (int(target_stamp_ns) - previous_stamp) / (
                    stamp_ns - previous_stamp
                )
                return previous_value + fraction * (value - previous_value)
        previous = (stamp_ns, value)
    return None


def summarize_rows(
    rows: Iterable[dict[str, Any]], expected_topics: Iterable[str] = ()
) -> dict[str, dict[str, Any]]:
    """Summarize receipt/source gaps while preserving per-message rows."""
    grouped: dict[str, list[dict[str, Any]]] = {}
    for row in rows:
        grouped.setdefault(str(row["topic"]), []).append(row)
    result: dict[str, dict[str, Any]] = {}
    for topic, topic_rows in grouped.items():
        receipt_gaps = [
            timestamp_gap_seconds(a["receipt_steady_ns"], b["receipt_steady_ns"])
            for a, b in zip(topic_rows, topic_rows[1:])
        ]
        source_rows = [
            row for row in topic_rows if row.get("source_stamp_ns") not in (None, 0)
        ]
        source_gaps = [
            timestamp_gap_seconds(a["source_stamp_ns"], b["source_stamp_ns"])
            for a, b in zip(source_rows, source_rows[1:])
        ]
        result[topic] = {
            "messages": len(topic_rows),
            "first_source_stamp_ns": source_rows[0]["source_stamp_ns"]
            if source_rows
            else None,
            "last_source_stamp_ns": source_rows[-1]["source_stamp_ns"]
            if source_rows
            else None,
            "latest_age_s": topic_rows[-1].get("age_s"),
            "max_receipt_gap_s": max(
                (gap for gap in receipt_gaps if gap is not None), default=None
            ),
            "max_source_gap_s": max(
                (gap for gap in source_gaps if gap is not None and gap >= 0.0),
                default=None,
            ),
            "source_backwards_or_equal": sum(
                1 for gap in source_gaps if gap is not None and gap <= 0.0
            ),
        }
    for topic in expected_topics:
        result.setdefault(
            topic,
            {
                "messages": 0,
                "first_source_stamp_ns": None,
                "last_source_stamp_ns": None,
                "latest_age_s": None,
                "max_receipt_gap_s": None,
                "max_source_gap_s": None,
                "source_backwards_or_equal": 0,
            },
        )
    return result


def csv_row(row: dict[str, Any]) -> dict[str, Any]:
    """Convert structured list/dict fields into stable compact CSV cells."""
    result = {field: row.get(field) for field in CSV_FIELDS}
    for field in ("action_statuses", "event_details"):
        if result[field] is not None:
            result[field] = json.dumps(result[field], separators=(",", ":"), sort_keys=True)
    return result


def write_report(report: dict[str, Any], json_out: str, csv_out: str) -> None:
    """Write JSON and a provenance-prefixed CSV atomically enough for a probe."""
    if json_out:
        path = Path(json_out)
        path.parent.mkdir(parents=True, exist_ok=True)
        temporary = path.with_suffix(path.suffix + ".tmp")
        temporary.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
        temporary.replace(path)
    if csv_out:
        path = Path(csv_out)
        path.parent.mkdir(parents=True, exist_ok=True)
        temporary = path.with_suffix(path.suffix + ".tmp")
        with temporary.open("w", encoding="utf-8", newline="") as handle:
            for key in ("tool", "schema_version", "source_sha", "source_branch", "capture_mode"):
                value = report["provenance"].get(key, report.get(key))
                handle.write(f"# {key}={value}\n")
            writer = csv.DictWriter(handle, fieldnames=CSV_FIELDS, extrasaction="ignore")
            writer.writeheader()
            writer.writerows(csv_row(row) for row in report["rows"])
        temporary.replace(path)


def _yaw(quaternion: Any) -> float:
    return math.atan2(
        2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
        1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z),
    )


def _path_signature(message: Any) -> str:
    if not message.poses:
        return "0"
    indices = sorted({0, len(message.poses) // 2, len(message.poses) - 1})
    points = [
        [
            round(float(message.poses[index].pose.position.x), 3),
            round(float(message.poses[index].pose.position.y), 3),
        ]
        for index in indices
    ]
    return json.dumps([len(message.poses), points], separators=(",", ":"))


def run_capture(args: argparse.Namespace) -> dict[str, Any]:
    """Run exactly one bounded subscription window."""
    import rclpy
    from action_msgs.msg import GoalStatusArray
    from geometry_msgs.msg import Twist
    from lifecycle_msgs.msg import TransitionEvent
    from nav_msgs.msg import Odometry, Path as NavPath
    from rclpy.node import Node
    from rclpy.qos import (
        DurabilityPolicy,
        HistoryPolicy,
        QoSProfile,
        ReliabilityPolicy,
        qos_profile_sensor_data,
    )
    from salus_interfaces.msg import CmdVelFinal, NavEvent, NavTelemetry
    from sensor_msgs.msg import Imu, LaserScan, PointCloud2
    from std_msgs.msg import String
    from tf2_msgs.msg import TFMessage

    # Reuse the existing timestamp/frame helpers used by the runtime sidecar.
    from runtime_timing_probe import normalize_frame, stamp_ns

    class CaptureNode(Node):
        def __init__(self) -> None:
            super().__init__("salus_ros_observability_probe")
            self.started_steady_ns = time.monotonic_ns()
            self.rows: list[dict[str, Any]] = []
            self.sequence = 0

        def record(
            self,
            topic: str,
            message_kind: str,
            source_stamp_ns: int | None = None,
            **fields: Any,
        ) -> None:
            receipt_steady_ns = time.monotonic_ns()
            receipt_ros_ns = int(self.get_clock().now().nanoseconds)
            self.sequence += 1
            row = {field: None for field in CSV_FIELDS}
            row.update(
                {
                    "sequence": self.sequence,
                    "topic": topic,
                    "message_kind": message_kind,
                    "source_stamp_ns": source_stamp_ns if source_stamp_ns else None,
                    "receipt_ros_ns": receipt_ros_ns,
                    "receipt_wall_ns": time.time_ns(),
                    "receipt_steady_ns": receipt_steady_ns,
                    "steady_since_start_s": round(
                        (receipt_steady_ns - self.started_steady_ns) / 1_000_000_000.0,
                        6,
                    ),
                    "age_s": age_seconds(receipt_ros_ns, source_stamp_ns),
                }
            )
            row.update(fields)
            self.rows.append(row)

        def on_pointcloud(self, topic: str, message: Any) -> None:
            self.record(
                topic,
                "PointCloud2",
                stamp_ns(message.header.stamp),
                source_frame=normalize_frame(message.header.frame_id),
                sample_count=int(message.width) * int(message.height),
            )

        def on_scan(self, topic: str, message: Any) -> None:
            self.record(
                topic,
                "LaserScan",
                stamp_ns(message.header.stamp),
                source_frame=normalize_frame(message.header.frame_id),
                sample_count=len(message.ranges),
            )

        def on_twist(self, message: Any) -> None:
            self.record(
                "/cmd_vel_safe",
                "Twist",
                linear_x_mps=float(message.linear.x),
                angular_z_rps=float(message.angular.z),
            )

        def on_final(self, message: Any) -> None:
            self.record(
                "/cmd_vel_final",
                "CmdVelFinal",
                linear_x_mps=float(message.twist.linear.x),
                angular_z_rps=float(message.twist.angular.z),
                brake_pct=int(message.brake_pct),
                command_source=int(message.source),
            )

        def on_imu(self, topic: str, message: Any) -> None:
            self.record(
                topic,
                "Imu",
                stamp_ns(message.header.stamp),
                source_frame=normalize_frame(message.header.frame_id),
                yaw_rad=_yaw(message.orientation),
                heading_valid=True,
            )

        def on_heading_debug(self, topic: str, message: Any) -> None:
            fields: dict[str, Any] = {}
            try:
                payload = json.loads(message.data)
            except (TypeError, json.JSONDecodeError):
                payload = {}
            if isinstance(payload, dict):
                fields.update(
                    {
                        "heading_valid": payload.get("valid"),
                        "heading_reason": payload.get("reason"),
                        "heading_source": payload.get("selected_source")
                        or (
                            "course_over_ground"
                            if topic == "/gps/course_heading/debug"
                            else None
                        ),
                    }
                )
            self.record(topic, "String", **fields)

        def on_odom(self, message: Any) -> None:
            pose = message.pose.pose
            self.record(
                "/odometry/global",
                "Odometry",
                stamp_ns(message.header.stamp),
                source_frame=normalize_frame(message.header.frame_id),
                child_frame=normalize_frame(message.child_frame_id),
                x_m=float(pose.position.x),
                y_m=float(pose.position.y),
                yaw_rad=_yaw(pose.orientation),
                linear_x_mps=float(message.twist.twist.linear.x),
                angular_z_rps=float(message.twist.twist.angular.z),
            )

        def on_tf(self, topic: str, message: Any) -> None:
            for transform in message.transforms:
                parent = normalize_frame(transform.header.frame_id)
                child = normalize_frame(transform.child_frame_id)
                if (parent, child) not in {
                    ("map", "odom"),
                    ("odom", "base_footprint"),
                }:
                    continue
                self.record(
                    topic,
                    "Transform",
                    stamp_ns(transform.header.stamp),
                    tf_parent=parent,
                    tf_child=child,
                    x_m=float(transform.transform.translation.x),
                    y_m=float(transform.transform.translation.y),
                    yaw_rad=_yaw(transform.transform.rotation),
                )

        def on_path(self, message: Any) -> None:
            self.record(
                "/plan",
                "Path",
                stamp_ns(message.header.stamp),
                source_frame=normalize_frame(message.header.frame_id),
                plan_poses=len(message.poses),
                path_signature=_path_signature(message),
            )

        def on_event(self, message: Any) -> None:
            self.record(
                "/nav_command_server/events",
                "NavEvent",
                stamp_ns(message.stamp),
                event_code=message.code,
                event_component=message.component,
                event_message=message.message,
                event_id=int(message.event_id),
                event_details={item.key: item.value for item in message.details},
            )

        def on_telemetry(self, message: Any) -> None:
            self.record(
                "/nav_command_server/telemetry",
                "NavTelemetry",
                goal_active=bool(message.goal_active),
                active_action=message.active_action,
                nav_result_status=int(message.nav_result_status),
                nav_result_text=message.nav_result_text,
                failure_code=message.failure_code,
                failure_component=message.failure_component,
                cmd_vel_safe_fresh=bool(message.cmd_vel_safe_fresh),
                cmd_vel_safe_age_s=float(message.cmd_vel_safe_age_s),
                collision_stop_active=bool(message.collision_stop_active),
            )

        def on_action_status(self, action_name: str, message: Any) -> None:
            self.record(
                f"/{action_name}/_action/status",
                "GoalStatusArray",
                stamp_ns(message.header.stamp),
                action_name=action_name,
                action_statuses=[int(status.status) for status in message.status_list],
            )

        def on_transition(self, node_name: str, message: Any) -> None:
            self.record(
                f"/{node_name}/transition_event",
                "TransitionEvent",
                stamp_ns(message.timestamp),
                lifecycle_node=node_name,
                lifecycle_start=message.start_state.label,
                lifecycle_goal=message.goal_state.label,
            )

        def on_rosout(self, message: Any) -> None:
            """Retain only the causal Collision Monitor timestamp warning."""
            if not is_collision_monitor_timestamp_ignore_signal(
                message.name, message.msg
            ):
                return
            log_stamp_ns = stamp_ns(message.stamp)
            self.record(
                "/rosout",
                "CollisionMonitorTimestampIgnore",
                log_stamp_ns,
                causal_signal=COLLISION_MONITOR_TIMESTAMP_IGNORE_SIGNAL,
                log_stamp_ns=log_stamp_ns,
                log_node=str(message.name),
                log_level=int(message.level),
                log_message=str(message.msg),
            )

    node = CaptureNode()
    sensor_topics = {
        "/scan": (LaserScan, lambda message: node.on_scan("/scan", message)),
        "/scan_clean": (LaserScan, lambda message: node.on_scan("/scan_clean", message)),
        "/gps/course_heading/debug": (
            String,
            lambda message: node.on_heading_debug("/gps/course_heading/debug", message),
        ),
        "/localization/orientation_selection/debug": (
            String,
            lambda message: node.on_heading_debug(
                "/localization/orientation_selection/debug", message
            ),
        ),
    }
    for pointcloud_topic in getattr(
        args, "pointcloud_capture_topics", resolve_pointcloud_topics("none")
    ):
        sensor_topics[pointcloud_topic] = (
            PointCloud2,
            lambda message, topic=pointcloud_topic: node.on_pointcloud(topic, message),
        )
    for topic, (message_type, callback) in sensor_topics.items():
        node.create_subscription(message_type, topic, callback, qos_profile_sensor_data)
    node.create_subscription(Twist, "/cmd_vel_safe", node.on_twist, 10)
    node.create_subscription(CmdVelFinal, "/cmd_vel_final", node.on_final, 10)
    node.create_subscription(
        Imu,
        "/gps/course_heading",
        lambda message: node.on_imu("/gps/course_heading", message),
        qos_profile_sensor_data,
    )
    node.create_subscription(
        Imu,
        "/localization/orientation",
        lambda message: node.on_imu("/localization/orientation", message),
        qos_profile_sensor_data,
    )
    node.create_subscription(Odometry, "/odometry/global", node.on_odom, 10)
    node.create_subscription(
        TFMessage,
        "/tf",
        lambda message: node.on_tf("/tf", message),
        qos_profile_sensor_data,
    )
    static_qos = QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=100,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )
    node.create_subscription(
        TFMessage,
        "/tf_static",
        lambda message: node.on_tf("/tf_static", message),
        static_qos,
    )
    node.create_subscription(NavPath, "/plan", node.on_path, 10)
    node.create_subscription(NavEvent, "/nav_command_server/events", node.on_event, 50)
    node.create_subscription(NavTelemetry, "/nav_command_server/telemetry", node.on_telemetry, 10)
    for action_name in ("navigate_to_pose", "navigate_through_poses"):
        node.create_subscription(
            GoalStatusArray,
            f"/{action_name}/_action/status",
            lambda message, name=action_name: node.on_action_status(name, message),
            10,
        )
    for lifecycle_node in (
        "planner_server",
        "controller_server",
        "bt_navigator",
        "behavior_server",
    ):
        node.create_subscription(
            TransitionEvent,
            f"/{lifecycle_node}/transition_event",
            lambda message, name=lifecycle_node: node.on_transition(name, message),
            10,
        )
    from rcl_interfaces.msg import Log

    node.create_subscription(Log, "/rosout", node.on_rosout, 100)

    started_wall = datetime.now(timezone.utc)
    started_steady_ns = node.started_steady_ns
    stop_requested = False

    def request_stop(_signum: int, _frame: Any) -> None:
        nonlocal stop_requested
        stop_requested = True

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)
    deadline = time.monotonic() + float(args.duration_s)
    try:
        while rclpy.ok() and not stop_requested and time.monotonic() < deadline:
            rclpy.spin_once(
                node, timeout_sec=min(0.2, max(0.0, deadline - time.monotonic()))
            )
    finally:
        ended_wall = datetime.now(timezone.utc)
        ended_steady_ns = time.monotonic_ns()
        rows = node.rows
        report = {
            "schema_version": 1,
            "capture_mode": "single_read_only_window",
            "provenance": {
                "tool": "ros_observability_probe",
                "schema_version": 1,
                "source_sha": args.source_sha,
                "source_branch": args.source_branch,
                "capture_mode": "single_read_only_window",
                "pointcloud_capture_mode": getattr(args, "pointcloud_mode", "none"),
                "pointcloud_capture_topics": list(
                    getattr(args, "pointcloud_capture_topics", ())
                ),
                "generated_at": ended_wall.isoformat(timespec="seconds"),
                "ros_domain_id": os.environ.get("ROS_DOMAIN_ID", ""),
                "no_publishers": True,
                "no_services_or_actions_called": True,
                "tf_payload_pairs": ["map -> odom", "odom -> base_footprint"],
            },
            "window": {
                "started_at": started_wall.isoformat(timespec="seconds"),
                "ended_at": ended_wall.isoformat(timespec="seconds"),
                "duration_s": round((ended_steady_ns - started_steady_ns) / 1_000_000_000.0, 6),
                "interrupted": stop_requested,
                "rows": len(rows),
            },
            "topics": summarize_rows(rows, EXPECTED_TOPICS),
            "rows": rows,
        }
        node.destroy_node()
        rclpy.shutdown()
    return report


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--duration-s", type=float, default=30.0)
    parser.add_argument("--json-out", default="")
    parser.add_argument("--csv-out", default="")
    parser.add_argument(
        "--pointcloud-mode",
        choices=("none", "selected", "all"),
        default="none",
        help="staged PointCloud2 capture: none, one selected topic, or all topics",
    )
    parser.add_argument(
        "--pointcloud-topic",
        dest="pointcloud_topics",
        action="append",
        choices=POINTCLOUD_TOPICS,
        help="PointCloud2 topic for selected mode; repeat to capture both",
    )
    parser.add_argument("--source-sha", default=os.environ.get("SOURCE_SHA", "unknown"))
    parser.add_argument("--source-branch", default=os.environ.get("SOURCE_BRANCH", "unknown"))
    args = parser.parse_args(argv)
    if args.duration_s <= 0.0:
        parser.error("--duration-s must be positive")
    if not args.json_out and not args.csv_out:
        parser.error("at least one of --json-out or --csv-out is required")
    try:
        args.pointcloud_capture_topics = resolve_pointcloud_topics(
        args.pointcloud_mode, args.pointcloud_topics
        )
    except ValueError as error:
        parser.error(str(error))
    return args


def main(argv: Sequence[str] | None = None) -> int:
    args = parse_args(argv)
    report = run_capture(args)
    write_report(report, args.json_out, args.csv_out)
    print(
        json.dumps(
            {"rows": report["window"]["rows"], "topics": report["topics"]},
            sort_keys=True,
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
