#!/usr/bin/env python3
"""Read-only evidence collector for the physical localization shadow trial.

This tool **only subscribes**. It never creates a publisher, never broadcasts a
transform, never calls an action or service and never writes to the DDS graph
beyond the node itself. It exists to characterise how the non-authoritative
Salus local EKF compares with the legacy ``/odometry/local`` while
``ROS2_SALUS`` keeps every authority.

Absolute pose equality is intentionally *not* asserted as a contract. This
first pass only characterises rates, frames, timestamp behaviour, finiteness
and current differences, as agreed in #161.

TF is measured by **payload, not endpoint**. ``robot_localization`` always
constructs a ``TransformBroadcaster``, so the shadow node advertises a ``/tf``
publisher even with ``publish_tf=false``; what must stay empty is the stream of
transforms. This tool therefore counts received ``/tf`` messages, the distinct
``parent -> child`` pairs and the approximate rate, and snapshots them before,
during and after the run: an increase in pairs or rate while the shadow runs
would mean TF authority leaked.

The only publishers this process holds are the implicit ``/rosout`` and
``/parameter_events`` endpoints that rclpy creates for any node; it creates no
domain publisher, broadcasts no transform and calls no service or action.

Example::

    python3 tools/observe_localization_shadow.py --duration 60 \
        --json-out /tmp/localization_shadow.json
"""

from __future__ import annotations

import argparse
import json
import math
import time
from collections import Counter, deque
from dataclasses import dataclass, field
from datetime import datetime, timezone

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_msgs.msg import TFMessage


MAX_TRACKED_SAMPLES = 4000


@dataclass
class TransformStats:
    """Payload accounting for a TF topic."""

    message_count: int = 0
    transform_count: int = 0
    pairs: Counter = field(default_factory=Counter)
    first_arrival_s: float | None = None
    last_arrival_s: float | None = None

    def add(self, message: TFMessage, arrival_s: float) -> None:
        if self.first_arrival_s is None:
            self.first_arrival_s = arrival_s
        self.last_arrival_s = arrival_s
        self.message_count += 1
        for transform in message.transforms:
            self.transform_count += 1
            parent = transform.header.frame_id or "<empty>"
            child = transform.child_frame_id or "<empty>"
            self.pairs[f"{parent} -> {child}"] += 1

    def summarise(self, now_s: float) -> dict[str, object]:
        window = (
            self.last_arrival_s - self.first_arrival_s
            if self.first_arrival_s is not None and self.last_arrival_s is not None
            else 0.0
        )
        rate = (
            round(self.message_count / window, 2)
            if window > 0.0 and self.message_count
            else None
        )
        return {
            "messages": self.message_count,
            "transforms": self.transform_count,
            "approx_message_rate_hz": rate,
            "pairs": dict(sorted(self.pairs.items())),
            "age_of_last_message_s": (
                round(now_s - self.last_arrival_s, 3) if self.last_arrival_s else None
            ),
        }



@dataclass
class OdometryStats:
    """Aggregated observation of one odometry topic."""

    topic: str
    count: int = 0
    nonfinite_count: int = 0
    monotonic_violations: int = 0
    frame_ids: set[str] = field(default_factory=set)
    child_frame_ids: set[str] = field(default_factory=set)
    first_arrival_s: float | None = None
    last_arrival_s: float | None = None
    last_stamp_s: float | None = None
    samples: deque = field(default_factory=lambda: deque(maxlen=MAX_TRACKED_SAMPLES))

    def add(self, message: Odometry, arrival_s: float) -> None:
        stamp_s = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
        if self.first_arrival_s is None:
            self.first_arrival_s = arrival_s
        if self.last_stamp_s is not None and stamp_s < self.last_stamp_s:
            self.monotonic_violations += 1
        self.last_stamp_s = stamp_s
        self.last_arrival_s = arrival_s
        self.count += 1
        self.frame_ids.add(message.header.frame_id)
        self.child_frame_ids.add(message.child_frame_id)

        pose = message.pose.pose
        twist = message.twist.twist
        values = (
            pose.position.x, pose.position.y, pose.position.z,
            pose.orientation.x, pose.orientation.y,
            pose.orientation.z, pose.orientation.w,
            twist.linear.x, twist.linear.y, twist.angular.z,
        )
        if not all(math.isfinite(value) for value in values):
            self.nonfinite_count += 1
            return

        self.samples.append({
            "stamp_s": stamp_s,
            "x": pose.position.x,
            "y": pose.position.y,
            "yaw": _yaw_from_quaternion(pose.orientation),
            "linear_x": twist.linear.x,
            "yaw_rate": twist.angular.z,
        })

    def summarise(self, now_s: float) -> dict[str, object]:
        window = (
            self.last_arrival_s - self.first_arrival_s
            if self.first_arrival_s is not None and self.last_arrival_s is not None
            else 0.0
        )
        rate = (self.count - 1) / window if window > 0.0 and self.count > 1 else None
        summary: dict[str, object] = {
            "messages": self.count,
            "approx_rate_hz": round(rate, 2) if rate is not None else None,
            "frame_ids": sorted(self.frame_ids),
            "child_frame_ids": sorted(self.child_frame_ids),
            "header_stamps_monotonic": self.monotonic_violations == 0,
            "monotonic_violations": self.monotonic_violations,
            "nonfinite_messages": self.nonfinite_count,
            "age_of_last_sample_s": (
                round(now_s - self.last_arrival_s, 3) if self.last_arrival_s else None
            ),
            "last_sample_stamp_s": self.last_stamp_s,
        }
        if self.samples:
            latest = self.samples[-1]
            first = self.samples[0]
            summary["latest"] = {
                "position_m": [round(latest["x"], 4), round(latest["y"], 4)],
                "yaw_rad": round(latest["yaw"], 4),
                "linear_velocity_mps": round(latest["linear_x"], 4),
                "yaw_rate_rps": round(latest["yaw_rate"], 4),
            }
            summary["window_drift_m"] = {
                "dx": round(latest["x"] - first["x"], 4),
                "dy": round(latest["y"] - first["y"], 4),
                "dyaw_rad": round(
                    _angle_difference(latest["yaw"] - first["yaw"]), 4
                ),
            }
        return summary


def _yaw_from_quaternion(orientation) -> float:
    return math.atan2(
        2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
        1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z),
    )


def _angle_difference(radians: float) -> float:
    return (radians + math.pi) % (2.0 * math.pi) - math.pi


def _nearest(samples: deque, stamp_s: float) -> dict | None:
    best, best_delta = None, None
    for sample in samples:
        delta = abs(sample["stamp_s"] - stamp_s)
        if best_delta is None or delta < best_delta:
            best, best_delta = sample, delta
    if best is None or best_delta is None:
        return None
    return {**best, "_matched_delta_s": best_delta}


def compare_topics(legacy: OdometryStats, shadow: OdometryStats) -> dict[str, object]:
    """Time-paired differences between the legacy estimate and the shadow."""
    if not legacy.samples or not shadow.samples:
        return {"pairs_compared": 0, "note": "one of the topics produced no usable samples"}

    pairs = []
    for sample in shadow.samples:
        match = _nearest(legacy.samples, sample["stamp_s"])
        if match is None:
            continue
        if match["_matched_delta_s"] > 0.25:
            continue
        pairs.append({
            "position_delta_m": [
                round(sample["x"] - match["x"], 4),
                round(sample["y"] - match["y"], 4),
            ],
            "yaw_delta_rad": round(_angle_difference(sample["yaw"] - match["yaw"]), 4),
            "linear_velocity_delta_mps": round(
                sample["linear_x"] - match["linear_x"], 4
            ),
            "yaw_rate_delta_rps": round(sample["yaw_rate"] - match["yaw_rate"], 4),
            "match_skew_s": round(match["_matched_delta_s"], 4),
        })
    if not pairs:
        return {"pairs_compared": 0, "note": "no temporally matching samples"}

    def extremes(key):
        values = [math.hypot(*p[key]) if key == "position_delta_m" else abs(p[key]) for p in pairs]
        return {"max": round(max(values), 4), "mean": round(sum(values) / len(values), 4)}

    return {
        "pairs_compared": len(pairs),
        "position_delta_m": extremes("position_delta_m"),
        "yaw_delta_rad": extremes("yaw_delta_rad"),
        "linear_velocity_delta_mps": extremes("linear_velocity_delta_mps"),
        "yaw_rate_delta_rps": extremes("yaw_rate_delta_rps"),
        "last_pair": pairs[-1],
    }


def _endpoint_names(infos) -> list[str]:
    names = []
    for info in infos:
        namespace = info.node_namespace.rstrip("/")
        names.append(f"{namespace}/{info.node_name}" if namespace else info.node_name)
    return names


def endpoints(node: Node, topic: str) -> dict[str, object]:
    types = [
        message_types
        for name, message_types in node.get_topic_names_and_types()
        if name == topic
    ]
    return {
        "topic": topic,
        "present_on_graph": bool(types),
        "types": types[0] if types else [],
        "publisher_count": node.count_publishers(topic),
        "publishers": sorted(set(_endpoint_names(node.get_publishers_info_by_topic(topic)))),
        "subscriber_count": node.count_subscribers(topic),
        "subscribers": sorted(
            set(_endpoint_names(node.get_subscriptions_info_by_topic(topic)))
        ),
    }


class Observer(Node):
    def __init__(self, topics: dict[str, str], tf_topic: str = "") -> None:
        super().__init__("localization_shadow_observer")
        self.stats: dict[str, OdometryStats] = {
            name: OdometryStats(topic=topic) for name, topic in topics.items()
        }
        self.transforms = TransformStats()
        self.qos = 50
        for name, topic in topics.items():
            if not topic:
                continue
            self.create_subscription(
                Odometry, topic, self._make_callback(name), self.qos
            )
        if tf_topic:
            self.create_subscription(
                TFMessage, tf_topic, self._on_transform, self.qos
            )

    def _make_callback(self, name: str):
        def callback(message: Odometry) -> None:
            self.stats[name].add(message, time.monotonic())
        return callback

    def _on_transform(self, message: TFMessage) -> None:
        self.transforms.add(message, time.monotonic())


def _tf_snapshot(
    node: Observer, topic: str, phase: str, started: float, previous: dict | None
) -> dict[str, object]:
    """Pair the graph view with the payload view of a TF topic."""
    now = time.monotonic()
    payload = node.transforms.summarise(now)
    delta = {
        "messages_since_previous": node.transforms.message_count
        - (previous or {}).get("_messages", 0),
        "transforms_since_previous": node.transforms.transform_count
        - (previous or {}).get("_transforms", 0),
    }
    return {
        "phase": phase,
        "at_s": round(now - started, 1),
        "endpoints": endpoints(node, topic),
        "payload": payload,
        "payload_delta": delta,
        "_messages": node.transforms.message_count,
        "_transforms": node.transforms.transform_count,
    }


def _strip_private(snapshots: list[dict[str, object]]) -> list[dict[str, object]]:
    return [
        {key: value for key, value in snapshot.items() if not key.startswith("_")}
        for snapshot in snapshots
    ]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--duration", type=float, default=30.0)
    parser.add_argument("--legacy-topic", default="/odometry/local")
    parser.add_argument("--shadow-topic", default="/salus/localization_shadow/odometry/local")
    parser.add_argument("--wheel-odometry-topic", default="/wheel/odometry")
    parser.add_argument("--tf-topic", default="/tf")
    parser.add_argument("--json-out", default="")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    rclpy.init()
    node = Observer(
        {
            "legacy_local_odometry": args.legacy_topic,
            "shadow_local_odometry": args.shadow_topic,
            "wheel_odometry": args.wheel_odometry_topic,
        },
        tf_topic=args.tf_topic,
    )
    started = time.monotonic()
    graph_wait = 5.0
    deadline = started + args.duration
    snapshots = [_tf_snapshot(node, args.tf_topic, "before", started, None)]
    midpoint_reported = False
    try:
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.2)
            if not midpoint_reported and time.monotonic() - started >= graph_wait + (
                deadline - started - graph_wait
            ) / 2.0:
                snapshots.append(_tf_snapshot(
                    node, args.tf_topic, "during", started, snapshots[-1]
                ))
                midpoint_reported = True
    except KeyboardInterrupt:
        pass
    finally:
        if not midpoint_reported:
            snapshots.append(_tf_snapshot(
                node, args.tf_topic, "during", started, snapshots[-1]
            ))
        snapshots.append(_tf_snapshot(
            node, args.tf_topic, "after", started, snapshots[-1]
        ))
        now = time.monotonic()
        report: dict[str, object] = {
            "tool": "observe_localization_shadow",
            "generated_at": datetime.now(timezone.utc).isoformat(timespec="seconds"),
            "window_s": round(now - started, 1),
            "publisher_allowed": False,
            "topics": {
                name: {**stats.summarise(now), "endpoints": endpoints(node, stats.topic)}
                for name, stats in node.stats.items()
            },
            "comparison": compare_topics(
                node.stats["legacy_local_odometry"], node.stats["shadow_local_odometry"]
            ),
            "tf": {
                "note": (
                    "advertised endpoints are not authority; read payload pairs "
                    "and payload_delta"
                ),
                "snapshots": _strip_private(snapshots),
                "total": node.transforms.summarise(now),
            },
        }
        node.destroy_node()
        rclpy.shutdown()

    text = json.dumps(report, indent=2, sort_keys=True)
    if args.json_out:
        with open(args.json_out, "w", encoding="utf-8") as handle:
            handle.write(text + "\n")
    print(text)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
