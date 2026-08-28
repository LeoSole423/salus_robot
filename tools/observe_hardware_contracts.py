#!/usr/bin/env python3
"""Produce a bounded, read-only inventory of ROS 2 hardware topic contracts.

The tool invokes only ``ros2 topic list -t``, ``ros2 topic info -v`` and, when
explicitly requested, ``ros2 topic echo --once``.  It never publishes, calls a
service, changes a parameter, starts a launch, or records a bag.  Sample data
is deliberately reduced before it reaches the JSON report: only header,
status, and numeric covariance fields are retained.
"""

from __future__ import annotations

import argparse
import json
import subprocess
import sys
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Callable


@dataclass(frozen=True)
class CommandResult:
    returncode: int | None
    stdout: str = ""
    stderr: str = ""
    timed_out: bool = False


Runner = Callable[[list[str], float], CommandResult]
SAFE_SAMPLE_TYPES = {
    "sensor_msgs/msg/Imu",
    "sensor_msgs/msg/NavSatFix",
    "nav_msgs/msg/Odometry",
}


def run_command(command: list[str], timeout_s: float) -> CommandResult:
    """Run a read-only ROS CLI command with a hard per-command deadline."""
    try:
        completed = subprocess.run(
            command,
            text=True,
            capture_output=True,
            timeout=timeout_s,
            check=False,
        )
    except subprocess.TimeoutExpired as exc:
        return CommandResult(
            returncode=None,
            stdout=(exc.stdout or "") if isinstance(exc.stdout, str) else "",
            stderr=(exc.stderr or "") if isinstance(exc.stderr, str) else "",
            timed_out=True,
        )
    except OSError as exc:
        return CommandResult(returncode=None, stderr=str(exc))
    return CommandResult(completed.returncode, completed.stdout, completed.stderr)


def command_error(result: CommandResult) -> dict[str, object]:
    """Return bounded diagnostic metadata without copying CLI output to reports."""
    return {
        "returncode": result.returncode,
        "timed_out": result.timed_out,
        "stderr_preview": (
            result.stderr.strip().splitlines()[0][:240] if result.stderr.strip() else ""
        ),
    }


def parse_topic_list(output: str) -> dict[str, str | None]:
    """Parse ``ros2 topic list -t`` while retaining topics with an unknown type."""
    topics: dict[str, str | None] = {}
    for line in output.splitlines():
        line = line.strip()
        if not line or not line.startswith("/"):
            continue
        if " [" in line and line.endswith("]"):
            topic, type_name = line.rsplit(" [", 1)
            topics[topic] = type_name[:-1] or None
        else:
            topics[line] = None
    return topics


def _scalar(value: str) -> str | int | float | bool | None:
    value = value.strip().strip("'\"")
    if value in {"", "null", "None", "~"}:
        return None
    if value.lower() in {"true", "false"}:
        return value.lower() == "true"
    try:
        return int(value)
    except ValueError:
        try:
            return float(value)
        except ValueError:
            return value


def parse_topic_info(output: str) -> dict[str, object]:
    """Extract counts and endpoint QoS from the human-readable ROS CLI output."""
    result: dict[str, object] = {
        "publisher_count": None,
        "subscriber_count": None,
        "publishers": [],
        "subscribers": [],
    }
    current_group: list[dict[str, object]] | None = None
    endpoint: dict[str, object] | None = None
    in_qos = False
    for raw_line in output.splitlines():
        stripped = raw_line.strip()
        if stripped.startswith("Publisher count:"):
            result["publisher_count"] = _scalar(stripped.partition(":")[2])
            current_group = result["publishers"]  # type: ignore[assignment]
            endpoint = None
            in_qos = False
            continue
        if stripped.startswith("Subscription count:") or stripped.startswith("Subscriber count:"):
            result["subscriber_count"] = _scalar(stripped.partition(":")[2])
            current_group = result["subscribers"]  # type: ignore[assignment]
            endpoint = None
            in_qos = False
            continue
        if stripped.startswith("Node name:") and current_group is not None:
            endpoint = {"node_name": stripped.partition(":")[2].strip(), "qos": {}}
            current_group.append(endpoint)
            in_qos = False
            continue
        if stripped == "QoS profile:" and endpoint is not None:
            in_qos = True
            continue
        if in_qos and endpoint is not None and ":" in stripped:
            key, _, value = stripped.partition(":")
            qos = endpoint["qos"]
            assert isinstance(qos, dict)
            qos[key.strip().lower().replace(" ", "_")] = _scalar(value)
    return result


def sanitize_sample(output: str) -> dict[str, object]:
    """Keep only a minimal, non-positional subset of a YAML-like ``ros2 echo``.

    No generic message fields are copied.  This prevents position, RTCM, image,
    diagnostic payloads, and future unknown fields from entering the report.
    """
    headers: dict[str, object] = {}
    statuses: dict[str, object] = {}
    covariances: dict[str, list[float]] = {}
    stack: list[tuple[int, str]] = []
    covariance_path: str | None = None
    covariance_indent = -1

    for raw_line in output.splitlines():
        if not raw_line.strip() or raw_line.lstrip().startswith("---"):
            continue
        indent = len(raw_line) - len(raw_line.lstrip())
        text = raw_line.strip()
        while stack and indent <= stack[-1][0]:
            stack.pop()
        if text.startswith("- ") and covariance_path:
            value = _scalar(text[2:])
            if isinstance(value, (int, float)) and not isinstance(value, bool):
                covariances[covariance_path].append(float(value))
            continue
        if covariance_path and indent <= covariance_indent:
            covariance_path = None
        if ":" not in text:
            continue
        key, _, raw_value = text.partition(":")
        key = key.strip()
        value = raw_value.strip()
        path = ".".join([part for _, part in stack] + [key])
        lowered_path = path.lower()
        if not value:
            stack.append((indent, key))
            if "covariance" in key.lower():
                covariance_path = path
                covariance_indent = indent
                covariances[path] = []
            continue
        if lowered_path in {"header.frame_id", "header.stamp.sec", "header.stamp.nanosec"}:
            headers[lowered_path] = _scalar(value)
        elif any(part.lower() == "status" for _, part in stack) or key.lower() == "status":
            statuses[path] = _scalar(value)
        elif "covariance" in key.lower():
            numbers: list[float] = []
            for item in value.strip("[]").split(","):
                scalar = _scalar(item)
                if isinstance(scalar, (int, float)) and not isinstance(scalar, bool):
                    numbers.append(float(scalar))
            covariances[path] = numbers
    return {"headers": headers, "statuses": statuses, "covariances": covariances}


def observe(
    topics: list[str] | None,
    *,
    include_samples: bool,
    timeout_s: float,
    runner: Runner = run_command,
) -> dict[str, object]:
    """Collect contract metadata.  All invoked commands are read-only topic CLI calls."""
    listed = runner(["ros2", "topic", "list", "-t"], timeout_s)
    discovered = parse_topic_list(listed.stdout) if listed.returncode == 0 else {}
    selected = list(dict.fromkeys(topics or list(discovered)))
    report: dict[str, object] = {
        "schema_version": 1,
        "observed_at_utc": datetime.now(timezone.utc).isoformat(),
        "read_only": True,
        "commands": [
            "ros2 topic list -t",
            "ros2 topic info -v",
            "ros2 topic echo --once (optional, sanitized)",
        ],
        "topic_list": command_error(listed),
        "topics": [],
    }
    for topic in selected:
        metadata = runner(["ros2", "topic", "info", "-v", topic], timeout_s)
        entry: dict[str, object] = {
            "name": topic,
            "type": discovered.get(topic),
            "info": parse_topic_info(metadata.stdout) if metadata.returncode == 0 else None,
            "info_command": command_error(metadata),
        }
        if include_samples and entry["type"] in SAFE_SAMPLE_TYPES:
            sample = runner(["ros2", "topic", "echo", "--once", topic], timeout_s)
            entry["sample"] = sanitize_sample(sample.stdout) if sample.returncode == 0 else None
            entry["sample_command"] = command_error(sample)
        elif include_samples:
            entry["sample"] = None
            entry["sample_skipped"] = "topic type is not on the safe metadata whitelist"
        report["topics"].append(entry)  # type: ignore[union-attr]
    return report


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True, help="Destination JSON report path.")
    parser.add_argument(
        "--topic",
        action="append",
        help="Topic to inspect; repeatable. Defaults to all discovered topics.",
    )
    parser.add_argument(
        "--sample-metadata",
        action="store_true",
        help=(
            "Subscribe once for sanitized metadata from explicitly selected, "
            "whitelisted message types."
        ),
    )
    parser.add_argument(
        "--timeout", type=float, default=2.0, help="Per-command timeout in seconds (default: 2)."
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.timeout <= 0:
        raise ValueError("--timeout must be positive")
    if args.sample_metadata and not args.topic:
        raise ValueError("--sample-metadata requires at least one explicit --topic")
    report = observe(
        args.topic, include_samples=args.sample_metadata, timeout_s=args.timeout
    )
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return 0


if __name__ == "__main__":
    sys.exit(main())
