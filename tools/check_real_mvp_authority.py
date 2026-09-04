#!/usr/bin/env python3
"""Fail-closed, read-only preflight for the physical real MVP.

The checker runs on the host before ``real_mvp.launch.py``.  It deliberately
only observes systemd state, process arguments, device holders and ROS graph
publishers.  It never stops, kills, or otherwise changes an owner.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass, field
from pathlib import Path
import re
import shlex
import shutil
import subprocess
import sys
from typing import Mapping, Sequence


LEGACY_SERVICE = "salus-real-global-v2-wifi.service"
DEFAULT_DEVICES = ("/dev/ttyACM0", "/dev/ttyUSB0")
RUNTIME_EXEC = Path(__file__).resolve().with_name("real_runtime_exec.sh")
CRITICAL_TOPICS = (
    "/mavros_node/send_rtcm",
    "/mavros_node/mavros_node/send_rtcm",
    "/scan_3d_raw",
    "/scan_3d",
    "/cmd_vel_final",
    "/odometry/local",
    "/tf",
    "/tf_static",
)
PROCESS_PATTERNS = (
    "mavros",
    "rslidar",
    "controller_serv",
    "salus_controller",
    "ros2_salus",
)


@dataclass(frozen=True)
class AuthoritySnapshot:
    """Observable state used by the pure authority decision."""

    legacy_service_active: bool = False
    process_matches: tuple[str, ...] = ()
    topic_publishers: Mapping[str, tuple[str, ...]] = field(default_factory=dict)
    device_owners: Mapping[str, tuple[str, ...]] = field(default_factory=dict)


class ProbeError(RuntimeError):
    """A required host probe could not be completed."""


def evaluate_authority(
    snapshot: AuthoritySnapshot,
    *,
    required_devices: Sequence[str] = (),
) -> tuple[str, ...]:
    """Return fail-closed diagnostics; an empty tuple means a clean preflight."""

    failures: list[str] = []
    if snapshot.legacy_service_active:
        failures.append(f"legacy service active: {LEGACY_SERVICE}")

    for process in snapshot.process_matches:
        failures.append(f"unexpected relevant process: {process}")

    for topic in CRITICAL_TOPICS:
        publishers = tuple(snapshot.topic_publishers.get(topic, ()))
        if not publishers:
            continue
        if len(publishers) > 1:
            failures.append(
                f"duplicate publishers on {topic}: {', '.join(publishers)}"
            )
        else:
            failures.append(f"unexpected publisher on {topic}: {publishers[0]}")

    for device in required_devices:
        owners = tuple(snapshot.device_owners.get(device, ()))
        if owners:
            failures.append(f"device already owned {device}: {', '.join(owners)}")

    return tuple(failures)


def _run(command: Sequence[str], *, timeout_s: float = 5.0) -> subprocess.CompletedProcess[str]:
    try:
        return subprocess.run(
            list(command),
            check=False,
            capture_output=True,
            text=True,
            timeout=timeout_s,
        )
    except (OSError, subprocess.TimeoutExpired) as exc:
        raise ProbeError(f"probe failed: {' '.join(command)}: {exc}") from exc


def _service_is_active(service: str) -> bool:
    result = _run(("systemctl", "is-active", "--quiet", service))
    if result.returncode == 0:
        return True
    if result.returncode == 3:
        return False
    detail = (result.stderr or result.stdout).strip()
    raise ProbeError(f"cannot determine service state for {service}: {detail}")


def _process_matches(patterns: Sequence[str]) -> tuple[str, ...]:
    result = _run(("ps", "-eo", "pid=,args="))
    if result.returncode != 0:
        raise ProbeError(f"ps failed: {(result.stderr or result.stdout).strip()}")
    lowered_patterns = tuple(pattern.lower() for pattern in patterns)
    matches = []
    for line in result.stdout.splitlines():
        text = line.strip()
        lowered = text.lower()
        if text and any(pattern in lowered for pattern in lowered_patterns):
            matches.append(text)
    return tuple(matches)


def parse_topic_publishers(output: str) -> tuple[str, ...]:
    """Extract publisher node names from ``ros2 topic info -v`` output."""

    publisher_count = 0
    in_publishers = False
    names: list[str] = []
    for raw_line in output.splitlines():
        line = raw_line.strip()
        if line.startswith("Publisher count:"):
            try:
                publisher_count = int(line.split(":", 1)[1].strip())
            except ValueError as exc:
                raise ProbeError(f"unparseable publisher count: {line}") from exc
            in_publishers = True
            continue
        if line.startswith("Subscription count:"):
            break
        if in_publishers and line.startswith("Node name:"):
            names.append(line.split(":", 1)[1].strip())
    if publisher_count == 0:
        return ()
    if not names:
        return ("<unknown>",) * publisher_count
    return tuple(names)


def _topic_publishers(topic: str) -> tuple[str, ...]:
    result = _run(
        (
            str(RUNTIME_EXEC),
            "--",
            "bash",
            "-lc",
            f"exec ros2 topic info --no-daemon {shlex.quote(topic)} -v",
        )
    )
    output = f"{result.stdout}\n{result.stderr}"
    if re.search(r"\bunknown topic\b|topic .* does not exist", output, re.IGNORECASE):
        return ()
    if result.returncode != 0:
        detail = output.strip()
        raise ProbeError(f"cannot inspect {topic}: {detail}")
    return parse_topic_publishers(result.stdout)


def _device_owners(device: str) -> tuple[str, ...]:
    if not Path(device).exists():
        raise ProbeError(f"required device does not exist: {device}")
    if not shutil.which("fuser"):
        raise ProbeError("required probe command is unavailable: fuser")
    result = _run(("fuser", "-v", "-a", device))
    if result.returncode not in (0, 1):
        detail = (result.stderr or result.stdout).strip()
        raise ProbeError(f"cannot inspect {device}: {detail}")
    pids = sorted({int(value) for value in re.findall(r"\b[1-9][0-9]*\b", result.stdout)})
    owners: list[str] = []
    for pid in pids:
        process = _run(("ps", "-p", str(pid), "-o", "args="))
        if process.returncode == 0 and process.stdout.strip():
            owners.append(process.stdout.strip())
    return tuple(owners)


def collect_snapshot(
    *,
    devices: Sequence[str] = DEFAULT_DEVICES,
    topics: Sequence[str] = CRITICAL_TOPICS,
    process_patterns: Sequence[str] = PROCESS_PATTERNS,
    legacy_service: str = LEGACY_SERVICE,
) -> AuthoritySnapshot:
    """Collect host observations without changing any external state."""

    return AuthoritySnapshot(
        legacy_service_active=_service_is_active(legacy_service),
        process_matches=_process_matches(process_patterns),
        topic_publishers={topic: _topic_publishers(topic) for topic in topics},
        device_owners={device: _device_owners(device) for device in devices},
    )


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--device",
        action="append",
        dest="devices",
        help="device to verify has no owner (repeatable; defaults to Pixhawk and UART)",
    )
    parser.add_argument("--service", default=LEGACY_SERVICE)
    args = parser.parse_args(argv)
    devices = tuple(args.devices or DEFAULT_DEVICES)

    try:
        snapshot = collect_snapshot(devices=devices, legacy_service=args.service)
        failures = evaluate_authority(snapshot, required_devices=devices)
    except ProbeError as exc:
        print(f"REAL_MVP_AUTHORITY_FAIL: {exc}", file=sys.stderr)
        return 2

    if failures:
        print("REAL_MVP_AUTHORITY_FAIL")
        for failure in failures:
            print(f"- {failure}")
        return 1

    print("REAL_MVP_AUTHORITY_PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
