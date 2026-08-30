#!/usr/bin/env python3
"""Low-overhead timing/resource sidecar for the #119 scheduler-to-costmap chain."""

from __future__ import annotations

import argparse
import json
import os
import re
import signal
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import rclpy
from nav_msgs.msg import OccupancyGrid, Odometry
from rcl_interfaces.msg import Log
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from tf2_msgs.msg import TFMessage


SAMPLE_INTERVAL_S = 1.0
CLOCK_OBSERVE_INTERVAL_S = 0.1
EXPENSIVE_SAMPLE_INTERVAL_S = 5.0
REPORT_INTERVAL_S = 5.0

INTERESTING_LOG = re.compile(
    r"(Failed to meet update rate|extrapolation|Unable to transform robot pose|"
    r"Failed to make progress|tick rate|Message Filter dropping|controller loop|"
    r"snapshot generation exceeded target)",
    re.IGNORECASE,
)
EXTRAPOLATION = re.compile(
    r"Requested time\s+([0-9.]+).*?latest (?:data )?(?:is )?at time\s+([0-9.]+)",
    re.IGNORECASE,
)


def stamp_ns(stamp: Any) -> int:
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def normalize_frame(frame: str) -> str:
    return str(frame).lstrip("/")


@dataclass
class StreamStats:
    name: str
    count: int = 0
    first_wall_s: float | None = None
    last_wall_s: float | None = None
    first_stamp_ns: int | None = None
    last_stamp_ns: int | None = None
    max_wall_gap_s: float = 0.0
    max_stamp_gap_s: float = 0.0
    backwards_or_equal_stamps: int = 0
    window_count: int = 0
    window_max_wall_gap_s: float = 0.0
    window_max_stamp_gap_s: float = 0.0

    def record(self, stamp: int, now_s: float | None = None) -> None:
        now = time.monotonic() if now_s is None else float(now_s)
        if self.last_wall_s is not None:
            gap = max(0.0, now - self.last_wall_s)
            self.max_wall_gap_s = max(self.max_wall_gap_s, gap)
            self.window_max_wall_gap_s = max(self.window_max_wall_gap_s, gap)
        if self.last_stamp_ns is not None:
            delta_ns = stamp - self.last_stamp_ns
            if delta_ns <= 0:
                self.backwards_or_equal_stamps += 1
            else:
                gap = delta_ns / 1_000_000_000.0
                self.max_stamp_gap_s = max(self.max_stamp_gap_s, gap)
                self.window_max_stamp_gap_s = max(self.window_max_stamp_gap_s, gap)
        self.count += 1
        self.window_count += 1
        if self.first_wall_s is None:
            self.first_wall_s = now
        if self.first_stamp_ns is None:
            self.first_stamp_ns = stamp
        self.last_wall_s = now
        self.last_stamp_ns = stamp

    def summary(self, ros_now_ns: int) -> dict[str, Any]:
        wall_span = (
            None
            if self.first_wall_s is None or self.last_wall_s is None
            else max(0.0, self.last_wall_s - self.first_wall_s)
        )
        stamp_span = (
            None
            if self.first_stamp_ns is None or self.last_stamp_ns is None
            else max(0.0, (self.last_stamp_ns - self.first_stamp_ns) / 1_000_000_000.0)
        )
        return {
            "count": self.count,
            "effective_wall_hz": (
                None if not wall_span or self.count < 2 else (self.count - 1) / wall_span
            ),
            "effective_stamp_hz": (
                None if not stamp_span or self.count < 2 else (self.count - 1) / stamp_span
            ),
            "max_wall_gap_s": self.max_wall_gap_s,
            "max_stamp_gap_s": self.max_stamp_gap_s,
            "backwards_or_equal_stamps": self.backwards_or_equal_stamps,
            "latest_stamp_ns": self.last_stamp_ns,
            "age_vs_clock_s": (
                None
                if not ros_now_ns or self.last_stamp_ns is None
                else (ros_now_ns - self.last_stamp_ns) / 1_000_000_000.0
            ),
        }

    def take_window(self, elapsed_s: float, ros_now_ns: int) -> dict[str, Any]:
        data = {
            "received": self.window_count,
            "wall_hz": None if elapsed_s <= 0.0 else self.window_count / elapsed_s,
            "max_wall_gap_s": self.window_max_wall_gap_s,
            "max_stamp_gap_s": self.window_max_stamp_gap_s,
            "latest_stamp_ns": self.last_stamp_ns,
            "age_vs_clock_s": (
                None
                if not ros_now_ns or self.last_stamp_ns is None
                else (ros_now_ns - self.last_stamp_ns) / 1_000_000_000.0
            ),
        }
        self.window_count = 0
        self.window_max_wall_gap_s = 0.0
        self.window_max_stamp_gap_s = 0.0
        return data


class ResourceSampler:
    def __init__(self) -> None:
        self.clock_ticks = float(os.sysconf(os.sysconf_names["SC_CLK_TCK"]))
        self.previous_wall_s: float | None = None
        self.previous_cpu: tuple[int, int] | None = None
        self.previous_cgroup: dict[str, int] | None = None
        self.previous_process_wall_s: float | None = None
        self.previous_process_ticks: dict[int, int] = {}
        self.cached_top_processes: list[dict[str, Any]] = []
        self.cpu_limit = self._cpu_limit()
        self.visible_cpu_count = os.cpu_count()

    @staticmethod
    def _system_cpu() -> tuple[int, int] | None:
        try:
            fields = Path("/proc/stat").read_text(encoding="utf-8").splitlines()[0].split()[1:]
            values = [int(value) for value in fields]
            idle = values[3] + (values[4] if len(values) > 4 else 0)
            return sum(values), idle
        except (OSError, ValueError, IndexError):
            return None

    @staticmethod
    def _cgroup_cpu() -> dict[str, int]:
        result: dict[str, int] = {}
        try:
            for line in Path("/sys/fs/cgroup/cpu.stat").read_text(encoding="utf-8").splitlines():
                key, value = line.split()
                result[key] = int(value)
        except (OSError, ValueError):
            pass
        return result

    @staticmethod
    def _cpu_limit() -> str:
        try:
            return Path("/sys/fs/cgroup/cpu.max").read_text(encoding="utf-8").strip()
        except OSError:
            return ""

    def _processes(self) -> dict[int, tuple[int, str]]:
        result: dict[int, tuple[int, str]] = {}
        for entry in Path("/proc").iterdir():
            if not entry.name.isdigit():
                continue
            pid = int(entry.name)
            try:
                stat = (entry / "stat").read_text(encoding="utf-8")
                rest = stat[stat.rfind(") ") + 2 :].split()
                ticks = int(rest[11]) + int(rest[12])
                raw = (entry / "cmdline").read_bytes().replace(b"\0", b" ").decode(
                    "utf-8", errors="replace"
                ).strip()
                result[pid] = (ticks, raw[:180] or f"pid:{pid}")
            except (OSError, ValueError, IndexError):
                continue
        return result

    def sample(self, now_s: float, *, include_processes: bool = False) -> dict[str, Any]:
        elapsed = None if self.previous_wall_s is None else max(1.0e-6, now_s - self.previous_wall_s)
        system = self._system_cpu()
        cgroup = self._cgroup_cpu()
        system_pct = None
        if elapsed is not None and self.previous_cpu is not None and system is not None:
            total_delta = system[0] - self.previous_cpu[0]
            idle_delta = system[1] - self.previous_cpu[1]
            if total_delta > 0:
                system_pct = 100.0 * max(0, total_delta - idle_delta) / total_delta
        cgroup_pct = None
        throttled_delta_us = None
        if elapsed is not None and self.previous_cgroup is not None:
            usage = cgroup.get("usage_usec")
            old_usage = self.previous_cgroup.get("usage_usec")
            if usage is not None and old_usage is not None:
                cgroup_pct = max(0.0, usage - old_usage) / 1_000_000.0 / elapsed * 100.0
            throttled = cgroup.get("throttled_usec")
            old_throttled = self.previous_cgroup.get("throttled_usec")
            if throttled is not None and old_throttled is not None:
                throttled_delta_us = max(0, throttled - old_throttled)
        if include_processes:
            processes = self._processes()
            process_elapsed = (
                None
                if self.previous_process_wall_s is None
                else max(1.0e-6, now_s - self.previous_process_wall_s)
            )
            top = []
            if process_elapsed is not None:
                for pid, (ticks, command) in processes.items():
                    previous = self.previous_process_ticks.get(pid)
                    if previous is None:
                        continue
                    cpu_pct = (
                        max(0, ticks - previous)
                        / self.clock_ticks
                        / process_elapsed
                        * 100.0
                    )
                    if cpu_pct >= 0.5:
                        top.append({"pid": pid, "cpu_pct": cpu_pct, "command": command})
                top.sort(key=lambda item: item["cpu_pct"], reverse=True)
            self.cached_top_processes = top[:8]
            self.previous_process_wall_s = now_s
            self.previous_process_ticks = {
                pid: values[0] for pid, values in processes.items()
            }
        self.previous_wall_s = now_s
        self.previous_cpu = system
        self.previous_cgroup = cgroup
        try:
            load = list(os.getloadavg())
        except OSError:
            load = []
        return {
            "system_cpu_pct": system_pct,
            "container_cpu_pct": cgroup_pct,
            "cgroup_cpu_max": self.cpu_limit,
            "cgroup_nr_throttled": cgroup.get("nr_throttled"),
            "cgroup_throttled_delta_us": throttled_delta_us,
            "loadavg": load,
            "visible_cpu_count": self.visible_cpu_count,
            "top_processes_sampled": include_processes,
            "top_processes": self.cached_top_processes,
        }


class RuntimeTimingProbe(Node):
    def __init__(self, scenario: str, report_path: Path) -> None:
        super().__init__(
            "runtime_timing_probe",
            parameter_overrides=[Parameter("use_sim_time", value=True)],
        )
        self.scenario = scenario
        self.report_path = report_path
        self.started_wall_s = time.monotonic()
        self.last_sample_wall_s = self.started_wall_s
        self.last_expensive_sample_wall_s = (
            self.started_wall_s - EXPENSIVE_SAMPLE_INTERVAL_S
        )
        self.last_report_wall_s = self.started_wall_s
        self.publisher_counts: dict[str, int] = {}
        self.latest_clock_ns = 0
        self.last_clock_observe_wall_s = 0.0
        self.previous_sample_clock_ns = 0
        self.timeline: list[dict[str, Any]] = []
        self.events: list[dict[str, Any]] = []
        self.resources = ResourceSampler()
        self.streams = {
            name: StreamStats(name)
            for name in (
                "clock",
                "local_ekf",
                "global_ekf",
                "tf_map_to_odom",
                "tf_odom_to_base",
                "local_costmap",
            )
        }
        self.create_subscription(Odometry, "/odometry/local", self._on_local_odom, 20)
        self.create_subscription(Odometry, "/odometry/global", self._on_global_odom, 20)
        grid_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            OccupancyGrid, "/local_costmap/costmap", self._on_costmap, grid_qos
        )
        self.create_subscription(TFMessage, "/tf", self._on_tf, qos_profile_sensor_data)
        self.create_subscription(Log, "/rosout", self._on_log, 100)

    def observe_clock(self, now_s: float | None = None) -> None:
        """Sample ROS time without a Python callback on every /clock message."""
        now = time.monotonic() if now_s is None else float(now_s)
        if now - self.last_clock_observe_wall_s < CLOCK_OBSERVE_INTERVAL_S:
            return
        self.latest_clock_ns = self.get_clock().now().nanoseconds
        self.streams["clock"].record(self.latest_clock_ns, now)
        self.last_clock_observe_wall_s = now

    def _on_local_odom(self, message: Odometry) -> None:
        self.streams["local_ekf"].record(stamp_ns(message.header.stamp))

    def _on_global_odom(self, message: Odometry) -> None:
        self.streams["global_ekf"].record(stamp_ns(message.header.stamp))

    def _on_costmap(self, message: OccupancyGrid) -> None:
        self.streams["local_costmap"].record(stamp_ns(message.header.stamp))

    def _on_tf(self, message: TFMessage) -> None:
        now = time.monotonic()
        for transform in message.transforms:
            parent = normalize_frame(transform.header.frame_id)
            child = normalize_frame(transform.child_frame_id)
            if parent == "map" and child == "odom":
                self.streams["tf_map_to_odom"].record(stamp_ns(transform.header.stamp), now)
            elif parent == "odom" and child == "base_footprint":
                self.streams["tf_odom_to_base"].record(stamp_ns(transform.header.stamp), now)

    def _on_log(self, message: Log) -> None:
        if not INTERESTING_LOG.search(message.msg):
            return
        event: dict[str, Any] = {
            "wall_monotonic_s": time.monotonic(),
            "ros_clock_ns": self.latest_clock_ns,
            "log_stamp_ns": stamp_ns(message.stamp),
            "logger": message.name,
            "level": int(message.level),
            "message": message.msg[:600],
            "tf_latest_stamp_ns": {
                "map_to_odom": self.streams["tf_map_to_odom"].last_stamp_ns,
                "odom_to_base": self.streams["tf_odom_to_base"].last_stamp_ns,
            },
            "tf_age_vs_clock_s": {
                name: (
                    None
                    if not self.latest_clock_ns or stream.last_stamp_ns is None
                    else (self.latest_clock_ns - stream.last_stamp_ns)
                    / 1_000_000_000.0
                )
                for name, stream in (
                    ("map_to_odom", self.streams["tf_map_to_odom"]),
                    ("odom_to_base", self.streams["tf_odom_to_base"]),
                )
            },
        }
        match = EXTRAPOLATION.search(message.msg)
        if match:
            requested = float(match.group(1))
            latest = float(match.group(2))
            event["tf_requested_s"] = requested
            event["tf_latest_s"] = latest
            event["tf_future_delta_s"] = requested - latest
        self.events.append(event)
        if len(self.events) > 200:
            self.events = self.events[-200:]

    def sample(self) -> None:
        now = time.monotonic()
        elapsed = max(1.0e-6, now - self.last_sample_wall_s)
        ros_delta_s = (
            None
            if not self.latest_clock_ns or not self.previous_sample_clock_ns
            else (self.latest_clock_ns - self.previous_sample_clock_ns) / 1_000_000_000.0
        )
        expensive_sample = (
            not self.publisher_counts
            or now - self.last_expensive_sample_wall_s
            >= EXPENSIVE_SAMPLE_INTERVAL_S
        )
        if expensive_sample:
            self.publisher_counts = {
                topic: self.count_publishers(topic)
                for topic in (
                    "/clock",
                    "/odometry/local",
                    "/odometry/global",
                    "/tf",
                    "/local_costmap/costmap",
                )
            }
            self.last_expensive_sample_wall_s = now

        entry = {
            "wall_monotonic_s": now,
            "wall_since_start_s": now - self.started_wall_s,
            "ros_time_ns": self.latest_clock_ns,
            "ros_progress_s": ros_delta_s,
            "ros_to_wall_ratio": None if ros_delta_s is None else ros_delta_s / elapsed,
            "publishers": dict(self.publisher_counts),
            "streams": {
                name: stream.take_window(elapsed, self.latest_clock_ns)
                for name, stream in self.streams.items()
            },
            "resources": self.resources.sample(
                now, include_processes=expensive_sample
            ),
        }
        self.timeline.append(entry)
        if len(self.timeline) > 300:
            self.timeline = self.timeline[-300:]
        self.last_sample_wall_s = now
        self.previous_sample_clock_ns = self.latest_clock_ns
        if now - self.last_report_wall_s >= REPORT_INTERVAL_S:
            self._write_report()
            self.last_report_wall_s = now

    def flush(self) -> None:
        self._write_report()
        self.last_report_wall_s = time.monotonic()

    def _write_report(self) -> None:
        payload = {
            "schema_version": 1,
            "scenario": self.scenario,
            "started_monotonic_s": self.started_wall_s,
            "generated_monotonic_s": time.monotonic(),
            "ros_time_ns": self.latest_clock_ns,
            "streams": {
                name: stream.summary(self.latest_clock_ns)
                for name, stream in self.streams.items()
            },
            "events": self.events,
            "timeline": self.timeline,
        }
        self.report_path.parent.mkdir(parents=True, exist_ok=True)
        temporary = self.report_path.with_suffix(self.report_path.suffix + ".tmp")
        temporary.write_text(
            json.dumps(payload, sort_keys=True, separators=(",", ":")) + "\n",
            encoding="utf-8",
        )
        temporary.replace(self.report_path)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--scenario", required=True)
    parser.add_argument("--report-path", required=True, type=Path)
    args = parser.parse_args()

    stop = False

    def request_stop(_signum, _frame) -> None:
        nonlocal stop
        stop = True

    rclpy.init()
    # rclpy installs its own handlers during init; replace them afterwards so
    # the sidecar gets one final JSON flush during smoke process-group teardown.
    signal.signal(signal.SIGTERM, request_stop)
    signal.signal(signal.SIGINT, request_stop)
    node = RuntimeTimingProbe(args.scenario, args.report_path)
    next_sample = time.monotonic() + SAMPLE_INTERVAL_S
    try:
        while rclpy.ok() and not stop:
            rclpy.spin_once(node, timeout_sec=0.1)
            now = time.monotonic()
            node.observe_clock(now)
            if now >= next_sample:
                node.sample()
                next_sample = now + SAMPLE_INTERVAL_S
        node.flush()
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
