#!/usr/bin/env python3
"""Low-overhead timing/resource sidecar for runtime reliability smokes."""

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
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import LaserScan
from tf2_msgs.msg import TFMessage


INTERESTING_LOG = re.compile(
    r"(Failed to meet update rate|extrapolation|Unable to transform robot pose|"
    r"Failed to make progress|tick rate|Message Filter dropping|controller loop)",
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
        self.previous_process_ticks: dict[int, int] = {}

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

    def sample(self, now_s: float) -> dict[str, Any]:
        elapsed = None if self.previous_wall_s is None else max(1.0e-6, now_s - self.previous_wall_s)
        system = self._system_cpu()
        cgroup = self._cgroup_cpu()
        processes = self._processes()
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
        top = []
        if elapsed is not None:
            for pid, (ticks, command) in processes.items():
                previous = self.previous_process_ticks.get(pid)
                if previous is None:
                    continue
                cpu_pct = max(0, ticks - previous) / self.clock_ticks / elapsed * 100.0
                if cpu_pct >= 0.5:
                    top.append({"pid": pid, "cpu_pct": cpu_pct, "command": command})
            top.sort(key=lambda item: item["cpu_pct"], reverse=True)
        self.previous_wall_s = now_s
        self.previous_cpu = system
        self.previous_cgroup = cgroup
        self.previous_process_ticks = {pid: values[0] for pid, values in processes.items()}
        try:
            load = list(os.getloadavg())
        except OSError:
            load = []
        return {
            "system_cpu_pct": system_pct,
            "container_cpu_pct": cgroup_pct,
            "cgroup_cpu_max": self._cpu_limit(),
            "cgroup_nr_throttled": cgroup.get("nr_throttled"),
            "cgroup_throttled_delta_us": throttled_delta_us,
            "loadavg": load,
            "visible_cpu_count": os.cpu_count(),
            "top_processes": top[:8],
        }


class RuntimeTimingProbe(Node):
    def __init__(self, scenario: str, report_path: Path) -> None:
        super().__init__("runtime_timing_probe")
        self.scenario = scenario
        self.report_path = report_path
        self.started_wall_s = time.monotonic()
        self.last_sample_wall_s = self.started_wall_s
        self.latest_clock_ns = 0
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
                "scan_clean",
            )
        }
        self.create_subscription(Clock, "/clock", self._on_clock, qos_profile_sensor_data)
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
        self.create_subscription(LaserScan, "/scan_clean", self._on_scan, qos_profile_sensor_data)
        self.create_subscription(TFMessage, "/tf", self._on_tf, qos_profile_sensor_data)
        self.create_subscription(Log, "/rosout", self._on_log, 100)

    def _on_clock(self, message: Clock) -> None:
        self.latest_clock_ns = stamp_ns(message.clock)
        self.streams["clock"].record(self.latest_clock_ns)

    def _on_local_odom(self, message: Odometry) -> None:
        self.streams["local_ekf"].record(stamp_ns(message.header.stamp))

    def _on_global_odom(self, message: Odometry) -> None:
        self.streams["global_ekf"].record(stamp_ns(message.header.stamp))

    def _on_costmap(self, message: OccupancyGrid) -> None:
        self.streams["local_costmap"].record(stamp_ns(message.header.stamp))

    def _on_scan(self, message: LaserScan) -> None:
        self.streams["scan_clean"].record(stamp_ns(message.header.stamp))

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
        entry = {
            "wall_monotonic_s": now,
            "wall_since_start_s": now - self.started_wall_s,
            "ros_time_ns": self.latest_clock_ns,
            "ros_progress_s": ros_delta_s,
            "ros_to_wall_ratio": None if ros_delta_s is None else ros_delta_s / elapsed,
            "publishers": {
                "/clock": self.count_publishers("/clock"),
                "/odometry/local": self.count_publishers("/odometry/local"),
                "/odometry/global": self.count_publishers("/odometry/global"),
                "/tf": self.count_publishers("/tf"),
                "/local_costmap/costmap": self.count_publishers("/local_costmap/costmap"),
                "/scan_clean": self.count_publishers("/scan_clean"),
            },
            "streams": {
                name: stream.take_window(elapsed, self.latest_clock_ns)
                for name, stream in self.streams.items()
            },
            "resources": self.resources.sample(now),
        }
        self.timeline.append(entry)
        if len(self.timeline) > 300:
            self.timeline = self.timeline[-300:]
        self.last_sample_wall_s = now
        self.previous_sample_clock_ns = self.latest_clock_ns
        self._write_report()

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
        temporary.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")
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

    signal.signal(signal.SIGTERM, request_stop)
    signal.signal(signal.SIGINT, request_stop)
    rclpy.init()
    node = RuntimeTimingProbe(args.scenario, args.report_path)
    next_sample = time.monotonic() + 1.0
    try:
        while rclpy.ok() and not stop:
            rclpy.spin_once(node, timeout_sec=0.1)
            now = time.monotonic()
            if now >= next_sample:
                node.sample()
                next_sample = now + 1.0
        node.sample()
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
