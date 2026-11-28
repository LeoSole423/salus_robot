#!/usr/bin/env python3
"""Measure the same simulated mission with and without the incident recorder."""
import argparse
import json
import math
import os
from pathlib import Path
import subprocess
import time

from rcl_interfaces.msg import Log
import rclpy
from rclpy.qos import qos_profile_sensor_data
from salus_interfaces.msg import CmdVelFinal


def percentile(values, fraction):
    if not values:
        return None
    return sorted(values)[math.ceil(len(values) * fraction) - 1]


def recorder_pid():
    for process in Path("/proc").glob("[0-9]*"):
        try:
            args = (process / "cmdline").read_bytes().split(b"\0")
        except OSError:
            continue
        if len(args) >= 2 and args[1].endswith(b"/incident_recorder"):
            return int(process.name)
    return None


def read_process(pid):
    process = Path("/proc") / str(pid)
    stat = (process / "stat").read_text().rsplit(")", 1)[1].split()
    return time.monotonic(), (int(stat[11]) + int(stat[12])) / os.sysconf("SC_CLK_TCK"), (
        int(stat[21]) * os.sysconf("SC_PAGE_SIZE")
    )


def compare(before, after):
    baseline = json.loads(before.read_text())
    active = json.loads(after.read_text())
    limits = {
        "same_mission_succeeded": baseline["mission_exit"] == active["mission_exit"] == 0,
        "recorder_cpu": active["recorder_cpu_p95_cores"] is not None
        and active["recorder_cpu_p95_cores"] < 1,
        "recorder_memory": active["recorder_peak_rss_bytes"] < 256 << 20,
        "command_rate": baseline["auto_interval_p99_s"] is not None
        and active["auto_interval_p99_s"] is not None
        and active["auto_interval_p99_s"] <= 1.1 * baseline["auto_interval_p99_s"],
        "controller_deadlines": active["controller_deadline_warnings"] <= baseline["controller_deadline_warnings"],
    }
    print(json.dumps({"baseline": baseline, "recording": active, "limits": limits}, indent=2))
    return 0 if all(limits.values()) else 1


def observe(output, command):
    rclpy.init(args=[])
    node = rclpy.create_node("incident_load_probe")
    intervals, cpu, rss = [], [], []
    previous = sample = None
    warnings = 0
    pid = recorder_pid()

    def final(message):
        nonlocal previous
        if message.source != CmdVelFinal.SOURCE_AUTO:
            previous = None
            return
        now = time.monotonic()
        if previous is not None:
            intervals.append(now - previous)
        previous = now

    def log(message):
        nonlocal warnings
        if "control loop missed its desired rate" in message.msg.lower():
            warnings += 1

    node.create_subscription(CmdVelFinal, "/cmd_vel_final", final, qos_profile_sensor_data)
    node.create_subscription(Log, "/rosout", log, qos_profile_sensor_data)
    try:
        with subprocess.Popen(command) as mission:
            while mission.poll() is None:
                rclpy.spin_once(node, timeout_sec=.1)
                if pid is None:
                    pid = recorder_pid()
                if pid and (sample is None or time.monotonic() - sample[0] >= 1):
                    measured = read_process(pid)
                    if sample:
                        cpu.append((measured[1] - sample[1]) / (measured[0] - sample[0]))
                    rss.append(measured[2])
                    sample = measured
        result = {
            "mission_exit": mission.returncode,
            "auto_interval_samples": len(intervals),
            "auto_interval_p99_s": percentile(intervals, .99),
            "recorder_cpu_p95_cores": percentile(cpu, .95),
            "recorder_peak_rss_bytes": max(rss, default=0),
            "controller_deadline_warnings": warnings,
        }
        output.write_text(json.dumps(result, indent=2))
        return mission.returncode
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--compare", nargs=2, type=Path)
    parser.add_argument("--output", type=Path)
    parser.add_argument("command", nargs=argparse.REMAINDER)
    args = parser.parse_args()
    if args.compare:
        raise SystemExit(compare(*args.compare))
    command = args.command[1:] if args.command[:1] == ["--"] else args.command
    if not args.output or not command:
        parser.error("--output and a mission command are required")
    raise SystemExit(observe(args.output, command))
