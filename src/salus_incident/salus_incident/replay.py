"""Assemble an incident into a ROS bag for local-only replay."""
import argparse
import heapq
import json
import os
from pathlib import Path
import subprocess

import rosbag2_py
from ament_index_python.packages import get_package_share_directory

from .recorder import metadata


def records(segment):
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(segment), storage_id="sqlite3"),
                rosbag2_py.ConverterOptions("", ""))
    types = {topic.name: topic.type for topic in reader.get_all_topics_and_types()}
    while reader.has_next():
        topic, raw, stamp = reader.read_next()
        yield stamp, topic, types[topic], raw


def assemble(incident, output):
    manifest = json.loads((incident / "manifest.json").read_text())
    if manifest["status"] not in {"available", "partial"}:
        raise ValueError("Incident must be finalized")
    streams = [records(path.parent) for path in sorted(incident.glob("*/segment.json"))]
    writer = rosbag2_py.SequentialWriter()
    writer.open(rosbag2_py.StorageOptions(uri=str(output), storage_id="sqlite3"),
                rosbag2_py.ConverterOptions("", ""))
    registered = set()
    for stamp, original, kind, raw in heapq.merge(*streams, key=lambda row: row[0]):
        if stamp > manifest["end_ns"]:
            break
        topic = "/incident_replay" + original if original.startswith("/cmd_vel") else original
        if topic not in registered:
            writer.create_topic(metadata(topic, kind))
            registered.add(topic)
        writer.write(topic, raw, stamp)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("incident", type=Path, help="Extracted incident directory")
    parser.add_argument("output", type=Path, help="New output bag directory")
    parser.add_argument("--play", action="store_true", help="Replay locally in ROS domain 197")
    parser.add_argument("--rviz", action="store_true", help="Open RViz and replay locally")
    args = parser.parse_args()
    assemble(args.incident, args.output)
    if args.play or args.rviz:
        env = dict(os.environ, ROS_DOMAIN_ID="197", ROS_LOCALHOST_ONLY="1")
        viewer = None
        try:
            if args.rviz:
                config = Path(get_package_share_directory("salus_incident")) / "config" / "incident.rviz"
                viewer = subprocess.Popen(["rviz2", "-d", str(config), "--ros-args",
                                           "-p", "use_sim_time:=true"], env=env)
            subprocess.run(["ros2", "bag", "play", str(args.output), "--clock", "--delay", "2"],
                           env=env, check=True)
        finally:
            if viewer:
                viewer.terminate()
                viewer.wait()
