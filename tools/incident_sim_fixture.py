#!/usr/bin/env python3
"""Drive the real recorder and SQLite replay with 17 minutes of synthetic ROS timestamps."""
import argparse
import json
from pathlib import Path
import time

from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import OccupancyGrid, Odometry, Path as NavPath
import rclpy
from rclpy.serialization import serialize_message
from rcl_interfaces.msg import Log
from salus_interfaces.msg import CmdVelFinal
from sensor_msgs.msg import LaserScan
from tf2_msgs.msg import TFMessage

from salus_incident.policy import NS
from salus_incident.recorder import Recorder
from salus_incident.replay import assemble, records


def run(root):
    rclpy.init(args=["--ros-args", "-p", f"root:={root}", "-p", "minimum_free_bytes:=1"])
    node = Recorder()
    stamp = 100 * NS
    node.now_ns = lambda: stamp

    def receive(topic, kind, message):
        node.receive(topic, kind, type(message), serialize_message(message))

    try:
        for tick in range(5101):
            stamp = 100 * NS + tick * (NS // 5)
            elapsed = tick / 5
            command = CmdVelFinal(source=CmdVelFinal.SOURCE_AUTO)
            command.twist.linear.x = 1.6
            command.twist.angular.z = -.028
            if elapsed == 900:
                stamp -= 31_870_494
                receive("/cmd_vel_final", "salus_interfaces/msg/CmdVelFinal", command)
                stamp += 31_870_494
                command.twist.angular.z = -.361
            receive("/cmd_vel_final", "salus_interfaces/msg/CmdVelFinal", command)
            seconds = stamp // NS
            if tick == 0:
                static_tf = TransformStamped(child_frame_id="laser")
                static_tf.header.frame_id = "base_footprint"
                static_tf.transform.rotation.w = 1.
                receive("/tf_static", "tf2_msgs/msg/TFMessage", TFMessage(transforms=[static_tf]))
            if tick == 4513:
                log = Log(level=40, name="controller_server",
                          msg="RegulatedPurePursuitController detected collision ahead!")
                receive("/rosout", "rcl_interfaces/msg/Log", log)
            pose = PoseStamped()
            pose.header.stamp.sec = seconds
            pose.header.frame_id = "map"
            pose.pose.orientation.w = 1.
            pose.pose.position.x = elapsed * .25
            plan = NavPath(header=pose.header, poses=[pose])
            odom = Odometry(header=pose.header, child_frame_id="base_footprint")
            odom.pose.pose = pose.pose
            tf = TransformStamped(header=pose.header, child_frame_id="base_footprint")
            tf.transform.rotation.w = 1.
            tf.transform.translation.x = pose.pose.position.x
            scan = LaserScan(angle_increment=.01, range_max=40., ranges=[12.] * 360)
            scan.header.stamp = pose.header.stamp
            scan.header.frame_id = "laser"
            costmap = OccupancyGrid(header=pose.header, data=[0] * 16)
            costmap.info.width = costmap.info.height = 4
            costmap.info.resolution = .1
            receive("/plan", "nav_msgs/msg/Path", plan)
            receive("/odometry/global", "nav_msgs/msg/Odometry", odom)
            receive("/tf", "tf2_msgs/msg/TFMessage", TFMessage(transforms=[tf]))
            receive("/scan", "sensor_msgs/msg/LaserScan", scan)
            receive("/local_costmap/costmap", "nav_msgs/msg/OccupancyGrid", costmap)
            receive("/global_costmap/costmap", "nav_msgs/msg/OccupancyGrid", costmap)
            if tick % 10 == 0:
                while node.queue.qsize() > 30:
                    if not node.worker.is_alive():
                        raise RuntimeError("Recorder stopped before the fixture ended")
                    time.sleep(.01)
        stamp += NS // 10
        while node.queue.qsize():
            time.sleep(.01)
        time.sleep(.4)
    finally:
        node.stopping.set()
        node.worker.join()
        node.destroy_node()
        rclpy.shutdown()

    paths = list((root / "incidents").glob("*/manifest.json"))
    assert len(paths) == 1, paths
    manifest = json.loads(paths[0].read_text())
    assert manifest["status"] == "available", manifest
    assert [item["reason"] for item in manifest["triggers"]] == [
        "STEERING_CURVATURE_JUMP", "PREDICTED_COLLISION",
    ]
    assert manifest["actual_end_ns"] - manifest["actual_start_ns"] >= 1020 * NS - NS // 5
    output = root / "replay"
    assemble(paths[0].parent, output)
    topics = {topic for _, topic, _, _ in records(output)}
    assert {"/plan", "/scan", "/tf", "/tf_static", "/odometry/global", "/incident_replay/cmd_vel_final"} <= topics
    assert "/cmd_vel_final" not in topics
    print(f"Validated 17-minute incident and replay: {output}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("root", type=Path)
    run(parser.parse_args().root)
