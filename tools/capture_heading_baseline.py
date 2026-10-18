#!/usr/bin/env python3
"""Manual simulation measurement: straight, strong turn, straight, stop."""
import argparse
import json
import math
import os
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, NavSatFix
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage
from salus_interfaces.msg import DriveTelemetry, CmdVelFinal

SEQUENCE = [('straight_before', 8.0, 1.2, 0.0),
            ('strong_turn', 4.5, 1.2, -0.35),
            ('straight_after', 8.0, 1.2, 0.0), ('stop', 3.0, 0.0, 0.0)]

def stamp_s(stamp):
    return stamp.sec + stamp.nanosec * 1e-9

def yaw(q):
    return math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))

class Probe(Node):
    def __init__(self, directory, scenario="right"):
        self.scenario = scenario
        self.sequence = list(SEQUENCE)
        if scenario in ("left", "gentle", "oscillations"):
            rate = {"left": 0.35, "gentle": -0.10, "oscillations": 0.0}[scenario]
            self.sequence[1] = ("strong_turn", 4.5, 1.2, rate)
        recipe_path = os.environ.get('HEADING_RECIPE', '')
        if recipe_path:
            self.sequence = json.loads(Path(recipe_path).read_text())['sequence']
        super().__init__('heading_baseline_probe', parameter_overrides=[Parameter('use_sim_time', value=True)])
        self.directory = directory
        self.stream = (directory / 'samples.jsonl').open('w')
        self.phase = 'readiness'
        self.start = None
        self.counts = {}
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.subs = []
        for topic in ['/simulation/ground_truth', '/odom_raw', '/odometry/local', '/odometry/global', '/wheel/odometry', '/odometry/local_global', '/gps/odometry_map',
                      '/diagnostic/global_no_gps_position', '/diagnostic/global_compensated_gps',
                      '/diagnostic/gps_base_position', '/diagnostic/global_delayed_gps_reference',
                      '/diagnostic/gps_ant_position_delayed']:
            self.subs.append(self.create_subscription(Odometry, topic,
                lambda m, t=topic: self.record(t, stamp_s(m.header.stamp),
                    yaw_rad=yaw(m.pose.pose.orientation), x=m.pose.pose.position.x,
                    y=m.pose.pose.position.y, yaw_rate=m.twist.twist.angular.z,
                    speed=m.twist.twist.linear.x, lateral_speed=m.twist.twist.linear.y,
                    pose_covariance=list(m.pose.covariance),
                    twist_covariance=list(m.twist.covariance),
                    frame_id=m.header.frame_id, child_frame_id=m.child_frame_id), 10))
        for topic in ['/imu/data', '/gps/course_heading', '/localization/orientation']:
            self.subs.append(self.create_subscription(Imu, topic,
                lambda m, t=topic: self.record(t, stamp_s(m.header.stamp),
                    yaw_rad=yaw(m.orientation), yaw_rate=m.angular_velocity.z,
                    yaw_variance=m.orientation_covariance[8],
                    gyro_variance=m.angular_velocity_covariance[8]), qos_profile_sensor_data))
        for topic in ['/gps/course_heading/debug', '/gps/odometry_map/debug']:
            self.subs.append(self.create_subscription(String, topic,
                lambda m,t=topic: self.on_debug(t,m), 10))
        self.subs.append(self.create_subscription(NavSatFix, '/gps/fix',
            lambda m: self.record('/gps/fix', stamp_s(m.header.stamp),
                lat=m.latitude, lon=m.longitude), qos_profile_sensor_data))
        self.subs.append(self.create_subscription(DriveTelemetry, '/controller/drive_telemetry',
            lambda m: self.record('/controller/drive_telemetry', stamp_s(m.stamp),
                steer=m.steer_deg_measured, steer_valid=m.steer_valid,
                fresh=m.fresh, speed=m.speed_mps_measured), 10))
        self.subs.append(self.create_subscription(CmdVelFinal, '/cmd_vel_final',
            lambda m: self.record('/cmd_vel_final', self.now(), speed=m.twist.linear.x,
                yaw_rate=m.twist.angular.z, source=m.source), 10))
        self.subs.append(self.create_subscription(TFMessage, '/tf', self.on_tf, 100))
        self.timer = self.create_timer(0.04, self.tick)
        self.done = False

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def record(self, topic, stamp, **values):
        self.counts[topic] = self.counts.get(topic, 0) + 1
        self.stream.write(json.dumps(dict(topic=topic, stamp_s=stamp,
            receive_sim_s=self.now(), phase=self.phase, **values))+'\n')

    def on_debug(self, topic, message):
        payload = json.loads(message.data)
        original_stamp = payload.pop('stamp_s', None)
        if original_stamp is not None:
            payload['measurement_stamp_s'] = original_stamp
        self.record(topic, self.now(), **payload)

    def on_tf(self, msg):
        for t in msg.transforms:
            if (t.header.frame_id, t.child_frame_id) in [('map','odom'), ('odom','base_footprint')]:
                self.record('/tf/'+t.header.frame_id+'/'+t.child_frame_id,
                    stamp_s(t.header.stamp), yaw_rad=yaw(t.transform.rotation))

    def tick(self):
        if self.start is None:
            required = ['/odom_raw', '/odometry/local', '/odometry/global',
                        '/gps/fix', '/imu/data', '/controller/drive_telemetry']
            if os.environ.get('HEADING_PHYSICAL_TRUTH') == 'true':
                required += ['/simulation/ground_truth']
            if os.environ.get('HEADING_SHADOWS') == 'true':
                required += ['/diagnostic/global_no_gps_position', '/diagnostic/global_compensated_gps', '/diagnostic/global_delayed_gps_reference']
            if self.pub.get_subscription_count() < 1 or not all(self.counts.get(t,0)>2 for t in required):
                return
            self.start = self.now()
            self.directory.joinpath('sequence.json').write_text(json.dumps(
                dict(start_sim_s=self.start, sequence=self.sequence, scenario=self.scenario, truth_topic='/simulation/ground_truth' if os.environ.get('HEADING_PHYSICAL_TRUTH')=='true' else '/odom_raw'), indent=2))
        elapsed = self.now()-self.start
        boundary = 0
        command = Twist()
        for phase, duration, speed, rate in self.sequence:
            boundary += duration
            if elapsed < boundary:
                self.phase = phase
                command.linear.x, command.angular.z = speed, rate
                if self.scenario == "oscillations" and speed > 0:
                    command.angular.z = 0.06 * math.sin(2*math.pi*elapsed/1.0)
                break
        else:
            self.done = True
        self.pub.publish(command)
        self.record('/experiment/command', self.now(), speed=command.linear.x,
                    yaw_rate=command.angular.z)

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--scenario', choices=['right', 'left', 'gentle', 'oscillations'], default='right')
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    rclpy.init()
    node = Probe(args.output, args.scenario)
    deadline = time.monotonic()+180
    try:
        while not node.done and time.monotonic()<deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
        failures=[]
        if not node.done: failures.append('capture timeout')
        if node.counts.get('/gps/course_heading',0)==0: failures.append('no GPS heading measurements')
        args.output.joinpath('capture.json').write_text(json.dumps(
            dict(success=not failures, failures=failures, counts=node.counts, scope='simulation, open-loop'), indent=2))
        if failures: raise RuntimeError('; '.join(failures))
    finally:
        node.pub.publish(Twist())
        node.stream.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
