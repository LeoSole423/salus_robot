#!/usr/bin/env python3
"""Software-only raw adapter/selector/native navsat lever-arm comparison.

Requires an explicitly isolated ROS domain. Never subscribes to production
hardware, starts MAVROS, or publishes vehicle commands.
"""
import argparse
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
import yaml

from salus_hardware.pixhawk_sensor_adapter import PixhawkSensorAdapterNode
from salus_localization.gnss_selector import GnssSelectorNode


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if int(os.environ.get('ROS_DOMAIN_ID', '0')) != 159:
        raise SystemExit('Run in isolated ROS_DOMAIN_ID=159 (never production).')
    params = yaml.safe_load((Path(__file__).resolve().parents[1] /
        'src/salus_localization/config/localization_global_real.yaml').read_text())
    params = params['navsat_transform']['ros__parameters']
    params['datum'] = [42.0, 3.0, 0.0]  # synthetic point on a UTM central meridian
    processes = []
    logs = []
    rclpy.init()
    n = Node('antenna_fixture')
    adapter = PixhawkSensorAdapterNode(parameter_overrides=[
        Parameter('gnss_input_topic', value='/antenna_fixture/raw'),
        Parameter('gnss_output_topic', value='/antenna_fixture/identified'),
        Parameter('imu_input_topic', value='/antenna_fixture/unused_imu'),
        Parameter('imu_output_topic', value='/antenna_fixture/unused_imu_out'),
        Parameter('gnss_output_frame', value='gps_link'),
    ])
    selector = GnssSelectorNode(parameter_overrides=[
        Parameter('primary_topic', value='/antenna_fixture/identified'),
        Parameter('primary_frame', value='gps_link'),
        Parameter('output_topic', value='/antenna_fixture/logical'),
    ])
    executor = SingleThreadedExecutor()
    for node in [n, adapter, selector]: executor.add_node(node)
    pub_fix = n.create_publisher(NavSatFix, '/antenna_fixture/raw', 10)
    pub_odom = n.create_publisher(Odometry, '/antenna_fixture/odom', 10)
    dynamic = TransformBroadcaster(n)
    static = StaticTransformBroadcaster(n)
    t = TransformStamped()
    t.header.stamp = n.get_clock().now().to_msg()
    t.header.frame_id = 'base_link'; t.child_frame_id = 'gps_link'
    t.transform.translation.x = .66; t.transform.translation.y = .20
    t.transform.translation.z = .62; t.transform.rotation.w = 1.
    base = TransformStamped()
    base.header.stamp = t.header.stamp
    base.header.frame_id = 'base_footprint'; base.child_frame_id = 'base_link'
    base.transform.rotation.w = 1.
    static.sendTransform([base,t])
    received = {'legacy': [], 'corrected': []}
    subscriptions = []
    for label in received:
        subscriptions.append(n.create_subscription(Odometry,
            '/antenna_fixture/'+label, lambda m,l=label: received[l].append(m), 10))
    report = {'isolated_domain':159, 'angles':[], 'passed':False}
    try:
        with tempfile.TemporaryDirectory(prefix='salus-antenna-fixture-') as directory:
            config = Path(directory)/'params.yaml'
            config.write_text(yaml.safe_dump({'/**':{'ros__parameters':params}}))
            for label,topic in [('legacy','raw'),('corrected','logical')]:
                log = open(Path(directory)/(label+'.log'), 'w+')
                logs.append(log)
                processes.append(subprocess.Popen([
                    'ros2','run','robot_localization','navsat_transform_node',
                    '--ros-args','--params-file',str(config),
                    '-r','__node:=antenna_'+label,
                    '-r','gps/fix:=/antenna_fixture/'+topic,
                    '-r','odometry/filtered:=/antenna_fixture/odom',
                    '-r','odometry/gps:=/antenna_fixture/'+label,
                    '-r','gps/filtered:=/antenna_fixture/'+label+'_filtered',
                    '-r','imu:=/antenna_fixture/unused_orientation',
                    '-r','fromLL:=/antenna_fixture/'+label+'_fromLL',
                    '-r','toLL:=/antenna_fixture/'+label+'_toLL',
                    '-r','datum:=/antenna_fixture/'+label+'_datum',
                ], stdout=log, stderr=subprocess.STDOUT, start_new_session=True))
            for degrees in [0.,69.3,-69.3,90.]:
                yaw = math.radians(degrees)
                expected = (.66*math.cos(yaw)-.20*math.sin(yaw),
                            .66*math.sin(yaw)+.20*math.cos(yaw))
                end = time.monotonic()+12
                valid = None
                received['legacy'].clear(); received['corrected'].clear()
                started = n.get_clock().now().nanoseconds
                while time.monotonic()<end:
                    if any(p.poll() is not None for p in processes):
                        raise AssertionError('Native navsat process exited')
                    now = n.get_clock().now()
                    tf = TransformStamped()
                    tf.header.stamp = now.to_msg(); tf.header.frame_id='map'
                    tf.child_frame_id='base_footprint'
                    tf.transform.rotation.z=math.sin(yaw/2)
                    tf.transform.rotation.w=math.cos(yaw/2)
                    dynamic.sendTransform(tf)
                    odom=Odometry(); odom.header.stamp=now.to_msg()
                    odom.header.frame_id='map'; odom.child_frame_id='base_footprint'
                    odom.pose.pose.orientation=tf.transform.rotation
                    pub_odom.publish(odom)
                    # Allow native TF buffers to receive both surrounding transforms.
                    stamp = rclpy.time.Time(nanoseconds=now.nanoseconds-250000000)
                    fix=NavSatFix(); fix.header.stamp=stamp.to_msg()
                    fix.header.frame_id='base_link'; fix.status.status=0
                    fix.latitude=42.; fix.longitude=3.; fix.altitude=100.
                    fix.position_covariance=[.04,0.,0.,0.,.04,0.,0.,0.,.09]
                    fix.position_covariance_type=2; pub_fix.publish(fix)
                    until=time.monotonic()+.07
                    while time.monotonic()<until: executor.spin_once(timeout_sec=.01)
                    legacy={(m.header.stamp.sec,m.header.stamp.nanosec):m
                            for m in received['legacy']}
                    for corrected in received['corrected']:
                        key=(corrected.header.stamp.sec,corrected.header.stamp.nanosec)
                        if key[0]*1000000000+key[1] < started+500000000: continue
                        if key not in legacy: continue
                        old=legacy[key].pose.pose.position; new=corrected.pose.pose.position
                        delta=(old.x-new.x,old.y-new.y)
                        error=math.hypot(delta[0]-expected[0],delta[1]-expected[1])
                        if error < .005:
                            valid={'yaw_deg':degrees,'removed_xy_m':delta,
                                   'expected_xy_m':expected,'error_m':error}
                            break
                    if valid: break
                if valid is None: raise AssertionError('No correct native output at yaw '+str(degrees))
                report['angles'].append(valid)
            report['passed']=True
    finally:
        for process in processes:
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
                try: process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL); process.wait()
        for log in logs: log.close()
        executor.shutdown()
        for node in [selector,adapter,n]: node.destroy_node()
        rclpy.shutdown()
        args.output.parent.mkdir(parents=True,exist_ok=True)
        args.output.write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report))


if __name__ == '__main__': main()
