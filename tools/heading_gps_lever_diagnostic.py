#!/usr/bin/env python3
"""Simulation observer: antenna -> base diagnostic measurement, no TF/commands."""
from bisect import bisect_left
from collections import deque
from copy import deepcopy
import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry


def compensate_position(x, y, yaw, lever_x=0.66, lever_y=0.20):
    # Dimensions from salus_description GPS mount; diagnostic only.
    return (x-math.cos(yaw)*lever_x+math.sin(yaw)*lever_y,
            y-math.sin(yaw)*lever_x-math.cos(yaw)*lever_y)


def stamp(message):
    return message.header.stamp.sec+message.header.stamp.nanosec*1e-9


class Diagnostic(Node):
    def __init__(self):
        super().__init__('heading_gps_lever_diagnostic')
        self.history = deque(maxlen=600)
        self.pending = deque(maxlen=100)
        self.reference = self.create_publisher(Odometry, '/diagnostic/gps_ant_position_delayed', 10)
        self.publisher = self.create_publisher(Odometry, '/diagnostic/gps_base_position', 10)
        self.create_subscription(Odometry, '/odometry/local', self.on_local, 10)
        self.create_subscription(Odometry, '/gps/odometry_map', self.on_gps, 10)

    def on_local(self, msg):
        q = msg.pose.pose.orientation
        yaw = math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
        if self.history and stamp(msg) <= self.history[-1][0]:
            return
        self.history.append((stamp(msg),yaw))
        self.flush()

    def on_gps(self, msg):
        if msg.child_frame_id == 'base_footprint':
            raise ValueError('diagnostic requires raw antenna position; refusing double compensation')
        self.pending.append(deepcopy(msg))
        self.flush()

    def flush(self):
        if len(self.history) < 2:
            return
        times = [p[0] for p in self.history]
        while self.pending:
            msg = self.pending[0]
            t = stamp(msg)
            if t > times[-1]:
                break
            self.pending.popleft()
            i = bisect_left(times,t)
            if i == 0 or times[i]-times[i-1] > 0.2:
                continue
            a,b = self.history[i-1],self.history[i]
            delta = math.atan2(math.sin(b[1]-a[1]), math.cos(b[1]-a[1]))
            yaw = a[1]+delta*(t-a[0])/(b[0]-a[0])
            self.reference.publish(deepcopy(msg))
            msg.pose.pose.position.x, msg.pose.pose.position.y = compensate_position(
                msg.pose.pose.position.x, msg.pose.pose.position.y, yaw)
            # Keep original stamp and covariance for this isolated ablation.
            # Production would need orientation/mount uncertainty propagation.
            self.publisher.publish(msg)


if __name__ == '__main__':
    rclpy.init()
    node = Diagnostic()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
