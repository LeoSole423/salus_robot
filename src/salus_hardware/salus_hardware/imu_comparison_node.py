"""Compare two IMUs in one TF frame; metrics never switch a sensor source."""

from collections import deque
import json
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import Imu
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener, TransformException

from .imu_comparison_domain import metrics, rotate


def stamp_ns(message):
    return message.header.stamp.sec * 1000000000 + message.header.stamp.nanosec


class ImuComparisonNode(Node):
    def __init__(self, *, parameter_overrides=None):
        super().__init__('imu_comparison', parameter_overrides=parameter_overrides)
        defaults = {'candidate_topic': '/hardware/imu_bmi088/data',
                    'reference_topic': '/hardware/imu_primary/data',
                    'target_frame': 'base_footprint', 'max_pair_dt_s': 0.03,
                    'max_age_s': 0.2, 'window_s': 5.0}
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.config = {name: self.get_parameter(name).value for name in defaults}
        for key in ('candidate_topic', 'reference_topic', 'target_frame'):
            if not self.config[key].strip():
                raise ValueError('comparison topics/frame must not be empty')
        for key, bounds in {'max_pair_dt_s': (0.001, 0.1), 'max_age_s': (0.01, 2.),
                            'window_s': (0.1, 60.)}.items():
            if (not math.isfinite(self.config[key]) or
                    not bounds[0] <= self.config[key] <= bounds[1]):
                raise ValueError('invalid comparison interval')
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)
        self.reference = deque(maxlen=1000)
        self.candidate = deque(maxlen=1000)
        self.pairs = deque(maxlen=10000)
        self.reason = 'waiting_for_samples'
        self.counts = {'candidate': 0, 'reference': 0, 'rejected': 0, 'unpaired': 0}
        self.last_stamps = {}
        self.last_pair_mono = None
        self.last_ros_ns = None
        self.create_subscription(
            Imu, self.config['candidate_topic'],
            lambda msg: self._accept('candidate', msg), qos_profile_sensor_data)
        self.create_subscription(
            Imu, self.config['reference_topic'],
            lambda msg: self._accept('reference', msg), qos_profile_sensor_data)
        self.publisher = self.create_publisher(String, '/hardware/imu_bmi088/comparison', 10)
        self.create_timer(0.02, self._pair)
        self.create_timer(0.5, self._report)

    def _accept(self, source, msg):
        now = self.get_clock().now().nanoseconds
        stamp = stamp_ns(msg)
        values = (msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z)
        if (not 0 < stamp <= now or now-stamp > self.config['max_age_s']*1e9 or
                stamp <= self.last_stamps.get(source, 0) or
                msg.angular_velocity_covariance[0] < 0 or
                not all(math.isfinite(v) for v in (*values, *msg.angular_velocity_covariance)) or
                not msg.header.frame_id):
            self.counts['rejected'] += 1
            return
        self.last_stamps[source] = stamp
        self.counts[source] += 1
        getattr(self, source).append(msg)

    def _vector(self, msg):
        values = (msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z)
        if msg.header.frame_id == self.config['target_frame']:
            return values
        transform = self.tf.lookup_transform(
            self.config['target_frame'], msg.header.frame_id, Time.from_msg(msg.header.stamp))
        q = transform.transform.rotation
        return rotate(values, (q.x, q.y, q.z, q.w))

    def _pair(self):
        now = self.get_clock().now().nanoseconds
        if self.last_ros_ns is not None and now < self.last_ros_ns:
            self.reference.clear()
            self.candidate.clear()
            self.pairs.clear()
            self.last_stamps.clear()
            self.last_pair_mono = None
            self.reason = 'ROS_clock_reversed'
        self.last_ros_ns = now
        age = int(self.config['max_age_s'] * 1e9)
        while self.reference and now - stamp_ns(self.reference[0]) > age:
            self.reference.popleft()
        while self.candidate:
            candidate = self.candidate[0]
            if now - stamp_ns(candidate) > age:
                self.candidate.popleft()
                self.counts['unpaired'] += 1
                continue
            if not self.reference:
                self.reason = 'waiting_for_reference'
                break
            reference = min(self.reference, key=lambda m: abs(stamp_ns(m)-stamp_ns(candidate)))
            delta = abs(stamp_ns(reference)-stamp_ns(candidate))
            if delta > self.config['max_pair_dt_s']*1e9:
                self.reason = 'waiting_for_timestamp_match'
                break
            try:
                a, b = self._vector(candidate), self._vector(reference)
            except (TransformException, ValueError) as error:
                self.reason = 'waiting_for_valid_tf: ' + str(error)
                break
            self.candidate.popleft()
            self.reference.remove(reference)
            self.pairs.append((time.monotonic(), tuple(x-y for x, y in zip(a, b)), delta/1e6))
            self.last_pair_mono = time.monotonic()
            self.reason = 'comparing'

    def _report(self):
        now = time.monotonic()
        while self.pairs and now-self.pairs[0][0] > self.config['window_s']:
            self.pairs.popleft()
        fresh = (self.last_pair_mono is not None and
                 now-self.last_pair_mono <= self.config['max_age_s'])
        result = metrics([p[1] for p in self.pairs])
        result.update(fresh=fresh, reason=self.reason if fresh else 'no_recent_comparable_pair',
                      detail=self.reason, counts=dict(self.counts),
                      mean_pair_dt_ms=(sum(p[2] for p in self.pairs)/len(self.pairs)
                                       if self.pairs else None),
                      target_frame=self.config['target_frame'], hardware_parity=False,
                      operational_source_changed=False)
        msg = String()
        msg.data = json.dumps(result, allow_nan=False)
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = ImuComparisonNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
