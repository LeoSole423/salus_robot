"""Observe BMI088 without changing the operational IMU or controlling actuators."""

from __future__ import annotations

from collections import deque
import json
import math
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, Temperature
from std_msgs.msg import String

from .bmi088_domain import Calibration, SequenceTracker, estimate_time, sample_stamp
from .bmi088_transport import ObservationClient


class Bmi088Node(Node):
    def __init__(self, *, parameter_overrides=None, start_worker=True):
        super().__init__('bmi088_observer', parameter_overrides=parameter_overrides)
        defaults = {
            'port': '', 'device_id': '', 'calibration_path': '', 'frame_id': 'bmi088_link',
            'poll_hz': 100.0, 'max_rtt_s': 0.02, 'max_sample_age_s': 0.1,
            'status_timeout_s': 0.2, 'gyro_filter_delay_s': 0.0, 'accel_filter_delay_s': 0.0,
            'gyro_variance_floor': 0.0001, 'accel_variance_floor': 0.01,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.config = {name: self.get_parameter(name).value for name in defaults}
        c = self.config
        if (not c['port'] or c['port'] == 'auto' or not c['frame_id'].strip() or
                c['frame_id'] in ('base_link', 'base_footprint', 'map', 'odom')):
            raise ValueError('explicit serial path and dedicated sensor frame are required')
        bounds = {'poll_hz': (1., 200.), 'max_rtt_s': (0.001, 0.1),
                  'max_sample_age_s': (0.01, 0.2), 'status_timeout_s': (0.02, 5.),
                  'gyro_filter_delay_s': (0., 0.05), 'accel_filter_delay_s': (0., 0.05),
                  'gyro_variance_floor': (1e-12, 1.), 'accel_variance_floor': (1e-12, 100.)}
        for key, (low, high) in bounds.items():
            if not math.isfinite(c[key]) or not low <= c[key] <= high:
                raise ValueError(f'{key} outside supported range')
        if self.get_parameter('use_sim_time').value:
            raise ValueError('physical BMI088 observation requires wall ROS time')
        self.calibration = (Calibration.load(c['calibration_path'], c['device_id'])
                            if c['calibration_path'] else None)
        prefix = '/hardware/imu_bmi088/'
        self.publishers_by_key = {
            k: self.create_publisher(Imu, prefix + k, qos_profile_sensor_data)
            for k in ('data_raw', 'data', 'accel_raw', 'accel')}
        self.temperature_pub = self.create_publisher(
            Temperature, prefix + 'temperature', qos_profile_sensor_data)
        self.status_pub = self.create_publisher(String, prefix + 'status', 10)
        self.tracker = SequenceTracker()
        self.pending = deque(maxlen=1)
        self.lock = threading.Lock()
        self.stop = threading.Event()
        self.last_good = None
        self.last_gyro_good = None
        self.last_accel_good = None
        self.last_temperature_stamp = 0
        self.last_temperature_device = None
        self.last_host_clock = None
        self.status = {'reason': 'waiting_for_usb', 'accepted_gyro': 0, 'accepted_accel': 0,
                       'rejected': 0, 'transport_errors': 0, 'hardware_parity': False}
        self.thread = None
        self.timer = self.create_timer(0.005, self._drain)
        self.status_timer = self.create_timer(0.2, self._status)
        if start_worker:
            self.thread = threading.Thread(target=self._read, daemon=True)
            self.thread.start()

    def _read(self):
        client = None
        try:
            while not self.stop.is_set():
                try:
                    if client is None:
                        client = ObservationClient(self.config['port'])
                    started = time.monotonic()
                    sample, sent, received = client.exchange()
                    ros_now = self.get_clock().now().nanoseconds
                    with self.lock:
                        self.pending.append((sample, sent, received, ros_now))
                    self.stop.wait(max(0., 1. / self.config['poll_hz'] -
                                       (time.monotonic() - started)))
                except (OSError, ValueError, TimeoutError) as error:
                    with self.lock:
                        self.status['transport_errors'] += 1
                        self.status['reason'] = str(error)
                        self.pending.clear()
                    if client is not None:
                        client.close()
                    client = None
                    self.stop.wait(1.)
        finally:
            if client is not None:
                client.close()

    def _message(self, key, values, stamp, variance):
        msg = Imu()
        msg.header.frame_id = self.config['frame_id']
        msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(stamp, 1_000_000_000)
        msg.orientation.w = 1.
        msg.orientation_covariance[0] = -1.
        msg.angular_velocity_covariance[0] = -1.
        msg.linear_acceleration_covariance[0] = -1.
        vector = msg.angular_velocity if key == 'gyro' else msg.linear_acceleration
        vector.x, vector.y, vector.z = values
        covariance = [variance[i // 3] if i in (0, 4, 8) else 0. for i in range(9)]
        if key == 'gyro':
            msg.angular_velocity_covariance = covariance
        else:
            msg.linear_acceleration_covariance = covariance
        return msg

    def process(self, sample, sent, received, ros_received):
        c = self.config
        timing = estimate_time(sample, sent, received, ros_received, int(c['max_rtt_s'] * 1e9))
        if self.last_host_clock is not None:
            old_mono, old_ros = self.last_host_clock
            if abs((ros_received - old_ros) - (received - old_mono)) > 50000000:
                self.last_host_clock = (received, ros_received)
                raise ValueError('ROS clock jump; sample rejected')
        self.last_host_clock = (received, ros_received)
        ros_now = self.get_clock().now().nanoseconds
        any_accepted = False
        channels = (('gyro', 'data_raw', 'data'), ('accel', 'accel_raw', 'accel'))
        for key, raw_topic, topic in channels:
            vector = getattr(sample, key)
            try:
                stamp = sample_stamp(
                    vector, sample, timing, ros_now, int(c['max_sample_age_s'] * 1e9),
                    int(c[key + '_filter_delay_s'] * 1e9))
            except ValueError:
                continue
            if not self.tracker.accept(sample, key, stamp):
                continue
            floor = c[key + '_variance_floor']
            raw_msg = self._message(key, vector.values, stamp, (floor,) * 3)
            self.publishers_by_key[raw_topic].publish(raw_msg)
            correction = getattr(self.calibration, key) if self.calibration else None
            values = correction.apply(vector.values) if correction else vector.values
            variance = (tuple(max(floor, v) for v in correction.variance)
                        if correction else (floor,) * 3)
            self.publishers_by_key[topic].publish(self._message(key, values, stamp, variance))
            self.status['accepted_' + key] += 1
            if key == 'gyro':
                self.last_gyro_good = time.monotonic()
            else:
                self.last_accel_good = time.monotonic()
            any_accepted = True
        if not any_accepted:
            raise ValueError('no new valid vectors')
        self.last_good = time.monotonic()
        warnings = []
        for key in ('gyro', 'accel'):
            correction = getattr(self.calibration, key) if self.calibration else None
            if correction and (not sample.temperature_valid or
                               abs(sample.temperature_c - correction.temperature_c) > 10):
                warnings.append(key + ': verify calibration temperature')
        self.status.update(reason='observing', boot_id=sample.boot_id,
                           init_attempts=sample.init_attempts, read_errors=sample.read_errors,
                           missed_accel=sample.missed_accel, missed_gyro=sample.missed_gyro,
                           timing_uncertainty_ms=timing.uncertainty_ns / 1e6,
                           rtt_ms=(received - sent) / 1e6, warnings=warnings)
        stamp = timing.ros_stamp(sample.temperature_us)
        temperature_key = (sample.boot_id, sample.init_attempts, sample.temperature_us)
        if (temperature_key != self.last_temperature_device and sample.temperature_valid and
                -40 <= sample.temperature_c <= 85 and
                stamp > self.last_temperature_stamp and
                0 <= ros_now - stamp <= 2000000000):
            msg = Temperature()
            msg.header.frame_id = c['frame_id']
            msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(stamp, 1000000000)
            msg.temperature = sample.temperature_c
            self.temperature_pub.publish(msg)
            self.last_temperature_stamp = stamp
            self.last_temperature_device = temperature_key

    def _drain(self):
        with self.lock:
            item = self.pending.pop() if self.pending else None
        if item is not None:
            try:
                self.process(*item)
            except ValueError as error:
                self.status['rejected'] += 1
                self.status['reason'] = str(error)

    def _status(self):
        now = time.monotonic()

        def fresh(stamp):
            return stamp is not None and now - stamp <= self.config['status_timeout_s']
        with self.lock:
            result = dict(self.status)
        result.update(fresh=fresh(self.last_good), gyro_fresh=fresh(self.last_gyro_good),
                      accel_fresh=fresh(self.last_accel_good),
                      calibration_profile=self.calibration.profile_id if self.calibration else '',
                      gyro_calibrated=bool(self.calibration and self.calibration.gyro),
                      accel_calibrated=bool(self.calibration and self.calibration.accel),
                      frame_id=self.config['frame_id'], operational_source_changed=False,
                      hardware_synchronized=False, timestamp_basis='USB_roundtrip_MCU_DRDY',
                      filter_delay_compensated=bool(self.config['gyro_filter_delay_s'] and
                                                    self.config['accel_filter_delay_s']))
        message = String()
        message.data = json.dumps(result, allow_nan=False)
        self.status_pub.publish(message)

    def destroy_node(self):
        self.stop.set()
        if self.thread:
            self.thread.join(timeout=2.)
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = Bmi088Node()
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
