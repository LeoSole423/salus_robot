"""Real pyserial + ROS/DDS against a virtual USB device, never physical hardware."""
import json
import os
import select
import threading
import time

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import Imu
from std_msgs.msg import String
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

from salus_hardware.bmi088_domain import crc8
from salus_hardware.bmi088_transport import ObservationClient
from salus_hardware.bmi088_node import Bmi088Node
from salus_hardware.imu_comparison_node import ImuComparisonNode
from test_bmi088_domain import packet


class VirtualEsp32:
    def __init__(self):
        self.master, self.slave = os.openpty()
        self.path = os.ttyname(self.slave)
        self.stop = threading.Event()
        self.pause = threading.Event()
        self.requests = []
        self.started = time.monotonic_ns() // 1000 - 1000000
        self.thread = threading.Thread(target=self.run, daemon=True)
        self.thread.start()

    def run(self):
        buffer = bytearray()
        while not self.stop.is_set():
            ready, _, _ = select.select([self.master], [], [], 0.02)
            if not ready:
                continue
            try:
                buffer.extend(os.read(self.master, 1024))
                while len(buffer) >= 7:
                    command = bytes(buffer[:7])
                    del buffer[:7]
                    self.requests.append(command)
                    if command[:3] != bytes([0xAB, 0x10, 3]) or crc8(command[:6]) != command[6]:
                        continue
                    if self.pause.is_set():
                        continue
                    received = time.monotonic_ns() // 1000 - self.started
                    now = time.monotonic_ns() // 1000 - self.started
                    token = int.from_bytes(command[3:6], 'little')
                    # A valid old-token response must be ignored, then match current token.
                    old = packet(token=token ^ 1, now=now, received=received, sequence=now//2500)
                    valid = packet(token=token, now=now, received=received, sequence=now//2500)
                    os.write(self.master, b'noise' + old + valid)
            except OSError:
                return

    def close(self):
        self.stop.set()
        self.thread.join(timeout=1)
        os.close(self.master)
        os.close(self.slave)


def test_real_serial_exclusive_transport_only_sends_tagged_diagnostics():
    firmware = VirtualEsp32()
    client = ObservationClient(firmware.path)
    try:
        sample, sent, received = client.exchange()
        assert sample.token == client.token and sent < received
        assert sample.capture_us >= sample.received_us
        assert firmware.requests
        assert all(r[:3] == bytes([0xAB, 0x10, 3]) for r in firmware.requests)
    finally:
        client.close()
        firmware.close()


def test_ros_usb_comparison_smoke_and_disconnect_staleness():
    firmware = VirtualEsp32()
    rclpy.init()
    observer = Bmi088Node(parameter_overrides=[Parameter('port', value=firmware.path)])
    comparator = ImuComparisonNode()
    probe = Node('bmi088_virtual_usb_probe')
    executor = SingleThreadedExecutor()
    for node in (observer, comparator, probe):
        executor.add_node(node)
    raw, accel, statuses, comparisons = [], [], [], []
    probe.create_subscription(
        Imu, '/hardware/imu_bmi088/data_raw', raw.append, qos_profile_sensor_data)
    probe.create_subscription(
        Imu, '/hardware/imu_bmi088/accel_raw', accel.append, qos_profile_sensor_data)
    probe.create_subscription(String, '/hardware/imu_bmi088/status',
                              lambda m: statuses.append(json.loads(m.data)), 10)
    probe.create_subscription(String, '/hardware/imu_bmi088/comparison',
                              lambda m: comparisons.append(json.loads(m.data)), 10)
    publisher = probe.create_publisher(Imu, '/hardware/imu_primary/data', qos_profile_sensor_data)
    broadcaster = StaticTransformBroadcaster(probe)
    tf = TransformStamped()
    tf.header.stamp = probe.get_clock().now().to_msg()
    tf.header.frame_id = 'base_footprint'
    tf.child_frame_id = 'bmi088_link'
    tf.transform.rotation.w = 1.
    broadcaster.sendTransform(tf)
    try:
        deadline = time.monotonic()+8
        while time.monotonic() < deadline:
            reference = Imu()
            ns = probe.get_clock().now().nanoseconds-2000000
            reference.header.stamp.sec, reference.header.stamp.nanosec = divmod(ns, 1000000000)
            reference.header.frame_id = 'base_footprint'
            reference.angular_velocity.x = 0.01
            reference.angular_velocity.y = -0.02
            reference.angular_velocity.z = 0.03
            publisher.publish(reference)
            executor.spin_once(timeout_sec=0.01)
            if len(raw) >= 5 and accel and statuses and comparisons and comparisons[-1]['fresh']:
                break
        assert len(raw) >= 5 and accel and statuses[-1]['gyro_fresh']
        assert comparisons[-1]['fresh'], comparisons[-1] if comparisons else 'no comparison'
        assert max(comparisons[-1]['rms_error_rad_s']) < 1e-6
        assert raw[-1].orientation_covariance[0] == -1
        assert accel[-1].angular_velocity_covariance[0] == -1
        assert raw[-1].header.frame_id == 'bmi088_link'
        assert not statuses[-1]['operational_source_changed']
        firmware.pause.set()
        deadline = time.monotonic()+3
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.02)
            if statuses and not statuses[-1]['gyro_fresh']:
                break
        assert not statuses[-1]['gyro_fresh']
        assert all(r[:3] == bytes([0xAB, 0x10, 3]) for r in firmware.requests)
    finally:
        executor.shutdown()
        for node in (observer, comparator, probe):
            node.destroy_node()
        rclpy.shutdown()
        firmware.close()
