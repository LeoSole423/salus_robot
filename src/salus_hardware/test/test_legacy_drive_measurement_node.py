from pathlib import Path
import math
import time

import pytest
import rclpy
from interfaces.msg import DriveTelemetry
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from salus_hardware.legacy_drive_measurement_node import LegacyDriveMeasurementNode
from salus_interfaces.msg import SteeringMeasurement, TractionMeasurement


class CapturingPublisher:
    def __init__(self) -> None:
        self.messages = []

    def publish(self, message: object) -> None:
        self.messages.append(message)


@pytest.fixture
def node():
    rclpy.init()
    instance = LegacyDriveMeasurementNode()
    try:
        yield instance
    finally:
        instance.destroy_node()
        rclpy.shutdown()


def test_node_defaults_and_callback_copy_stamp_and_field_provenance(node) -> None:
    assert node._traction_publisher.topic_name == "/vehicle/measurements/traction"
    assert node._steering_publisher.topic_name == "/vehicle/measurements/steering"
    traction_publisher = CapturingPublisher()
    steering_publisher = CapturingPublisher()
    node._traction_publisher = traction_publisher
    node._steering_publisher = steering_publisher
    legacy = DriveTelemetry()
    legacy.stamp.sec = 12
    legacy.stamp.nanosec = 34
    legacy.fresh = True
    legacy.reverse_requested = True
    legacy.speed_valid = True
    legacy.steer_valid = True
    legacy.speed_mps_measured = 2.0
    legacy.steer_deg_measured = 180.0

    node._on_telemetry(legacy)

    traction = traction_publisher.messages[0]
    steering = steering_publisher.messages[0]
    assert traction.metadata.header.stamp.sec == 12
    assert traction.metadata.header.stamp.nanosec == 34
    assert traction.metadata.source_id == "rear_traction_motor"
    assert traction.metadata.sequence == 1
    assert traction.linear_velocity_mps == -2.0
    assert math.isnan(traction.position_rad)
    assert math.isnan(traction.angular_velocity_rad_s)
    assert traction.inferred_fields == traction.FIELD_LINEAR_VELOCITY
    assert traction.calculated_fields == 0
    assert steering.metadata.source_id == "front_steering_linkage"
    assert steering.metadata.sequence == 1
    assert steering.position_rad == pytest.approx(3.141592653589793)
    assert math.isnan(steering.angular_velocity_rad_s)
    assert steering.calculated_fields == steering.FIELD_POSITION


def test_console_entry_point_is_packaged() -> None:
    setup = (Path(__file__).parents[1] / "setup.py").read_text(encoding="utf-8")
    entry_point = (
        "legacy_drive_measurement_node = "
        "salus_hardware.legacy_drive_measurement_node:main"
    )
    assert entry_point in setup


def test_sequence_wraps_without_overflowing_uint32(node) -> None:
    traction_publisher = CapturingPublisher()
    steering_publisher = CapturingPublisher()
    node._traction_publisher = traction_publisher
    node._steering_publisher = steering_publisher
    node._sequence = 0xFFFFFFFF
    legacy = DriveTelemetry()
    legacy.fresh = True
    legacy.speed_valid = True
    legacy.steer_valid = True

    node._on_telemetry(legacy)

    assert traction_publisher.messages[0].metadata.sequence == 0
    assert steering_publisher.messages[0].metadata.sequence == 0


def test_real_legacy_wire_publisher_reaches_canonical_measurements() -> None:
    """The DDS boundary accepts ``interfaces/msg/DriveTelemetry``, not a mock."""
    rclpy.init()
    adapter = LegacyDriveMeasurementNode()
    publisher_node = Node("legacy_drive_telemetry_wire_publisher")
    observer = Node("canonical_drive_measurement_observer")
    executor = SingleThreadedExecutor()
    traction_messages = []
    steering_messages = []
    try:
        publisher = publisher_node.create_publisher(
            DriveTelemetry, "/controller/drive_telemetry", 10
        )
        observer.create_subscription(
            TractionMeasurement,
            "/vehicle/measurements/traction",
            traction_messages.append,
            10,
        )
        observer.create_subscription(
            SteeringMeasurement,
            "/vehicle/measurements/steering",
            steering_messages.append,
            10,
        )
        for ros_node in (adapter, publisher_node, observer):
            executor.add_node(ros_node)

        legacy = DriveTelemetry()
        legacy.stamp.sec = 12
        legacy.stamp.nanosec = 34
        legacy.fresh = True
        legacy.reverse_requested = True
        legacy.speed_valid = True
        legacy.steer_valid = True
        legacy.speed_mps_measured = 2.0
        legacy.steer_deg_measured = 180.0

        deadline = time.monotonic() + 3.0
        while time.monotonic() < deadline and (
            not traction_messages or not steering_messages
        ):
            publisher.publish(legacy)
            executor.spin_once(timeout_sec=0.05)

        assert traction_messages and steering_messages
        topic_types = dict(publisher_node.get_topic_names_and_types())
        assert topic_types["/controller/drive_telemetry"] == [
            "interfaces/msg/DriveTelemetry"
        ]
        traction = traction_messages[-1]
        steering = steering_messages[-1]
        assert (
            traction.metadata.header.stamp.sec,
            traction.metadata.header.stamp.nanosec,
        ) == (12, 34)
        assert traction.metadata.source_id == "rear_traction_motor"
        assert steering.metadata.source_id == "front_steering_linkage"
        assert traction.metadata.sequence == steering.metadata.sequence == 1
        assert traction.linear_velocity_mps == -2.0
        assert steering.position_rad == pytest.approx(math.pi)
        assert traction.inferred_fields == traction.FIELD_LINEAR_VELOCITY
        assert steering.calculated_fields == steering.FIELD_POSITION
    finally:
        for ros_node in (observer, publisher_node, adapter):
            executor.remove_node(ros_node)
            ros_node.destroy_node()
        executor.shutdown()
        rclpy.shutdown()
