import rclpy
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, ReliabilityPolicy,
                       qos_profile_sensor_data)

from salus_perception.scan_ground_filter import ScanGroundFilter, ground_filter_input_qos


def test_ground_filter_input_qos_keeps_only_the_latest_best_effort_cloud() -> None:
    qos = ground_filter_input_qos()

    assert qos.history == HistoryPolicy.KEEP_LAST
    assert qos.depth == 1
    assert qos.reliability == ReliabilityPolicy.BEST_EFFORT
    assert qos.durability == DurabilityPolicy.VOLATILE


def test_ground_filter_input_qos_does_not_mutate_sensor_data_profile() -> None:
    assert qos_profile_sensor_data.history == HistoryPolicy.KEEP_LAST
    assert qos_profile_sensor_data.depth == 5
    assert qos_profile_sensor_data.reliability == ReliabilityPolicy.BEST_EFFORT
    assert qos_profile_sensor_data.durability == DurabilityPolicy.VOLATILE


def test_ground_filter_output_keeps_sensor_data_qos() -> None:
    rclpy.init()
    node = ScanGroundFilter()
    try:
        qos = node.pub.qos_profile
        assert qos.history == HistoryPolicy.KEEP_LAST
        assert qos.depth == 5
        assert qos.reliability == ReliabilityPolicy.BEST_EFFORT
        assert qos.durability == DurabilityPolicy.VOLATILE
    finally:
        node.destroy_node()
        rclpy.shutdown()
