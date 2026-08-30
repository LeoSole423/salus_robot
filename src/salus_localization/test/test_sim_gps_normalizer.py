from pathlib import Path

from rclpy.qos import DurabilityPolicy, ReliabilityPolicy, qos_profile_sensor_data


PACKAGE = Path(__file__).parents[1]


def test_bridged_gps_input_uses_sensor_qos() -> None:
    source = (
        PACKAGE / "salus_localization" / "sim_gps_normalizer.py"
    ).read_text(encoding="utf-8")

    assert "from rclpy.qos import qos_profile_sensor_data" in source
    assert "self.on_fix,\n            qos_profile_sensor_data," in source
    assert qos_profile_sensor_data.reliability == ReliabilityPolicy.BEST_EFFORT
    assert qos_profile_sensor_data.durability == DurabilityPolicy.VOLATILE
