"""Check standard ROS units and unknown fields from recorded BMS measurements."""
import json
import math
from pathlib import Path

from builtin_interfaces.msg import Time
from sensor_msgs.msg import BatteryState

from salus_hardware.battery_node import battery_message
from salus_hardware.pylontech_protocol import decode_reply


def test_standard_battery_state_has_measured_soc_and_unknown_cell_temperatures():
    row = json.loads((Path(__file__).parent / 'fixtures' /
                      'pylontech_us2000_readings.json').read_text())[0]
    msg = battery_message(decode_reply(row['raw_frame'].encode()), Time(sec=10))
    assert msg.header.stamp.sec == 10
    assert msg.present
    assert math.isclose(msg.percentage, .93, rel_tol=1e-6)
    assert msg.power_supply_status == BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
    assert msg.power_supply_technology == BatteryState.POWER_SUPPLY_TECHNOLOGY_LIFE
    assert math.isnan(msg.design_capacity)
    assert len(msg.cell_voltage) == 15
    assert len(msg.cell_temperature) == 15
    assert all(math.isnan(t) for t in msg.cell_temperature)
    assert msg.power_supply_health == BatteryState.POWER_SUPPLY_HEALTH_UNKNOWN
