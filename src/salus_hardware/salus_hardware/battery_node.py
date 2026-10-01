"""Isolated battery ROS adapter; it never starts or commands the drive UART."""
import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import BatteryState

from .battery_backends import create_battery_backend


def battery_message(sample, stamp):
    """Translate a measured sample without inventing unknown quantities."""
    msg = BatteryState()
    msg.header.stamp = stamp
    msg.present = True
    msg.voltage = sample.voltage
    msg.current = sample.current
    msg.charge = sample.charge
    msg.capacity = sample.capacity
    msg.design_capacity = math.nan
    msg.percentage = sample.percentage
    msg.temperature = sample.temperature
    msg.cell_voltage = list(sample.cell_voltage)
    # BMS temperatures are grouped sensors, not individually identified cells.
    msg.cell_temperature = [math.nan] * len(sample.cell_voltage)
    msg.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_LIFE
    msg.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_UNKNOWN
    msg.power_supply_status = (
        BatteryState.POWER_SUPPLY_STATUS_CHARGING if sample.current > 0 else
        BatteryState.POWER_SUPPLY_STATUS_DISCHARGING if sample.current < 0 else
        BatteryState.POWER_SUPPLY_STATUS_NOT_CHARGING)
    return msg


class BatteryNode(Node):
    """Publish successful reads only; consumers own their stale timeout."""

    def __init__(self):
        super().__init__('salus_battery')
        defaults = dict(backend='pylontech_us2000', serial_port='', baud=115200,
                        address=2, timeout_s=0.5, poll_hz=1.0,
                        state_topic='/battery/backend_state')
        for key, value in defaults.items():
            self.declare_parameter(key, value)
        values = {key: self.get_parameter(key).value for key in defaults}
        hz = float(values['poll_hz'])
        if not math.isfinite(hz) or not 0.1 <= hz <= 2.0:
            raise ValueError('poll_hz must be in [0.1, 2]')
        self._backend = create_battery_backend(
            values['backend'], port=values['serial_port'], baud=int(values['baud']),
            address=int(values['address']), timeout_s=float(values['timeout_s']))
        self._publisher = self.create_publisher(BatteryState, values['state_topic'], 10)
        self._failed = False
        self._timer = self.create_timer(1 / hz, self._poll)

    def _poll(self):
        try:
            sample = self._backend.read()
        except (OSError, ValueError) as exc:
            if not self._failed:
                self.get_logger().warning(f'Battery read failed: {exc}')
            self._failed = True
            return
        if self._failed:
            self.get_logger().info('Battery communication recovered')
        self._failed = False
        self._publisher.publish(battery_message(sample, self.get_clock().now().to_msg()))

    def destroy_node(self):
        self._backend.close()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = BatteryNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
