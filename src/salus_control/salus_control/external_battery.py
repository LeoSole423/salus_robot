"""Generic measured battery input; no hardware-specific imports."""
import math
from dataclasses import dataclass


@dataclass(frozen=True)
class ExternalBattery:
    """Validated snapshot of the standard backend BatteryState contract."""

    battery_voltage_v: float
    percentage: float
    current: float
    temperature: float
    charge: float
    capacity: float
    cell_voltage: tuple[float, ...]
    cell_temperature: tuple[float, ...]
    rx_monotonic_s: float
    message: object
    ready: bool = True
    fresh: bool = True
    suspect: bool = False

    def as_dict(self):
        return dict(battery_voltage_v=self.battery_voltage_v, percentage=self.percentage,
                    current_a=self.current, temperature_c=self.temperature,
                    charge_ah=self.charge, capacity_ah=self.capacity,
                    cell_voltage_v=list(self.cell_voltage), ready=self.ready,
                    fresh=self.fresh, suspect=self.suspect, source='external_bms')


def external_battery_sample(msg, received_s):
    """Require usable voltage/SOC; unknown optional fields may remain NaN."""
    if not msg.present or not math.isfinite(msg.voltage) or msg.voltage <= 0:
        raise ValueError('Battery is absent or voltage is invalid')
    if not math.isfinite(msg.percentage) or not 0 <= msg.percentage <= 1:
        raise ValueError('Battery SOC must be a fraction in [0, 1]')
    return ExternalBattery(
        msg.voltage, msg.percentage, msg.current, msg.temperature, msg.charge,
        msg.capacity, tuple(msg.cell_voltage), tuple(msg.cell_temperature),
        received_s, msg,
        suspect=msg.power_supply_health not in (
            msg.POWER_SUPPLY_HEALTH_UNKNOWN, msg.POWER_SUPPLY_HEALTH_GOOD))
