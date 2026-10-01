"""Battery adapter contract, independent of ROS and serial implementation."""
from dataclasses import dataclass
from typing import Protocol


@dataclass(frozen=True)
class BatterySample:
    """SI values; capacities in Ah and percentage as a fraction."""

    voltage: float
    current: float
    charge: float
    capacity: float
    cell_voltage: tuple[float, ...]
    grouped_cell_temperatures: tuple[float, ...]
    temperature: float
    cycles: int

    @property
    def percentage(self) -> float:
        return self.charge / self.capacity


class BatteryBackend(Protocol):
    """One bounded read, no battery settings or actuator writes."""

    def read(self) -> BatterySample:
        ...

    def close(self) -> None:
        ...
