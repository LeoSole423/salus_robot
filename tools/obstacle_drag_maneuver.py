#!/usr/bin/env python3
"""Run one bounded open-loop Ackermann maneuver for obstacle-drag measurement."""

from __future__ import annotations

import sys
import argparse

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import String


ENTRY_S = 2.0
TURN_S = 10.0
EXIT_S = 2.0
STOP_S = 1.0
PAUSE_S = 6.0
SPEED_MPS = 0.5
TURN_RATE_RPS = -0.125  # 4 m radius at 0.5 m/s; right turn stays clear of near post.


class ObstacleDragManeuver(Node):
    """Publish the existing /cmd_vel input through one deterministic sequence."""

    def __init__(self, repetitions: int = 1, pause_s: float = PAUSE_S) -> None:
        super().__init__("obstacle_drag_maneuver")
        if repetitions < 1:
            raise ValueError("repetitions must be positive")
        if pause_s < 0.0:
            raise ValueError("pause_s must be non-negative")
        self.repetitions = repetitions
        self.pause_s = pause_s
        self.publisher = self.create_publisher(Twist, "/cmd_vel", 10)
        self.phase_publisher = self.create_publisher(String, "/obstacle_drag/phase", 10)
        self.started_s: float | None = None
        self.timer = self.create_timer(0.04, self._tick)

    def _tick(self) -> None:
        if self.publisher.get_subscription_count() < 1:
            return
        now_s = self.get_clock().now().nanoseconds * 1.0e-9
        if self.started_s is None:
            self.started_s = now_s
        elapsed_s = now_s - self.started_s
        cycle_s = ENTRY_S + TURN_S + EXIT_S + STOP_S + self.pause_s
        message = Twist()
        if elapsed_s >= cycle_s * self.repetitions:
            self.publisher.publish(message)
            self._publish_phase("done")
            self.timer.cancel()
            rclpy.shutdown()
            return
        cycle_index = min(int(elapsed_s // cycle_s), self.repetitions - 1)
        cycle_elapsed = elapsed_s - cycle_index * cycle_s
        prefix = f"turn_{cycle_index + 1}"
        if cycle_elapsed < ENTRY_S:
            message.linear.x = SPEED_MPS
            phase = f"entry_{cycle_index + 1}"
        elif cycle_elapsed < ENTRY_S + TURN_S:
            message.linear.x = SPEED_MPS
            message.angular.z = TURN_RATE_RPS
            phase = prefix
        elif cycle_elapsed < ENTRY_S + TURN_S + EXIT_S:
            message.linear.x = SPEED_MPS
            phase = f"exit_{cycle_index + 1}"
        elif cycle_elapsed < ENTRY_S + TURN_S + EXIT_S + STOP_S:
            phase = f"stop_{cycle_index + 1}"
        else:
            phase = f"pause_{cycle_index + 1}"
        self._publish_phase(phase)
        self.publisher.publish(message)

    def _publish_phase(self, phase: str) -> None:
        message = String()
        message.data = phase
        self.phase_publisher.publish(message)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--repetitions", type=int, default=1)
    parser.add_argument("--pause-s", type=float, default=PAUSE_S)
    args, _ = parser.parse_known_args()
    rclpy.init()
    node = ObstacleDragManeuver(args.repetitions, args.pause_s)
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
