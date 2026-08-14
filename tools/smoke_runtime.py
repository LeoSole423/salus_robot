#!/usr/bin/env python3
"""Event-driven runtime shared by ROS 2 smoke scenarios."""

from __future__ import annotations

import json
import math
import time
from dataclasses import asdict, dataclass, field
from enum import Enum
from pathlib import Path
from typing import Any, Callable

import rclpy


class PhaseState(str, Enum):
    WAITING = "WAITING"
    RUNNING = "RUNNING"
    PASSED = "PASSED"
    FAILED = "FAILED"
    TIMED_OUT = "TIMED_OUT"


@dataclass
class PhaseReport:
    name: str
    state: str = PhaseState.WAITING.value
    timeout_s: float = 0.0
    duration_s: float = 0.0
    iterations: int = 0
    last_condition: Any = None
    error: str = ""


@dataclass
class ScenarioReport:
    scenario: str
    state: str = PhaseState.WAITING.value
    duration_s: float = 0.0
    phases: list[PhaseReport] = field(default_factory=list)
    evidence: dict[str, Any] = field(default_factory=dict)
    error: str = ""


class SmokeTimeout(RuntimeError):
    pass


class SmokeRuntime:
    """Bounded ROS executor loop with mandatory JSON evidence."""

    def __init__(self, node, scenario: str, report_path: Path, global_timeout_s: float = 110.0):
        self.node = node
        self.report_path = Path(report_path)
        self.started = time.monotonic()
        self.deadline = self.started + global_timeout_s
        self.report = ScenarioReport(scenario=scenario, state=PhaseState.RUNNING.value)

    def _remaining(self) -> float:
        return max(0.0, self.deadline - time.monotonic())

    def spin(self, timeout_s: float = 0.05) -> None:
        rclpy.spin_once(self.node, timeout_sec=min(timeout_s, self._remaining()))

    def wait(self, name: str, predicate: Callable[[], bool], timeout_s: float, *,
             observe: Callable[[], Any] | None = None,
             stimulate: Callable[[], None] | None = None) -> None:
        phase = PhaseReport(name=name, state=PhaseState.RUNNING.value, timeout_s=timeout_s)
        self.report.phases.append(phase)
        started = time.monotonic()
        deadline = min(started + timeout_s, self.deadline)
        try:
            while time.monotonic() < deadline:
                if stimulate is not None:
                    stimulate()
                self.spin()
                phase.iterations += 1
                if observe is not None:
                    phase.last_condition = observe()
                if predicate():
                    phase.state = PhaseState.PASSED.value
                    return
            phase.state = PhaseState.TIMED_OUT.value
            raise SmokeTimeout(
                f"phase {name!r} timed out after {time.monotonic() - started:.2f}s; "
                f"last={phase.last_condition!r}"
            )
        except SmokeTimeout:
            raise
        except Exception as exc:
            phase.state = PhaseState.FAILED.value
            phase.error = str(exc)
            raise
        finally:
            phase.duration_s = time.monotonic() - started

    def wait_publisher_match(self, name: str, publisher, minimum: int = 1,
                             timeout_s: float = 20.0) -> None:
        self.wait(name, lambda: publisher.get_subscription_count() >= minimum, timeout_s,
                  observe=lambda: {"subscriptions": publisher.get_subscription_count(),
                                   "minimum": minimum})

    def wait_topic_publishers(self, name: str, topic: str, minimum: int = 1,
                              timeout_s: float = 20.0) -> None:
        self.wait(name, lambda: self.node.count_publishers(topic) >= minimum, timeout_s,
                  observe=lambda: {"publishers": self.node.count_publishers(topic),
                                   "minimum": minimum})

    def call(self, name: str, client, request, timeout_s: float = 10.0):
        self.wait(f"{name}:service", client.service_is_ready, timeout_s,
                  observe=lambda: {"service_ready": client.service_is_ready()})
        future = client.call_async(request)
        self.wait(f"{name}:response", future.done, timeout_s,
                  observe=lambda: {"done": future.done()})
        result = future.result()
        if result is None:
            raise RuntimeError(f"service {name!r} returned no response")
        return result

    def wait_action(self, name: str, action_client, timeout_s: float = 15.0) -> None:
        self.wait(
            f"action {name}",
            lambda: action_client.wait_for_server(timeout_sec=0.0),
            timeout_s,
            observe=lambda: {"ready": action_client.server_is_ready()},
        )

    def wait_transform(self, name: str, tf_buffer, target: str, source: str,
                       timeout_s: float = 15.0) -> None:
        from rclpy.time import Time
        self.wait(
            f"TF {name}",
            lambda: tf_buffer.can_transform(target, source, Time()),
            timeout_s,
            observe=lambda: {"target": target, "source": source},
        )

    def wait_lifecycle(self, node_name: str, timeout_s: float = 20.0) -> None:
        from lifecycle_msgs.srv import GetState
        client = self.node.create_client(GetState, f"{node_name.rstrip('/')}/get_state")
        latest = {"future": None, "label": "service unavailable"}

        def poll() -> bool:
            future = latest["future"]
            if future is not None and future.done():
                result = future.result()
                latest["label"] = result.current_state.label if result else "invalid response"
                latest["future"] = None
                return latest["label"] == "active"
            return False

        def request_state() -> None:
            if client.service_is_ready() and latest["future"] is None:
                latest["future"] = client.call_async(GetState.Request())

        self.wait(
            f"lifecycle {node_name}", poll, timeout_s,
            stimulate=request_state,
            observe=lambda: {"service_ready": client.service_is_ready(),
                             "last_state": latest["label"]},
        )

    def finish(self, success: bool, *, error: BaseException | None = None,
               evidence: dict[str, Any] | None = None) -> None:
        if evidence:
            self.report.evidence.update(evidence)
        self.report.duration_s = time.monotonic() - self.started
        self.report.state = PhaseState.PASSED.value if success else PhaseState.FAILED.value
        if error is not None:
            self.report.error = f"{type(error).__name__}: {error}"
        self.report_path.parent.mkdir(parents=True, exist_ok=True)
        with self.report_path.open("w", encoding="utf-8") as stream:
            json.dump(asdict(self.report), stream, indent=2, sort_keys=True)
            stream.write("\n")


def stamp_ns(message) -> int:
    stamp = message.header.stamp
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def has_increasing_stamps(messages: list[Any], minimum: int = 2) -> bool:
    if len(messages) < minimum:
        return False
    stamps = [stamp_ns(message) for message in messages[-minimum:]]
    return all(current > previous for previous, current in zip(stamps, stamps[1:]))


def finite_odometry(message) -> bool:
    pose = message.pose.pose
    twist = message.twist.twist
    values = [pose.position.x, pose.position.y, pose.position.z,
              pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w,
              twist.linear.x, twist.linear.y, twist.linear.z,
              twist.angular.x, twist.angular.y, twist.angular.z,
              *message.pose.covariance, *message.twist.covariance]
    norm = math.sqrt(sum(value * value for value in
                         (pose.orientation.x, pose.orientation.y,
                          pose.orientation.z, pose.orientation.w)))
    return all(math.isfinite(value) for value in values) and norm > 1.0e-6
