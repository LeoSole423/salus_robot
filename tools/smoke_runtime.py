#!/usr/bin/env python3
"""Event-driven runtime shared by ROS 2 smoke scenarios."""

from __future__ import annotations

import json
import math
import os
import time
from dataclasses import asdict, dataclass, field
from enum import Enum
from pathlib import Path
from typing import Any, Callable

import rclpy
from diagnostic_msgs.msg import DiagnosticArray


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


class TopicReadinessState(str, Enum):
    NO_PUBLISHER = "NO_PUBLISHER"
    NO_MESSAGES = "NO_MESSAGES"
    INVALID = "INVALID"
    NOT_PROGRESSIVE = "NOT_PROGRESSIVE"
    READY = "READY"


@dataclass
class TopicEvidence:
    """Reusable causal evidence for a required ROS topic."""

    label: str = ""
    started_at_s: float = field(default_factory=time.monotonic)
    received: int = 0
    valid: int = 0
    first_latency_s: float | None = None
    timestamps_ns: list[int] = field(default_factory=list)
    frames: list[str] = field(default_factory=list)
    errors: list[str] = field(default_factory=list)

    def record(self, message: Any, validator: Callable[[Any], Any]) -> None:
        self.received += 1
        if self.first_latency_s is None:
            self.first_latency_s = max(0.0, time.monotonic() - self.started_at_s)
        try:
            result = validator(message)
            if result is False:
                raise ValueError("validator rejected message")
        except ValueError as exc:
            self.errors.append(str(exc))
            return
        self.valid += 1
        self.timestamps_ns.append(stamp_ns(message))
        header = getattr(message, "header", None)
        self.frames.append(getattr(header, "frame_id", "") if header is not None else "")

    @property
    def has_progress(self) -> bool:
        return (
            len(self.timestamps_ns) >= 2
            and self.timestamps_ns[-1] > self.timestamps_ns[-2]
        )

    def state(self, *, publisher_count: int | None = None) -> TopicReadinessState:
        if publisher_count is not None and publisher_count <= 0:
            return TopicReadinessState.NO_PUBLISHER
        if self.received == 0:
            return TopicReadinessState.NO_MESSAGES
        if self.valid == 0:
            return TopicReadinessState.INVALID
        if not self.has_progress:
            return TopicReadinessState.NOT_PROGRESSIVE
        return TopicReadinessState.READY

    def snapshot(self, *, publisher_count: int | None = None) -> dict[str, Any]:
        state = self.state(publisher_count=publisher_count)
        return {
            "label": self.label,
            "state": state.value,
            "publisher_count": publisher_count,
            "received": self.received,
            "valid": self.valid,
            "first_latency_s": self.first_latency_s,
            "timestamps_ns": self.timestamps_ns[-3:],
            "frames": self.frames[-3:],
            "errors": self.errors[-3:],
            "progressive": self.has_progress,
        }


@dataclass
class NavigationStartupEvidence:
    """Last causal startup diagnostic published by the Nav2 coordinator."""

    messages: int = 0
    state: str = "UNSEEN"
    reason: str = "no navigation startup diagnostic received"
    values: dict[str, str] = field(default_factory=dict)
    last_received_monotonic_s: float | None = None

    def record(self, message: DiagnosticArray) -> None:
        for status in message.status:
            if status.name != "navigation_startup":
                continue
            self.messages += 1
            self.last_received_monotonic_s = time.monotonic()
            self.values = {item.key: item.value for item in status.values}
            self.state = self.values.get("state", "UNKNOWN")
            self.reason = self.values.get("reason", status.message)

    @property
    def active(self) -> bool:
        return self.state == "ACTIVE"

    def snapshot(self, now_s: float | None = None) -> dict[str, Any]:
        now = time.monotonic() if now_s is None else float(now_s)
        age_s = (
            None
            if self.last_received_monotonic_s is None
            else max(0.0, now - self.last_received_monotonic_s)
        )
        return {
            "messages": self.messages,
            "age_s": age_s,
            "state": self.state,
            "reason": self.reason,
            "values": self.values,
        }


def subscribe_navigation_startup(node) -> NavigationStartupEvidence:
    evidence = NavigationStartupEvidence()
    node.create_subscription(
        DiagnosticArray, "/navigation_startup/diagnostics", evidence.record, 10
    )
    return evidence


class AsyncServicePoller:
    """Rate-limited service polling with at most one request in flight."""

    def __init__(self, client, request_factory: Callable[[], Any], *,
                 interval_s: float = 0.5, response_timeout_s: float = 8.0):
        self.client = client
        self.request_factory = request_factory
        self.interval_s = interval_s
        self.response_timeout_s = response_timeout_s
        self.future = None
        self.sent_at = 0.0
        self.next_request_at = 0.0
        self.sent = 0
        self.responses = 0
        self.timeouts = 0
        self.latest = None
        self.last_error = ""

    def poll(self) -> None:
        now = time.monotonic()
        if self.future is not None:
            if self.future.done():
                try:
                    result = self.future.result()
                    if result is None:
                        self.last_error = "empty response"
                    else:
                        self.latest = result
                        self.responses += 1
                        self.last_error = ""
                except Exception as exc:  # pragma: no cover - rclpy transport detail
                    self.last_error = f"{type(exc).__name__}: {exc}"
                self.future = None
                self.next_request_at = now + self.interval_s
            elif now - self.sent_at >= self.response_timeout_s:
                self.future.cancel()
                self.future = None
                self.timeouts += 1
                self.last_error = "response timeout"
                self.next_request_at = now + self.interval_s
            return
        if now >= self.next_request_at and self.client.service_is_ready():
            self.future = self.client.call_async(self.request_factory())
            self.sent_at = now
            self.sent += 1

    def evidence(self) -> dict[str, Any]:
        return {
            "service_ready": self.client.service_is_ready(),
            "request_in_flight": self.future is not None,
            "requests_sent": self.sent,
            "responses_received": self.responses,
            "response_timeouts": self.timeouts,
            "last_error": self.last_error,
        }


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

    def wait_navigation_startup(
        self,
        evidence: NavigationStartupEvidence,
        timeout_s: float,
        *,
        name: str = "navigation startup active",
    ) -> None:
        self.wait(
            name,
            lambda: evidence.active,
            timeout_s,
            observe=evidence.snapshot,
        )

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
        self.report.evidence.setdefault("isolation", {
            "ros_domain_id": os.environ.get("ROS_DOMAIN_ID", ""),
            "gz_partition": os.environ.get("GZ_PARTITION", ""),
            "run_token": os.environ.get("SMOKE_RUN_TOKEN", ""),
            "runtime_dir": os.environ.get("SMOKE_RUNTIME_DIR", ""),
            "probe_pid": os.getpid(),
        })
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
