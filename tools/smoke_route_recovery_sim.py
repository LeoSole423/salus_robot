#!/usr/bin/env python3
"""Bounded simulated collision after a credited route checkpoint."""

import json
import math
import os
from pathlib import Path
import threading
import time

import rclpy
from nav2_msgs.msg import CollisionMonitorState
from salus_interfaces.msg import CmdVelFinal
from salus_interfaces.srv import CancelRouteMission, GetRouteMissionState

from smoke_route_executor_sim import Smoke, call, request_from_pose, route_event
from smoke_runtime import AsyncServicePoller, SmokeRuntime


def details(event):
    return {item.key: item.value for item in event.details}


def samples(messages):
    return [
        [int(item.header.stamp.sec) * 1_000_000_000
         + int(item.header.stamp.nanosec),
         float(item.pose.pose.position.x), float(item.pose.pose.position.y)]
        for item in messages
    ]


def plan_samples(messages):
    return [
        {"stamp_ns": int(item.header.stamp.sec) * 1_000_000_000
         + int(item.header.stamp.nanosec),
         "xy": [[float(pose.pose.position.x), float(pose.pose.position.y)]
                for pose in item.poses]}
        for item in messages
    ]


def backward_excursion(values):
    """Largest reversal along the ordered route direction, in metres."""
    if not values:
        return 0.0
    peak = values[0]
    largest = 0.0
    for value in values[1:]:
        largest = max(largest, peak - value)
        peak = max(peak, value)
    return largest


def main():
    if backward_excursion([0.0, 1.0, 0.0]) != 1.0:
        raise RuntimeError("backward-excursion diagnostic missed a reversal")
    if backward_excursion([0.0, 1.0, 2.0]) != 0.0:
        raise RuntimeError("backward-excursion diagnostic rejected forward progress")
    rclpy.init()
    node = Smoke()
    runtime = SmokeRuntime(
        node, "route-recovery-partial",
        Path(os.environ.get("SMOKE_ARTIFACT_DIR", ".")) / "recovery_probe.json",
        global_timeout_s=150.0,
    )
    node.runtime = runtime
    stop_fault = threading.Event()
    fault_thread = None
    success = False
    error = None
    fault_at = None
    retry_at = None
    original = None
    retry = None
    cancelled_at = None
    final_before_cancel = None
    odom_at_retry = None
    plan_count_at_retry = None
    retry_start_projection = None
    final_at_fault = None
    plan_reverse_m = None
    odom_reverse_m = None
    poller = AsyncServicePoller(
        node.state, GetRouteMissionState.Request, interval_s=0.25,
        response_timeout_s=5.0,
    )
    fault_pub = node.create_publisher(
        CollisionMonitorState, "/collision_monitor_state", 10)
    loop_case = os.environ.get("SMOKE_ROUTE_SCENARIO") == "recovery_loop"
    synthetic_case = os.environ.get("SMOKE_ROUTE_SCENARIO") == "recovery_synthetic"
    action_case = os.environ.get("SMOKE_ROUTE_SCENARIO") == "recovery_action"
    expected_original = ([5, 0, 1, 2] if loop_case
                         else [1, 2] if action_case
                         else [0, 1] if synthetic_case else [0, 1, 2])
    credit_count = int(os.environ.get("SMOKE_RECOVERY_CREDIT_COUNT", "1"))
    if credit_count < 1 or credit_count >= len(expected_original):
        raise ValueError("credit count must leave a pending checkpoint")
    expected_retry = expected_original[credit_count:]
    pending_indices = expected_retry
    try:
        runtime.wait("route stack ready", node.startup_ready, 40,
                     stimulate=node.poll_bt_state, observe=node.startup_evidence)
        runtime.wait("fresh odometry", lambda: len(node.odom) >= 2, 15)
        request = request_from_pose(node.odom[-1].pose.pose)
        if (request.loop != loop_case
                or (request.leg_spacing_m > 0.0) != synthetic_case):
            raise RuntimeError("recovery probe route shape does not match scenario")
        if not all(str(value) == "nan" for value in request.yaws_deg):
            raise RuntimeError("recovery probe requires automatic yaws")
        if action_case and (
            request.waypoint_roles[0] != "hard"
            or not request.waypoint_action_jsons[0]
        ):
            raise RuntimeError("action recovery requires a hard action boundary")
        accepted = call(node, node.set, request)
        if not accepted.ok:
            raise RuntimeError(accepted.error)

        def credited_prefix():
            nonlocal original
            if any(route_event(node, "ROUTE_CHECKPOINT_REACHED", index) is None
                   for index in expected_original[:credit_count]):
                return False
            match = next((item for item in node.dispatch_evidence()
                          if len(json.loads(item["checkpoint_occurrences"])) >= 2
                          and item["input_indices"][0] == expected_original[0]), None)
            if match is None:
                return False
            original = match
            return True

        runtime.wait("credited prefix in multipose chunk", credited_prefix, 55,
                     stimulate=poller.poll)
        if action_case and route_event(node, "ROUTE_WAYPOINT_ACTION_FINISHED", 0) is None:
            raise RuntimeError("prior hard-boundary action did not finish")
        original_occurrences = json.loads(original["checkpoint_occurrences"])
        if [item["input_index"] for item in original_occurrences] != expected_original:
            raise RuntimeError("recovery probe did not dispatch expected checkpoints")
        if synthetic_case and not original["synthetic_offsets"]:
            raise RuntimeError("synthetic recovery scenario has no synthetic poses")
        cut = original_occurrences[credit_count - 1]["offset"] + 1
        pending_indices = original["input_indices"][cut:]
        first_xy = original["poses_xy"][original_occurrences[credit_count - 1]["offset"]]
        second_xy = original["poses_xy"][original_occurrences[credit_count]["offset"]]
        dx = second_xy[0] - first_xy[0]
        dy = second_xy[1] - first_xy[1]
        direction_length = math.hypot(dx, dy)
        runtime.wait(
            "physical passage beyond credited checkpoint",
            lambda: bool(node.odom) and (
                ((node.odom[-1].pose.pose.position.x - first_xy[0]) * dx
                 + (node.odom[-1].pose.pose.position.y - first_xy[1]) * dy)
                / direction_length >= 0.5
            ),
            15, stimulate=poller.poll,
        )
        fault_at = time.monotonic()
        final_at_fault = len(node.final)

        def inject_stop():
            message = CollisionMonitorState()
            message.action_type = CollisionMonitorState.STOP
            message.polygon_name = "smoke_partial_progress"
            while not stop_fault.is_set():
                fault_pub.publish(message)
                stop_fault.wait(0.005)

        fault_thread = threading.Thread(target=inject_stop, daemon=True)
        fault_thread.start()
        cancel_in_recovery = os.environ.get("SMOKE_RECOVERY_CANCEL_IN_WAIT") == "1"
        if cancel_in_recovery:
            runtime.wait(
                "blocked route waiting for retry",
                lambda: route_event(node, "ROUTE_BLOCKED_WAITING") is not None,
                12, stimulate=poller.poll,
            )
            final_before_cancel = len(node.final)
            cancelled_at = time.monotonic()
            cancelled = call(node, node.cancel, CancelRouteMission.Request())
            if not cancelled.ok:
                raise RuntimeError(cancelled.error)
            stop_fault.set()
            fault_thread.join(timeout=2.0)
            runtime.wait("cancel during recovery", lambda: poller.latest is not None
                         and poller.latest.status == "CANCELLED", 8,
                         stimulate=poller.poll)
            runtime.wait(
                "safe zero after recovery cancel",
                lambda: any(item.source == CmdVelFinal.SOURCE_SAFETY
                            and abs(item.twist.linear.x) < 1e-6
                            and abs(item.twist.angular.z) < 1e-6
                            for item in node.final[final_before_cancel:]),
                8,
            )
            success = True
            print("Route recovery cancellation simulation smoke passed")
            return 0
        runtime.wait(
            "recovery retry after credited checkpoint",
            lambda: route_event(node, "ROUTE_BLOCKED_RETRYING") is not None,
            30, stimulate=poller.poll,
            observe=lambda: {"state": poller.evidence(),
                             "events": [event.code for event in node.events[-10:]]},
        )
        retry_at = time.monotonic()
        event = route_event(node, "ROUTE_BLOCKED_RETRYING")
        event_details = details(event)
        if json.loads(event_details["original_input_indices"]) != original["input_indices"]:
            raise RuntimeError("recovery original request changed unexpectedly")
        if json.loads(event_details["retry_input_indices"]) != pending_indices:
            raise RuntimeError("recovery did not trim credited checkpoint")
        credited = json.loads(event_details["credited_checkpoints"])
        if [item["input_index"] for item in credited] != expected_original[:credit_count]:
            raise RuntimeError("retry credit evidence is incomplete")
        stop_fault.set()
        fault_thread.join(timeout=2.0)
        release = CollisionMonitorState()
        release.action_type = CollisionMonitorState.DO_NOTHING
        for _ in range(50):
            fault_pub.publish(release)
            runtime.spin(timeout_s=0.01)
        runtime.wait(
            "injected collision stop released",
            lambda: bool(node.telemetry)
            and not node.telemetry[-1].collision_stop_active,
            5,
        )
        odom_at_retry = len(node.odom)
        plan_count_at_retry = len(node.plans)

        def retry_dispatched():
            nonlocal retry
            retry = next((item for item in node.dispatch_evidence()
                          if item["chunk_id"] == original["chunk_id"]
                          and item["event_id"] > original["event_id"]), None)
            return retry is not None

        runtime.wait("pending suffix dispatched", retry_dispatched, 12,
                     stimulate=poller.poll)
        if retry["input_indices"] != pending_indices:
            raise RuntimeError("retry dispatched a checkpoint already credited")
        if loop_case:
            original_occurrences = json.loads(
                original["checkpoint_occurrences"])
            retry_occurrences = json.loads(retry["checkpoint_occurrences"])
            if ([item["loop_iteration"] for item in original_occurrences]
                    != [0, 1, 1, 1]
                    or [item["loop_iteration"] for item in retry_occurrences]
                    != [1] * len(expected_retry)):
                raise RuntimeError("retry lost loop iteration at closure")
        credits = [details(item) for item in node.events
                   if item.code == "ROUTE_CHECKPOINT_REACHED"]
        identities = [(item["loop_iteration"], item["input_index"])
                      for item in credits]
        if len(identities) != len(set(identities)):
            raise RuntimeError("checkpoint was credited twice")
        if action_case:
            started = [item for item in node.events
                       if item.code == "ROUTE_WAYPOINT_ACTION_STARTED"
                       and details(item).get("waypoint_index") == "0"]
            finished = [item for item in node.events
                        if item.code == "ROUTE_WAYPOINT_ACTION_FINISHED"
                        and details(item).get("waypoint_index") == "0"]
            if len(started) != 1 or len(finished) != 1:
                raise RuntimeError("hard-boundary action was repeated")

        credited_xy = first_xy
        next_xy = second_xy
        vx = next_xy[0] - credited_xy[0]
        vy = next_xy[1] - credited_xy[1]
        norm = math.hypot(vx, vy)
        if norm < 0.5:
            raise RuntimeError("retry path has no measurable forward direction")
        start_pose = node.odom[odom_at_retry - 1].pose.pose.position
        retry_start_projection = (
            (start_pose.x - credited_xy[0]) * vx
            + (start_pose.y - credited_xy[1]) * vy
        ) / norm

        def forward_progress():
            for item in node.odom[odom_at_retry:]:
                x = item.pose.pose.position.x - credited_xy[0]
                y = item.pose.pose.position.y - credited_xy[1]
                if (x * vx + y * vy) / norm >= retry_start_projection + 1.0:
                    return True
            return False

        runtime.wait("physical forward progress after retry", forward_progress,
                     20, stimulate=poller.poll)
        if len(node.plans) <= plan_count_at_retry:
            raise RuntimeError("Nav2 did not publish a plan after retry")
        plan_reverse_m = 0.0
        for plan in node.plans[plan_count_at_retry:]:
            along = [
                ((pose.pose.position.x - credited_xy[0]) * vx
                 + (pose.pose.position.y - credited_xy[1]) * vy) / norm
                for pose in plan.poses
            ]
            if along and min(along) < -0.5:
                raise RuntimeError("retry plan returns behind credited checkpoint")
            plan_reverse_m = max(plan_reverse_m, backward_excursion(along))
        if plan_reverse_m > 0.5:
            raise RuntimeError("retry plan contains a sustained reverse segment")
        projections = [
            ((item.pose.pose.position.x - credited_xy[0]) * vx
             + (item.pose.pose.position.y - credited_xy[1]) * vy) / norm
            for item in node.odom[odom_at_retry:]
        ]
        if min(projections) < -0.5:
            raise RuntimeError("robot returned behind credited checkpoint")
        odom_reverse_m = backward_excursion(projections)
        if odom_reverse_m > 0.5:
            raise RuntimeError("robot executed a sustained reversal after retry")

        final_before_cancel = len(node.final)
        cancelled_at = time.monotonic()
        cancelled = call(node, node.cancel, CancelRouteMission.Request())
        if not cancelled.ok:
            raise RuntimeError(cancelled.error)
        runtime.wait("route cancelled", lambda: poller.latest is not None
                     and poller.latest.status == "CANCELLED", 8,
                     stimulate=poller.poll)
        runtime.wait(
            "safe zero after cancel",
            lambda: any(item.source == CmdVelFinal.SOURCE_SAFETY
                        and abs(item.twist.linear.x) < 1e-6
                        and abs(item.twist.angular.z) < 1e-6
                        for item in node.final[final_before_cancel:]),
            8,
        )
        success = True
        print("Route recovery partial-progress simulation smoke passed")
        return 0
    except Exception as exc:
        error = exc
        raise
    finally:
        stop_fault.set()
        if fault_thread is not None:
            fault_thread.join(timeout=2.0)
        runtime.finish(success, error=error, evidence={
            "original_dispatch": original,
            "retry_dispatch": retry,
            "dispatches": node.dispatch_evidence(),
            "retry_event": (None if route_event(node, "ROUTE_BLOCKED_RETRYING") is None
                            else details(route_event(node, "ROUTE_BLOCKED_RETRYING"))),
            "checkpoint_events": [details(item) for item in node.events
                                  if item.code == "ROUTE_CHECKPOINT_REACHED"],
            "fault_started_steady_s": fault_at,
            "final_count_at_fault": final_at_fault,
            "retry_started_steady_s": retry_at,
            "cancel_started_steady_s": cancelled_at,
            "final_count_at_cancel": final_before_cancel,
            "odom_count_at_retry": odom_at_retry,
            "plan_count_at_retry": plan_count_at_retry,
            "retry_start_projection_m": retry_start_projection,
            "plan_backward_excursion_m": plan_reverse_m,
            "odometry_backward_excursion_m": odom_reverse_m,
            "plans": plan_samples(node.plans),
            "odometry": samples(node.odom),
            "final_commands": [
                {"source": int(item.source),
                 "linear_x": float(item.twist.linear.x),
                 "angular_z": float(item.twist.angular.z),
                 "brake_pct": int(item.brake_pct)}
                for item in node.final
            ],
            "state": poller.evidence(),
            "last_status": getattr(poller.latest, "status", "unavailable"),
            "last_nav_result": (node.telemetry[-1].nav_result_text
                                if node.telemetry else "unavailable"),
            "recovery_events": [item.code for item in node.events
                                if item.code.startswith("ROUTE_BLOCKED_")],
        })
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
