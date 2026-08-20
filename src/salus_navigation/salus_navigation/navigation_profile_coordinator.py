"""Transactional ROS adapter for the urban/rural navigation profiles."""
from __future__ import annotations

import threading
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from nav2_msgs.srv import ClearEntireCostmap
from salus_interfaces.srv import SetNavigationProfile

from .navigation_profiles import NavigationProfileTransaction, PROFILES, TransactionState


def double_parameter(name: str, value: float) -> Parameter:
    return Parameter(name=name, value=ParameterValue(
        type=ParameterType.PARAMETER_DOUBLE, double_value=float(value)))


class NavigationProfileCoordinator(Node):
    """Apply one profile to every runtime component or restore all of them."""

    def __init__(self) -> None:
        super().__init__("navigation_profile_coordinator")
        self.declare_parameter("request_timeout_s", 3.0)
        group = ReentrantCallbackGroup()
        self._parameter_clients = {
            "ground_filter": self.create_client(
                SetParameters, "/scan_ground_filter/set_parameters", callback_group=group),
            "local_inflation": self.create_client(
                SetParameters, "/local_costmap/local_costmap/set_parameters", callback_group=group),
            "global_inflation": self.create_client(
                SetParameters, "/global_costmap/global_costmap/set_parameters", callback_group=group),
            "controller": self.create_client(
                SetParameters, "/controller_server/set_parameters", callback_group=group),
        }
        self._clear_clients = (
            self.create_client(ClearEntireCostmap, "/local_costmap/clear_entirely_local_costmap", callback_group=group),
            self.create_client(ClearEntireCostmap, "/global_costmap/clear_entirely_global_costmap", callback_group=group),
        )
        self._transaction = NavigationProfileTransaction("urban")
        self.create_service(SetNavigationProfile, "/navigation_profile_coordinator/apply",
                            self._apply, callback_group=group)

    def _parameters_for(self, component: str, profile: str) -> list[Parameter]:
        value = PROFILES[profile]
        if component == "ground_filter":
            return [double_parameter("ground_tolerance_m", value.ground_tolerance_m)]
        if component == "local_inflation":
            return [double_parameter("inflation_layer.inflation_radius", value.local_inflation_radius),
                    double_parameter("inflation_layer.cost_scaling_factor", value.local_cost_scaling)]
        if component == "global_inflation":
            return [double_parameter("inflation_layer.inflation_radius", value.global_inflation_radius),
                    double_parameter("inflation_layer.cost_scaling_factor", value.global_cost_scaling)]
        return [double_parameter("FollowPath.desired_linear_vel", value.desired_linear_vel)]

    def _set_component(self, component: str, profile: str) -> tuple[bool, str]:
        client = self._parameter_clients[component]
        timeout = float(self.get_parameter("request_timeout_s").value)
        if not client.wait_for_service(timeout_sec=timeout):
            return False, f"{component} parameter service unavailable"
        try:
            future = client.call_async(SetParameters.Request(
                parameters=self._parameters_for(component, profile)))
            ready = threading.Event()
            future.add_done_callback(lambda _done: ready.set())
            if not ready.wait(timeout):
                return False, f"{component} update timed out"
            response = future.result()
        except Exception as exc:
            return False, f"{component} update failed: {exc}"
        failures = [result.reason or "rejected" for result in response.results if not result.successful]
        return (not failures), "; ".join(failures)

    def _apply(self, request, response):
        target = str(request.profile).strip().lower()
        error = self._transaction.begin(target)
        if error:
            response.ok, response.error = False, error
            response.active_profile = self._transaction.active_profile
            return response
        for component in tuple(self._transaction.pending):
            ok, reason = self._set_component(component, target)
            self._transaction.confirm(component, ok, reason)
            if not ok:
                break
        if self._transaction.state == TransactionState.ROLLING_BACK:
            for component in tuple(self._transaction.pending):
                ok, reason = self._set_component(component, self._transaction.previous_profile)
                self._transaction.confirm_rollback(component, ok, reason)
        response.ok = self._transaction.state == TransactionState.SUCCEEDED
        response.error = "" if response.ok else (
            f"{self._transaction.failed_component}: {self._transaction.error}")
        response.active_profile = self._transaction.active_profile
        if response.ok:
            for client in self._clear_clients:
                if client.service_is_ready():
                    client.call_async(ClearEntireCostmap.Request())
        return response


def main(args=None) -> None:
    rclpy.init(args=args)
    node = NavigationProfileCoordinator()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.shutdown(); node.destroy_node(); rclpy.shutdown()
