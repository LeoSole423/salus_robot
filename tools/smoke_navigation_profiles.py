#!/usr/bin/env python3
"""Exercise the public profile contract and verify the committed parameters."""
from pathlib import Path
import os
import sys

import rclpy
from rcl_interfaces.srv import GetParameters
from salus_interfaces.srv import SetNavigationProfile

sys.path.insert(0, "/ros2_ws/tools")
from smoke_runtime import SmokeRuntime  # noqa: E402


def values(response) -> list[float]:
    return [item.double_value for item in response.values]


def main() -> int:
    rclpy.init()
    node = rclpy.create_node("navigation_profiles_smoke")
    target = Path(os.environ.get("SMOKE_ARTIFACT_DIR", "/ros2_ws/artifacts")) / "profile_probe.json"
    runtime = SmokeRuntime(node, "navigation-profiles", target, global_timeout_s=45.0)
    apply = node.create_client(SetNavigationProfile, "/route_executor/set_navigation_profile")
    clients = {
        "ground": node.create_client(GetParameters, "/scan_ground_filter/get_parameters"),
        "local": node.create_client(GetParameters, "/local_costmap/local_costmap/get_parameters"),
        "global": node.create_client(GetParameters, "/global_costmap/global_costmap/get_parameters"),
    }
    report = {"profiles": []}
    try:
        runtime.wait("profile services", lambda: apply.service_is_ready()
                     and all(client.service_is_ready() for client in clients.values()), 15.0)
        for profile, expected in (
            ("rural", {"ground": [0.25], "local": [0.8, 3.0], "global": [0.8, 3.0]}),
            ("urban", {"ground": [0.20], "local": [1.4, 1.3], "global": [1.5, 1.4]}),
        ):
            response = runtime.call(
                "set profile",
                apply,
                SetNavigationProfile.Request(profile=profile),
                28.0,
            )
            if not response.ok or response.active_profile != profile:
                raise RuntimeError(f"profile {profile} rejected: {response.error}")
            observed = {
                "ground": values(runtime.call("ground parameters", clients["ground"],
                    GetParameters.Request(names=["ground_tolerance_m"]), 4.0)),
                "local": values(runtime.call("local parameters", clients["local"],
                    GetParameters.Request(names=["inflation_layer.inflation_radius",
                                                 "inflation_layer.cost_scaling_factor"]), 4.0)),
                "global": values(runtime.call("global parameters", clients["global"],
                    GetParameters.Request(names=["inflation_layer.inflation_radius",
                                                 "inflation_layer.cost_scaling_factor"]), 4.0)),
            }
            if observed != expected:
                raise RuntimeError(f"profile {profile} mismatch: {observed} != {expected}")
            report["profiles"].append({"profile": profile, "observed": observed})
        report["success"] = True
        runtime.finish(True, evidence=report)
        return 0
    except Exception as exc:
        report.update(success=False, error=str(exc))
        runtime.finish(False, error=exc, evidence=report)
        raise
    finally:
        node.destroy_node(); rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
