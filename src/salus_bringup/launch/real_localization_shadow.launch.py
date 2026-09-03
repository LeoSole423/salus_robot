"""
Operator entry point for the physical localization shadow trial.

It composes two already-scoped profiles and adds no new capability of its own:

1. ``real_observation.launch.py``: the read-only coexistence profile validated
   on the robot (Pixhawk IMU/GNSS adaptation, RTCM dry-run, drive telemetry and
   command shadow adapters);
2. ``localization_real_shadow.launch.py``: one local EKF that publishes only
   ``/salus/localization_shadow/odometry/local``.

``ROS2_SALUS`` therefore keeps every authority: ``/odometry/local``, the
``odom -> base_footprint`` transform, the rest of TF, hardware and control.

This wrapper deliberately declares **no** launch argument. There is no way to
enable RTCM delivery, TF publication, control, UART, Nav2 or any hardware
ownership from it, and the security properties proven for
``real_observation.launch.py`` are inherited unchanged.
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def _include(package: str, launch_file: str) -> IncludeLaunchDescription:
    share = Path(get_package_share_directory(package))
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(share / "launch" / launch_file))
    )


def generate_launch_description() -> LaunchDescription:
    """Run the validated observers plus the non-authoritative local EKF."""
    return LaunchDescription([
        _include("salus_bringup", "real_observation.launch.py"),
        _include("salus_localization", "localization_real_shadow.launch.py"),
    ])
