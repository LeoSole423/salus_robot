"""Compose the final physical sensor owners and Pixhawk input boundary."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def _include(package: str, launch_file: str, arguments: dict[str, object]):
    share = Path(get_package_share_directory(package))
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(share / "launch" / launch_file)),
        launch_arguments=arguments.items(),
    )


def generate_launch_description() -> LaunchDescription:
    """Return only the final physical owners and the logical sensor boundary."""
    fcu_url = LaunchConfiguration("fcu_url")
    ntrip_config_path = LaunchConfiguration("ntrip_config_path")
    ntrip_active_source_id = LaunchConfiguration("ntrip_active_source_id")

    return LaunchDescription([
        DeclareLaunchArgument("fcu_url", default_value="/dev/ttyACM0:921600"),
        DeclareLaunchArgument(
            "ntrip_config_path",
            description="Path to the NTRIP sources configuration; no inline credentials",
        ),
        DeclareLaunchArgument("ntrip_active_source_id", default_value=""),
        _include(
            "salus_hardware",
            "pixhawk_real.launch.py",
            {"fcu_url": fcu_url},
        ),
        _include(
            "salus_bringup",
            "pixhawk_sensor_inputs.launch.py",
            {
                "imu_expected_frame": "imu_link",
                "gnss_expected_frame": "gps_link",
            },
        ),
        _include(
            "salus_hardware",
            "ntrip_rtcm_source_real.launch.py",
            {
                "config_path": ntrip_config_path,
                "active_source_id": ntrip_active_source_id,
            },
        ),
        _include(
            "salus_hardware",
            "pixhawk_rtk_delivery_real.launch.py",
            {},
        ),
        _include("salus_hardware", "rs16_real.launch.py", {}),
    ])
