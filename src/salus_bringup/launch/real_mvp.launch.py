"""Compose the final production real MVP from existing package owners."""

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
    """Compose the final physical stack without adding an orchestrator."""
    fcu_url = LaunchConfiguration("fcu_url")
    ntrip_config_path = LaunchConfiguration("ntrip_config_path")
    ntrip_active_source_id = LaunchConfiguration("ntrip_active_source_id")
    serial_port = LaunchConfiguration("serial_port")
    use_keepout = LaunchConfiguration("use_keepout")
    zones_runtime_dir = LaunchConfiguration("zones_runtime_dir")
    patrol_runtime_dir = LaunchConfiguration("patrol_runtime_dir")
    patrol_battery_guard_topic = LaunchConfiguration("patrol_battery_guard_topic")
    patrol_battery_state_topic = LaunchConfiguration("patrol_battery_state_topic")
    web_gps_fix_topic = LaunchConfiguration("web_gps_fix_topic")
    camera_host = LaunchConfiguration("camera_host")
    camera_port = LaunchConfiguration("camera_port")
    camera_channel = LaunchConfiguration("camera_channel")
    camera_presets_file = LaunchConfiguration("camera_presets_file")

    return LaunchDescription([
        DeclareLaunchArgument("fcu_url", default_value="/dev/ttyACM0:921600"),
        DeclareLaunchArgument(
            "ntrip_config_path",
            description="Path to the NTRIP sources configuration; no inline credentials",
        ),
        DeclareLaunchArgument("ntrip_active_source_id", default_value=""),
        DeclareLaunchArgument("serial_port", default_value="auto"),
        DeclareLaunchArgument("use_keepout", default_value="true"),
        DeclareLaunchArgument("zones_runtime_dir", default_value="runtime/zones"),
        DeclareLaunchArgument("patrol_runtime_dir", default_value="runtime/patrol"),
        DeclareLaunchArgument(
            "patrol_battery_guard_topic", default_value="/battery_mission_guard"),
        DeclareLaunchArgument(
            "patrol_battery_state_topic", default_value="/battery_state"),
        DeclareLaunchArgument(
            "web_gps_fix_topic", default_value="/salus/gps/fix"),
        DeclareLaunchArgument("camera_host", default_value=""),
        DeclareLaunchArgument("camera_port", default_value="0"),
        DeclareLaunchArgument("camera_channel", default_value="0"),
        DeclareLaunchArgument(
            "camera_presets_file",
            default_value="/ros2_ws/log/runtime/camera/presets.json",
        ),
        _include("salus_description", "description_real.launch.py", {}),
        _include(
            "salus_bringup",
            "real_hardware.launch.py",
            {
                "fcu_url": fcu_url,
                "ntrip_config_path": ntrip_config_path,
                "ntrip_active_source_id": ntrip_active_source_id,
            },
        ),
        _include(
            "salus_control",
            "control_real_uart.launch.py",
            {"serial_port": serial_port},
        ),
        _include(
            "salus_localization",
            "localization_local_real.launch.py",
            {},
        ),
        _include(
            "salus_localization",
            "global_localization_real.launch.py",
            {},
        ),
        _include("salus_perception", "perception_real.launch.py", {}),
        _include(
            "salus_hardware",
            "camera_real.launch.py",
            {
                "camera_host": camera_host,
                "camera_port": camera_port,
                "camera_channel": camera_channel,
                "camera_presets_file": camera_presets_file,
            },
        ),
        _include(
            "salus_navigation",
            "navigation_real.launch.py",
            {
                "use_keepout": use_keepout,
                "zones_runtime_dir": zones_runtime_dir,
                "patrol_runtime_dir": patrol_runtime_dir,
                "patrol_battery_guard_topic": patrol_battery_guard_topic,
                "patrol_battery_state_topic": patrol_battery_state_topic,
            },
        ),
        _include(
            "salus_web",
            "web_bridge.launch.py",
            {"gps_fix_topic": web_gps_fix_topic},
        ),
    ])
