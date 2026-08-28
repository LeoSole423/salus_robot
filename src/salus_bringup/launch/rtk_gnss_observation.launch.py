"""Read-only RTK/GNSS observation bridge for coexistence with the legacy stack."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def observer_parameters(
    *,
    legacy_status_topic,
    legacy_fix_topic,
    legacy_rtcm_topic,
    canonical_status_topic,
    canonical_rtcm_topic,
    delivery_backend,
    legacy_rtcm_type,
    stale_timeout_s,
):
    """Return the complete read-only legacy observer parameter boundary."""
    return {
        "legacy_status_topic": legacy_status_topic,
        "legacy_fix_topic": legacy_fix_topic,
        "legacy_rtcm_topic": legacy_rtcm_topic,
        "canonical_status_topic": canonical_status_topic,
        "canonical_rtcm_topic": canonical_rtcm_topic,
        "delivery_backend": delivery_backend,
        "legacy_rtcm_type": legacy_rtcm_type,
        "stale_timeout_s": stale_timeout_s,
    }


def dry_run_parameters(*, canonical_rtcm_topic):
    """Return only parameters owned by the non-delivering canonical sink."""
    return {
        "input_topic": canonical_rtcm_topic,
        "status_topic": "/salus/hardware/rtcm/dry_run_status",
    }


def generate_launch_description():
    enabled = LaunchConfiguration("enabled")
    legacy_status_topic = LaunchConfiguration("legacy_status_topic")
    legacy_fix_topic = LaunchConfiguration("legacy_fix_topic")
    legacy_rtcm_topic = LaunchConfiguration("legacy_rtcm_topic")
    canonical_status_topic = LaunchConfiguration("canonical_status_topic")
    canonical_rtcm_topic = LaunchConfiguration("canonical_rtcm_topic")
    delivery_backend = LaunchConfiguration("delivery_backend")
    legacy_rtcm_type = LaunchConfiguration("legacy_rtcm_type")
    stale_timeout_s = LaunchConfiguration("stale_timeout_s")

    legacy_observer_parameters = observer_parameters(
        legacy_status_topic=legacy_status_topic,
        legacy_fix_topic=legacy_fix_topic,
        legacy_rtcm_topic=legacy_rtcm_topic,
        canonical_status_topic=canonical_status_topic,
        canonical_rtcm_topic=canonical_rtcm_topic,
        delivery_backend=delivery_backend,
        legacy_rtcm_type=legacy_rtcm_type,
        stale_timeout_s=stale_timeout_s,
    )

    return LaunchDescription([
        DeclareLaunchArgument("enabled", default_value="true"),
        DeclareLaunchArgument(
            "legacy_status_topic", default_value="/gps/rtk_source/status_json"
        ),
        DeclareLaunchArgument(
            "legacy_fix_topic", default_value="/gps/rtk_status_mavros"
        ),
        DeclareLaunchArgument("legacy_rtcm_topic", default_value="/rtcm"),
        DeclareLaunchArgument(
            "canonical_status_topic",
            default_value="/salus/hardware/gnss_primary/rtk_status",
        ),
        DeclareLaunchArgument(
            "canonical_rtcm_topic",
            default_value="/salus/hardware/rtcm/corrections",
        ),
        DeclareLaunchArgument("delivery_backend", default_value="disabled"),
        DeclareLaunchArgument(
            "legacy_rtcm_type", default_value="uint8_multi_array"
        ),
        DeclareLaunchArgument("stale_timeout_s", default_value="5.0"),
        Node(
            package="salus_hardware",
            executable="legacy_rtk_observer",
            name="legacy_rtk_observer",
            namespace="/salus/hardware",
            output="screen",
            condition=IfCondition(enabled),
            parameters=[legacy_observer_parameters],
        ),
        Node(
            package="salus_hardware",
            executable="rtcm_dry_run_sink",
            name="rtcm_dry_run_sink",
            namespace="/salus/hardware",
            output="screen",
            condition=IfCondition(enabled),
            parameters=[dry_run_parameters(
                canonical_rtcm_topic=canonical_rtcm_topic,
            )],
        ),
    ])
