"""RTK/GNSS observation and explicitly guarded Pixhawk delivery profile."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def observer_parameters(**values):
    """Return parameters owned by the legacy source observer."""
    return values


def dry_run_parameters(*, canonical_rtcm_topic):
    return {
        "input_topic": canonical_rtcm_topic,
        "status_topic": "/salus/hardware/rtcm/dry_run_status",
    }


def _as_bool(value: str, *, name: str) -> bool:
    normalized = value.strip().lower()
    if normalized not in ("true", "false"):
        raise ValueError(f"{name} must be true or false")
    return normalized == "true"


def profile_status_topics(*, profile: str, canonical_status_topic: str):
    """Return observer and optional adapter outputs for one status authority."""
    if profile == "pixhawk_mavros":
        return (
            "/salus/hardware/gnss_primary/rtk_source_status",
            canonical_status_topic,
        )
    return canonical_status_topic, None


def build_profile_nodes(*, profile: str, delivery_enabled: bool, values: dict):
    """Build a profile with exactly one canonical RTK status authority."""
    if profile not in ("disabled", "pixhawk_mavros", "direct_usb"):
        raise ValueError(
            "delivery_backend must be disabled, pixhawk_mavros or direct_usb"
        )
    if profile == "direct_usb":
        raise ValueError("direct_usb delivery backend is not implemented")
    if delivery_enabled and profile != "pixhawk_mavros":
        raise ValueError("delivery_enabled requires delivery_backend=pixhawk_mavros")

    source_status_topic, adapter_status_topic = profile_status_topics(
        profile=profile,
        canonical_status_topic=values["canonical_status_topic"],
    )
    stale_timeout = float(values["stale_timeout_s"])
    if stale_timeout <= 0.0:
        raise ValueError("stale_timeout_s must be positive")
    nodes = [
        Node(
            package="salus_hardware",
            executable="legacy_rtk_observer",
            name="legacy_rtk_observer",
            namespace="/salus/hardware",
            output="screen",
            parameters=[observer_parameters(
                legacy_status_topic=values["legacy_status_topic"],
                legacy_fix_topic=values["legacy_fix_topic"],
                legacy_rtcm_topic=values["legacy_rtcm_topic"],
                canonical_status_topic=source_status_topic,
                canonical_rtcm_topic=values["canonical_rtcm_topic"],
                delivery_backend="disabled",
                legacy_rtcm_type=values["legacy_rtcm_type"],
                stale_timeout_s=stale_timeout,
            )],
        ),
        Node(
            package="salus_hardware",
            executable="rtcm_dry_run_sink",
            name="rtcm_dry_run_sink",
            namespace="/salus/hardware",
            output="screen",
            parameters=[dry_run_parameters(
                canonical_rtcm_topic=values["canonical_rtcm_topic"]
            )],
        ),
    ]
    if profile == "pixhawk_mavros":
        nodes.append(Node(
            package="salus_hardware",
            executable="pixhawk_rtk_adapter",
            name="pixhawk_rtk_adapter",
            namespace="/salus/hardware",
            output="screen",
            parameters=[{
                "source_status_topic": source_status_topic,
                "rtcm_input_topic": values["canonical_rtcm_topic"],
                "gpsraw_topic": values["gpsraw_topic"],
                "status_topic": adapter_status_topic,
                "mavros_rtcm_topic": values["mavros_rtcm_topic"],
                "delivery_backend": profile,
                "delivery_enabled": delivery_enabled,
                "stale_timeout_s": stale_timeout,
            }],
        ))
    return nodes


def _launch_setup(context):
    launch_enabled = _as_bool(
        LaunchConfiguration("enabled").perform(context), name="enabled"
    )
    if not launch_enabled:
        return []
    names = (
        "legacy_status_topic", "legacy_fix_topic", "legacy_rtcm_topic",
        "canonical_status_topic", "canonical_rtcm_topic", "legacy_rtcm_type",
        "stale_timeout_s", "gpsraw_topic", "mavros_rtcm_topic",
    )
    values = {name: LaunchConfiguration(name).perform(context) for name in names}
    profile = (
        LaunchConfiguration("delivery_backend").perform(context).strip().lower()
    )
    enabled = _as_bool(
        LaunchConfiguration("delivery_enabled").perform(context),
        name="delivery_enabled",
    )
    return build_profile_nodes(profile=profile, delivery_enabled=enabled, values=values)


def generate_launch_description():
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
        DeclareLaunchArgument(
            "gpsraw_topic", default_value="/mavros_node/gps1/raw"
        ),
        DeclareLaunchArgument(
            "mavros_rtcm_topic", default_value="/mavros_node/send_rtcm"
        ),
        DeclareLaunchArgument("delivery_backend", default_value="disabled"),
        DeclareLaunchArgument("delivery_enabled", default_value="false"),
        DeclareLaunchArgument("legacy_rtcm_type", default_value="uint8_multi_array"),
        DeclareLaunchArgument("stale_timeout_s", default_value="5.0"),
        OpaqueFunction(function=_launch_setup),
    ])
