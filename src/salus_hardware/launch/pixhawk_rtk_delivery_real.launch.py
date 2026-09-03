"""Start the isolated physical RTCM delivery profile for the Pixhawk."""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """Create only the adapter that delivers canonical RTCM to MAVROS."""

    return LaunchDescription(
        [
            Node(
                package="salus_hardware",
                executable="pixhawk_rtk_adapter",
                name="pixhawk_rtk_adapter",
                output="screen",
                parameters=[
                    {
                        "source_status_topic": (
                            "/salus/hardware/gnss_primary/rtk_source_status"
                        ),
                        "rtcm_input_topic": "/salus/hardware/rtcm/corrections",
                        "gpsraw_topic": "/mavros_node/mavros_node/gps1/raw",
                        "status_topic": "/salus/hardware/gnss_primary/rtk_status",
                        "mavros_rtcm_topic": (
                            "/mavros_node/mavros_node/send_rtcm"
                        ),
                        "delivery_backend": "pixhawk_mavros",
                        "delivery_enabled": True,
                        "stale_timeout_s": 5.0,
                        "status_period_s": 1.0,
                        "use_sim_time": False,
                    }
                ],
            )
        ]
    )
