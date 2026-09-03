"""
Authoritative compatible local localization profile for physical inputs.

This deliberately small composition owns only the measured Ackermann wheel
odometry adapter and the local ``robot_localization`` EKF.  The EKF is the
single publisher of ``/odometry/local`` and ``odom -> base_footprint``.
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


LOCAL_NODE_NAME = "salus_local_ekf"
LOCAL_PARAMS_FILE = "localization_local_real.yaml"
TELEMETRY_TOPIC = "/controller/drive_telemetry"
WHEEL_ODOMETRY_TOPIC = "/wheel/odometry"
IMU_TOPIC = "/salus/imu/data"
LOCAL_ODOMETRY_TOPIC = "/odometry/local"


def generate_launch_description() -> LaunchDescription:
    """Build the two-node local real MVP without hardware owners."""
    params_file = (
        Path(get_package_share_directory("salus_localization"))
        / "config"
        / LOCAL_PARAMS_FILE
    )

    return LaunchDescription([
        Node(
            package="salus_localization",
            executable="ackermann_odometry",
            name="ackermann_odometry",
            output="screen",
            parameters=[
                {
                    "telemetry_topic": TELEMETRY_TOPIC,
                    "odom_topic": WHEEL_ODOMETRY_TOPIC,
                    "twist_topic": "/vehicle/twist",
                    "odom_frame": "odom",
                    "base_frame": "base_footprint",
                    "wheelbase_m": 0.94,
                    "steering_limit_rad": 0.5235987756,
                    "invert_measured_steer_sign": True,
                    "max_dt_s": 0.2,
                    "require_steer_valid": False,
                    "pose_covariance_xy": 0.05,
                    "pose_covariance_yaw": 0.1,
                    "twist_covariance_vx": 0.05,
                    "twist_covariance_vy": 0.01,
                    "twist_covariance_yaw_rate": 0.1,
                    "use_sim_time": False,
                }
            ],
        ),
        Node(
            package="robot_localization",
            executable="ekf_node",
            name=LOCAL_NODE_NAME,
            output="screen",
            parameters=[
                str(params_file),
                {
                    "publish_tf": True,
                    "use_control": False,
                    "publish_acceleration": False,
                    "use_sim_time": False,
                },
            ],
            remappings=[("odometry/filtered", LOCAL_ODOMETRY_TOPIC)],
        ),
    ])
