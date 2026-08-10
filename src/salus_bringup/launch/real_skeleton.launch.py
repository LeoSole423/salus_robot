"""Non-operational placeholder for the future real SALUS bringup."""

from launch import LaunchDescription
from launch.actions import LogInfo


def generate_launch_description():
    return LaunchDescription([
        LogInfo(msg="WARNING: SALUS real skeleton; no nodes or hardware are started."),
    ])
