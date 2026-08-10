"""Non-operational placeholder for the future simulated SALUS bringup."""

from launch import LaunchDescription
from launch.actions import LogInfo


def generate_launch_description():
    return LaunchDescription([
        LogInfo(msg="WARNING: SALUS simulation skeleton; no simulator or nodes are started."),
    ])
