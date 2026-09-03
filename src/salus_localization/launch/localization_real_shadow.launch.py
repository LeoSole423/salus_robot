"""
Shadow-only local EKF for the physical robot.

The legacy ``ROS2_SALUS`` stack remains the single authority for
``/odometry/local`` and for the ``odom -> base_footprint`` transform. This
composition only estimates and publishes a comparison odometry on an isolated
topic.

The profile is structurally incapable of taking TF or command authority:

* it declares no launch argument, so nothing can be reconfigured from the
  command line;
* ``publish_tf``, ``use_control`` and ``publish_acceleration`` are pinned false
  both in the parameters file and again here as node overrides;
* the only output is remapped into the ``/salus/localization_shadow``
  namespace;
* it starts no odometry generator, global EKF, ``navsat_transform``, heading
  source, ``robot_state_publisher``, Nav2, Collision Monitor or hardware owner.

One nuance is worth recording: ``robot_localization`` always constructs its
``TransformBroadcaster``, so the node still advertises a ``/tf`` *endpoint*
even with ``publish_tf=false``. The enforced property is the payload: no
transform may ever be broadcast. The runtime test subscribes to ``/tf`` and
asserts an empty stream so this cannot regress silently.
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

SHADOW_NODE_NAME = "salus_local_ekf_shadow"
SHADOW_PARAMS_FILE = "localization_local_real_shadow.yaml"
SHADOW_ODOMETRY_TOPIC = "/salus/localization_shadow/odometry/local"
SHADOW_DIAGNOSTICS_TOPIC = "/salus/localization_shadow/diagnostics"


def generate_launch_description() -> LaunchDescription:
    """Build exactly one non-authoritative local EKF."""
    params_file = (
        Path(get_package_share_directory("salus_localization"))
        / "config"
        / SHADOW_PARAMS_FILE
    )
    return LaunchDescription([
        Node(
            package="robot_localization",
            executable="ekf_node",
            name=SHADOW_NODE_NAME,
            output="screen",
            parameters=[
                str(params_file),
                {
                    "publish_tf": False,
                    "use_control": False,
                    "publish_acceleration": False,
                    "use_sim_time": False,
                },
            ],
            remappings=[
                ("odometry/filtered", SHADOW_ODOMETRY_TOPIC),
                ("diagnostics", SHADOW_DIAGNOSTICS_TOPIC),
            ],
        ),
    ])
