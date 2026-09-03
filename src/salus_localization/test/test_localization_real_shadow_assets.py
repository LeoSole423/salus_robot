"""Structural safety tests for the physical shadow local EKF (#161).

These guard the boundary decided in the design notes: the Salus local EKF may
estimate, but ``ROS2_SALUS`` keeps every authority. A test must fail if anyone
re-enables TF/control, adds a second estimator, points the profile at the
legacy output topic or pulls in a global/hardware owner.
"""

from importlib.util import module_from_spec, spec_from_file_location
from pathlib import Path

import ast
import yaml


CONFIG = (
    Path(__file__).parents[1]
    / "config"
    / "localization_local_real_shadow.yaml"
)
LAUNCH = (
    Path(__file__).parents[1]
    / "launch"
    / "localization_real_shadow.launch.py"
)


def _code_text(path: Path) -> str:
    """Source without the module docstring or comments.

    The profile deliberately *documents* what it excludes in prose, so the
    prohibition scan must look at executable composition only.
    """
    source = path.read_text(encoding="utf-8")
    lines = source.splitlines()
    tree = ast.parse(source)
    if tree.body:
        first = tree.body[0]
        if (
            isinstance(first, ast.Expr)
            and isinstance(first.value, ast.Constant)
            and isinstance(first.value.value, str)
        ):
            for index in range(first.lineno - 1, first.end_lineno):
                lines[index] = ""
    return "\n".join(
        "" if line.lstrip().startswith("#") else line for line in lines
    )


SHADOW_NODE = "salus_local_ekf_shadow"
SHADOW_ODOMETRY_TOPIC = "/salus/localization_shadow/odometry/local"

# Kept as literals so reordering the config cannot silently pass.
ODOM_MASK = [
    True, True, False, False, False, True,
    True, True, False, False, False, True,
    False, False, False,
]
IMU_MASK = [
    False, False, False, False, False, False,
    False, False, False, False, False, True,
    False, False, False,
]


def _parameters() -> dict:
    document = yaml.safe_load(CONFIG.read_text(encoding="utf-8"))
    return document[SHADOW_NODE]["ros__parameters"]


def test_shadow_config_targets_only_the_shadow_node() -> None:
    document = yaml.safe_load(CONFIG.read_text(encoding="utf-8"))
    assert list(document) == [SHADOW_NODE]


def test_shadow_config_keeps_the_local_model_without_new_tuning() -> None:
    parameters = _parameters()
    assert parameters["frequency"] == 30.0
    assert parameters["sensor_timeout"] == 0.2
    assert parameters["two_d_mode"] is True
    assert parameters["transform_time_offset"] == 0.0
    assert parameters["transform_timeout"] == 0.0
    assert parameters["print_diagnostics"] is True
    assert parameters["map_frame"] == "map"
    assert parameters["odom_frame"] == "odom"
    assert parameters["base_link_frame"] == "base_footprint"
    assert parameters["world_frame"] == "odom"
    # No process-noise, covariance or rejection tuning was invented here.
    forbidden_tuning = [
        key for key in parameters
        if any(token in key for token in (
            "process_noise", "euler_orientation", "linear_acceleration",
            "angular_velocity", "orientation_rejection", "velocity_rejection",
            "acceleration_rejection", "reset", "_covariance",
        ))
    ]
    assert forbidden_tuning == []


def test_shadow_config_pins_every_authority_flag_false() -> None:
    parameters = _parameters()
    assert parameters["publish_tf"] is False
    assert parameters["use_control"] is False
    assert parameters["publish_acceleration"] is False


def test_shadow_config_uses_legacy_wheel_odometry_and_logical_salus_imu() -> None:
    parameters = _parameters()
    assert parameters["odom0"] == "/wheel/odometry"
    assert parameters["imu0"] == "/salus/imu/data"
    assert parameters["odom0_config"] == ODOM_MASK
    assert parameters["imu0_config"] == IMU_MASK
    assert parameters["odom0_queue_size"] == 10
    assert parameters["imu0_queue_size"] == 20
    assert parameters["imu0_remove_gravitational_acceleration"] is True


def test_shadow_config_is_local_only() -> None:
    text = CONFIG.read_text(encoding="utf-8")
    lowered = text.lower()
    for forbidden in (
        "/odometry/local",
        "navsat",
        "gps",
        "odom1",
        "imu1",
        "world_frame: map",
        "course_heading",
        "external_heading",
    ):
        assert forbidden not in lowered


def test_shadow_launch_starts_exactly_one_non_authoritative_ekf() -> None:
    contents = LAUNCH.read_text(encoding="utf-8")
    assert contents.count("executable=\"ekf_node\"") == 1
    assert contents.count("Node(") == 1
    assert f"SHADOW_NODE_NAME = \"{SHADOW_NODE}\"" in contents
    assert f'SHADOW_ODOMETRY_TOPIC = "{SHADOW_ODOMETRY_TOPIC}"' in contents
    assert "localization_local_real_shadow.yaml" in contents
    # The node argument must not be the simulated, TF-owning identity.
    assert "ekf_filter_node_local" not in contents


def test_shadow_launch_repeats_the_authority_overrides() -> None:
    contents = LAUNCH.read_text(encoding="utf-8")
    assert '"publish_tf": False' in contents
    assert '"use_control": False' in contents
    assert '"publish_acceleration": False' in contents
    assert '"use_sim_time": False' in contents


def test_shadow_launch_exposes_no_way_to_reconfigure_authority() -> None:
    contents = LAUNCH.read_text(encoding="utf-8")
    assert "DeclareLaunchArgument" not in contents
    assert 'default_value="true"' not in contents


def test_shadow_launch_redirects_every_output_into_the_shadow_namespace() -> None:
    contents = LAUNCH.read_text(encoding="utf-8")
    assert '("odometry/filtered", SHADOW_ODOMETRY_TOPIC)' in contents
    assert '("diagnostics", SHADOW_DIAGNOSTICS_TOPIC)' in contents
    assert 'SHADOW_DIAGNOSTICS_TOPIC = "/salus/localization_shadow/diagnostics"' in contents


def test_shadow_launch_excludes_every_other_authority_and_backend() -> None:
    code = _code_text(LAUNCH).lower()
    for forbidden in (
        "navsat_transform",
        "robot_state_publisher",
        "gps_course_heading",
        "external_heading",
        "orientation_source_selector",
        "global_stationary_gates",
        "map_gps_absolute_measurement",
        "ekf_filter_node_global",
        "nav2",
        "collision_monitor",
        "controller_server",
        "serial",
        "uart",
        "mavros",
        "ntrip",
        "rslidar",
        "robosense",
        "ackermann_odometry",
        "kinematic",
        "wheel_odometry.launch",
        "/tf",
    ):
        assert forbidden not in code
    assert "ekf_node" in code  # the only started runtime is the shadow EKF


def test_shadow_launch_is_constructible_without_hardware() -> None:
    spec = spec_from_file_location("localization_real_shadow", LAUNCH)
    module = module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    description = module.generate_launch_description()
    assert len(description.entities) == 1
