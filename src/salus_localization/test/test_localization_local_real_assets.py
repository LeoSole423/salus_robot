"""Structural contract tests for the authoritative local real MVP (#179)."""

import ast
from importlib.util import module_from_spec, spec_from_file_location
from pathlib import Path

import yaml


PACKAGE = Path(__file__).parents[1]
CONFIG = PACKAGE / "config" / "localization_local_real.yaml"
LAUNCH = PACKAGE / "launch" / "localization_local_real.launch.py"

LOCAL_NODE = "salus_local_ekf"
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
    return document[LOCAL_NODE]["ros__parameters"]


def _executable_code(path: Path) -> str:
    """Return launch code without comments/docstrings for prohibition scans."""
    source = path.read_text(encoding="utf-8")
    lines = source.splitlines()
    tree = ast.parse(source)
    for node in ast.walk(tree):
        if isinstance(node, ast.Expr) and isinstance(node.value, ast.Constant):
            if isinstance(node.value.value, str):
                for index in range(node.lineno - 1, node.end_lineno):
                    lines[index] = ""
    return "\n".join(
        "" if line.lstrip().startswith("#") else line for line in lines
    )


def test_real_config_targets_only_the_authoritative_local_node() -> None:
    document = yaml.safe_load(CONFIG.read_text(encoding="utf-8"))
    assert list(document) == [LOCAL_NODE]


def test_real_config_preserves_the_frozen_local_profile() -> None:
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

    # The real MVP changes authority, not estimator tuning.
    forbidden_tuning = [
        key for key in parameters
        if any(token in key for token in (
            "process_noise", "euler_orientation", "linear_acceleration",
            "angular_velocity", "orientation_rejection", "velocity_rejection",
            "acceleration_rejection", "reset", "_covariance",
        ))
    ]
    assert forbidden_tuning == []


def test_real_config_owns_only_the_local_tf_and_uses_exact_inputs() -> None:
    parameters = _parameters()
    assert parameters["publish_tf"] is True
    assert parameters["use_control"] is False
    assert parameters["publish_acceleration"] is False
    assert parameters["odom0"] == "/wheel/odometry"
    assert parameters["imu0"] == "/salus/imu/data"
    assert parameters["odom0_config"] == ODOM_MASK
    assert parameters["imu0_config"] == IMU_MASK
    assert parameters["odom0_queue_size"] == 10
    assert parameters["imu0_queue_size"] == 20
    assert parameters["imu0_remove_gravitational_acceleration"] is True


def test_real_config_has_no_global_or_secondary_sensor_path() -> None:
    document = yaml.safe_load(CONFIG.read_text(encoding="utf-8"))
    parameters = _parameters()
    assert "odom1" not in parameters
    assert "imu1" not in parameters
    lowered = CONFIG.read_text(encoding="utf-8").lower()
    for forbidden in (
        "navsat", "gps", "course_heading", "external_heading", "map_frame: map",
    ):
        # map_frame is a required robot_localization parameter, but it is not
        # the selected world frame and is covered separately above.
        if forbidden == "map_frame: map":
            continue
        assert forbidden not in lowered
    assert list(document) == [LOCAL_NODE]


def test_real_launch_contains_exactly_ackermann_and_local_ekf() -> None:
    contents = LAUNCH.read_text(encoding="utf-8")
    assert contents.count("Node(") == 2
    assert contents.count('executable="ackermann_odometry"') == 1
    assert contents.count('executable="ekf_node"') == 1
    assert f'LOCAL_NODE_NAME = "{LOCAL_NODE}"' in contents
    assert 'LOCAL_PARAMS_FILE = "localization_local_real.yaml"' in contents
    assert 'LOCAL_ODOMETRY_TOPIC = "/odometry/local"' in contents
    assert 'remappings=[("odometry/filtered", LOCAL_ODOMETRY_TOPIC)]' in contents


def test_real_launch_freezes_compatible_ackermann_parameters() -> None:
    contents = LAUNCH.read_text(encoding="utf-8")
    for expected in (
        '"telemetry_topic": TELEMETRY_TOPIC',
        '"odom_topic": WHEEL_ODOMETRY_TOPIC',
        '"twist_topic": "/vehicle/twist"',
        '"odom_frame": "odom"',
        '"base_frame": "base_footprint"',
        '"wheelbase_m": 0.94',
        '"steering_limit_rad": 0.5235987756',
        '"invert_measured_steer_sign": True',
        '"max_dt_s": 0.2',
        '"require_steer_valid": False',
        '"pose_covariance_xy": 0.05',
        '"pose_covariance_yaw": 0.1',
        '"twist_covariance_vx": 0.05',
        '"twist_covariance_vy": 0.01',
        '"twist_covariance_yaw_rate": 0.1',
        '"use_sim_time": False',
    ):
        assert expected in contents


def test_real_launch_repeats_authority_overrides() -> None:
    contents = LAUNCH.read_text(encoding="utf-8")
    assert '"publish_tf": True' in contents
    assert '"use_control": False' in contents
    assert '"publish_acceleration": False' in contents
    assert '"use_sim_time": False' in contents


def test_real_launch_excludes_other_runtime_owners() -> None:
    code = _executable_code(LAUNCH).lower()
    for forbidden in (
        "global_localization", "navsat_transform", "robot_state_publisher",
        "gps_course_heading", "external_heading", "orientation_source_selector",
        "global_stationary_gates", "map_gps_absolute_measurement", "nav2",
        "collision_monitor", "controller_server", "serial", "uart", "mavros",
        "ntrip", "rslidar", "robosense", "kinematic_ackermann_odometry",
        "kinematic", "wheel_odometry.launch", "map -> odom",
    ):
        assert forbidden not in code
    assert "ackermann_odometry" in code
    assert "ekf_node" in code


def test_real_launch_is_constructible_without_hardware() -> None:
    spec = spec_from_file_location("localization_local_real", LAUNCH)
    module = module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    description = module.generate_launch_description()
    assert len(description.entities) == 2
