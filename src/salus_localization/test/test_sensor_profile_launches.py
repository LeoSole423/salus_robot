from pathlib import Path

import yaml


PACKAGE = Path(__file__).parents[1]


def _local_ekf_parameters(filename: str) -> dict:
    document = yaml.safe_load(
        (PACKAGE / "config" / filename).read_text(encoding="utf-8")
    )
    return document["ekf_filter_node_local"]["ros__parameters"]


def test_local_launch_uses_hardware_identity_before_logical_imu_topic() -> None:
    contents = (PACKAGE / "launch" / "localization_sim.launch.py").read_text(
        encoding="utf-8"
    )
    assert 'choices=["imu_primary", "imu_secondary"]' in contents
    assert '"/hardware/imu_primary/data_raw"' in contents
    assert '"/hardware/imu_primary/data"' in contents
    assert 'executable="imu_selector"' in contents
    assert '"selected_source": imu_source' in contents


def test_global_launch_selects_one_heading_for_navsat_and_global_ekf() -> None:
    launch = (PACKAGE / "launch" / "global_localization_sim.launch.py").read_text(
        encoding="utf-8"
    )
    config = (PACKAGE / "config" / "localization_global_sim.yaml").read_text(
        encoding="utf-8"
    )
    assert 'choices=["course_over_ground", "external_heading"]' in launch
    assert 'executable="orientation_source_selector"' in launch
    assert '("imu/data", "/localization/orientation")' in launch
    assert "imu1: /localization/orientation" in config
    assert "odom2: /odometry/local_yaw_hold" not in config
    assert "imu0: /imu/data_global" in config


def test_external_heading_fixture_is_profile_gated_not_a_fallback() -> None:
    launch = (PACKAGE / "launch" / "global_localization_sim.launch.py").read_text(
        encoding="utf-8"
    )
    assert 'executable="sim_external_heading_from_odom"' in launch
    assert 'orientation_source, "\' == \'external_heading\'"' in launch


def test_local_sim_launch_selects_the_frozen_baseline_by_default() -> None:
    launch = (PACKAGE / "launch" / "localization_sim.launch.py").read_text(
        encoding="utf-8"
    )
    assert '"local_ekf_params_file"' in launch
    assert '"config" / "localization_local_sim.yaml"' in launch
    assert "default_value=str(default_params_file)" in launch
    assert "parameters=[" in launch
    assert "local_ekf_params_file," in launch


def test_local_sim_variants_preserve_the_imu_yaw_rate_contract() -> None:
    baseline = _local_ekf_parameters("localization_local_sim.yaml")
    for filename in (
        "localization_local_sim_wheel_twist_imu_yaw_rate.yaml",
        "localization_local_sim_wheel_pose_imu_yaw_rate.yaml",
    ):
        parameters = _local_ekf_parameters(filename)
        assert parameters["imu0"] == baseline["imu0"]
        assert parameters["imu0_config"] == baseline["imu0_config"]
        assert parameters["frequency"] == baseline["frequency"]
        assert parameters["sensor_timeout"] == baseline["sensor_timeout"]
        assert parameters["odom0"] == baseline["odom0"]


def test_local_sim_wheel_twist_variant_fuses_only_vx_and_vy() -> None:
    parameters = _local_ekf_parameters(
        "localization_local_sim_wheel_twist_imu_yaw_rate.yaml"
    )
    assert parameters["odom0_config"] == [
        False, False, False, False, False, False,
        True, True, False, False, False, False,
        False, False, False,
    ]


def test_local_sim_wheel_pose_variant_fuses_only_x_y_and_yaw() -> None:
    parameters = _local_ekf_parameters(
        "localization_local_sim_wheel_pose_imu_yaw_rate.yaml"
    )
    assert parameters["odom0_config"] == [
        True, True, False, False, False, True,
        False, False, False, False, False, False,
        False, False, False,
    ]


def test_local_real_launch_remains_bound_to_the_authoritative_real_yaml() -> None:
    launch = (PACKAGE / "launch" / "localization_local_real.launch.py").read_text(
        encoding="utf-8"
    )
    assert 'LOCAL_PARAMS_FILE = "localization_local_real.yaml"' in launch
    assert "local_ekf_params_file" not in launch
