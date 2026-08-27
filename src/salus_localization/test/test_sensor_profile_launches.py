from pathlib import Path


PACKAGE = Path(__file__).parents[1]


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
