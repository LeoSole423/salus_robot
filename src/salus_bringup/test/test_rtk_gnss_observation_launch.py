from pathlib import Path


LAUNCH = (
    Path(__file__).parents[1] / "launch" / "rtk_gnss_observation.launch.py"
)


def test_rtk_gnss_observation_launch_wires_explicit_read_only_contract() -> None:
    contents = LAUNCH.read_text(encoding="utf-8")

    expected_arguments = {
        "enabled": "true",
        "legacy_status_topic": "/gps/rtk_source/status_json",
        "legacy_fix_topic": "/gps/rtk_status_mavros",
        "legacy_rtcm_topic": "/rtcm",
        "canonical_status_topic": "/salus/hardware/gnss_primary/rtk_status",
        "canonical_rtcm_topic": "/salus/hardware/rtcm/corrections",
        "delivery_backend": "disabled",
        "legacy_rtcm_type": "uint8_multi_array",
        "stale_timeout_s": "5.0",
    }
    for name, default in expected_arguments.items():
        assert f'"{name}"' in contents
        assert f'default_value="{default}"' in contents

    assert contents.count('package="salus_hardware"') == 2
    assert 'executable="legacy_rtk_observer"' in contents
    assert 'executable="rtcm_dry_run_sink"' in contents
    assert contents.count('namespace="/salus/hardware"') == 2
    assert contents.count("condition=IfCondition(enabled)") == 2


def test_rtk_gnss_observation_launch_excludes_control_and_transport_outputs() -> None:
    contents = LAUNCH.read_text(encoding="utf-8").lower()

    for forbidden in (
        "send_rtcm",
        "ntrip",
        "cmd_vel",
        "uart",
        "robot_state_publisher",
    ):
        assert forbidden not in contents
