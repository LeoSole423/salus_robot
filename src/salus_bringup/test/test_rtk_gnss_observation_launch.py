from pathlib import Path
import importlib.util


LAUNCH = (
    Path(__file__).parents[1] / "launch" / "rtk_gnss_observation.launch.py"
)


def _launch_module():
    spec = importlib.util.spec_from_file_location("rtk_gnss_observation", LAUNCH)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def test_custom_topics_are_routed_to_their_owning_nodes() -> None:
    module = _launch_module()
    observer = module.observer_parameters(
        legacy_status_topic="/test/legacy/status",
        legacy_fix_topic="/test/legacy/fix",
        legacy_rtcm_topic="/test/legacy/rtcm",
        canonical_status_topic="/test/canonical/status",
        canonical_rtcm_topic="/test/canonical/rtcm",
        delivery_backend="disabled",
        legacy_rtcm_type="uint8_multi_array",
        stale_timeout_s=7.5,
    )
    sink = module.dry_run_parameters(
        canonical_rtcm_topic="/test/canonical/rtcm",
    )

    assert observer == {
        "legacy_status_topic": "/test/legacy/status",
        "legacy_fix_topic": "/test/legacy/fix",
        "legacy_rtcm_topic": "/test/legacy/rtcm",
        "canonical_status_topic": "/test/canonical/status",
        "canonical_rtcm_topic": "/test/canonical/rtcm",
        "delivery_backend": "disabled",
        "legacy_rtcm_type": "uint8_multi_array",
        "stale_timeout_s": 7.5,
    }
    assert sink == {
        "input_topic": "/test/canonical/rtcm",
        "status_topic": "/salus/hardware/rtcm/dry_run_status",
    }
    assert not set(observer).intersection({"input_topic", "status_topic"})
    assert not set(sink).intersection({"legacy_status_topic", "delivery_backend"})


def test_rtk_gnss_observation_launch_defaults_fail_closed() -> None:
    contents = LAUNCH.read_text(encoding="utf-8")

    expected_arguments = {
        "enabled": "true",
        "legacy_status_topic": "/gps/rtk_source/status_json",
        "legacy_fix_topic": "/gps/rtk_status_mavros",
        "legacy_rtcm_topic": "/rtcm",
        "canonical_status_topic": "/salus/hardware/gnss_primary/rtk_status",
        "canonical_rtcm_topic": "/salus/hardware/rtcm/corrections",
        "delivery_backend": "disabled",
        "delivery_enabled": "false",
        "gpsraw_topic": "/mavros_node/gps1/raw",
        "mavros_rtcm_topic": "/mavros_node/send_rtcm",
        "legacy_rtcm_type": "uint8_multi_array",
        "stale_timeout_s": "5.0",
    }
    for name, default in expected_arguments.items():
        assert f'"{name}"' in contents
        assert f'default_value="{default}"' in contents

    assert contents.count('package="salus_hardware"') == 3
    assert 'executable="legacy_rtk_observer"' in contents
    assert 'executable="rtcm_dry_run_sink"' in contents
    assert 'executable="pixhawk_rtk_adapter"' in contents
    assert contents.count('namespace="/salus/hardware"') == 3
    assert 'stale_timeout = float(values["stale_timeout_s"])' in contents


def test_rtk_gnss_observation_launch_excludes_active_stack_and_control() -> None:
    contents = LAUNCH.read_text(encoding="utf-8").lower()

    for forbidden in (
        "ntrip",
        "cmd_vel",
        "uart",
        "robot_state_publisher",
        "fcu_url",
    ):
        assert forbidden not in contents
    assert 'package="mavros"' not in contents
    assert 'executable="mavros_node"' not in contents


def _profile_values():
    return {
        "legacy_status_topic": "/legacy/status",
        "legacy_fix_topic": "/legacy/fix",
        "legacy_rtcm_topic": "/legacy/rtcm",
        "canonical_status_topic": "/canonical/status",
        "canonical_rtcm_topic": "/canonical/rtcm",
        "legacy_rtcm_type": "uint8_multi_array",
        "stale_timeout_s": "5.0",
        "gpsraw_topic": "/mavros/gpsraw",
        "mavros_rtcm_topic": "/mavros/rtcm",
    }


def test_pixhawk_profile_adds_one_explicit_adapter() -> None:
    module = _launch_module()
    disabled = module.build_profile_nodes(
        profile="disabled", delivery_enabled=False, values=_profile_values()
    )
    pixhawk = module.build_profile_nodes(
        profile="pixhawk_mavros", delivery_enabled=False, values=_profile_values()
    )
    assert len(disabled) == 2
    assert len(pixhawk) == 3
    assert module.profile_status_topics(
        profile="disabled", canonical_status_topic="/canonical/status"
    ) == ("/canonical/status", None)
    assert module.profile_status_topics(
        profile="pixhawk_mavros", canonical_status_topic="/canonical/status"
    ) == (
        "/salus/hardware/gnss_primary/rtk_source_status",
        "/canonical/status",
    )


def test_unimplemented_or_incoherent_profiles_fail_closed() -> None:
    import pytest

    module = _launch_module()
    with pytest.raises(ValueError, match="not implemented"):
        module.build_profile_nodes(
            profile="direct_usb", delivery_enabled=False, values=_profile_values()
        )
    with pytest.raises(ValueError, match="requires"):
        module.build_profile_nodes(
            profile="disabled", delivery_enabled=True, values=_profile_values()
        )
    invalid_timeout = _profile_values()
    invalid_timeout["stale_timeout_s"] = "not-a-number"
    with pytest.raises(ValueError):
        module.build_profile_nodes(
            profile="disabled", delivery_enabled=False, values=invalid_timeout
        )
