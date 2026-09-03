import importlib.util
from pathlib import Path


LAUNCH = Path(__file__).parents[1] / "launch" / "ntrip_rtcm_source_real.launch.py"


def test_ntrip_real_launch_has_one_acquisition_owner_and_no_delivery_owner() -> None:
    source = LAUNCH.read_text(encoding="utf-8")
    assert source.count('executable="ntrip_rtcm_source"') == 1
    assert source.count('package="salus_hardware"') == 1
    assert "config_path" in source
    assert "active_source_id" in source
    for forbidden in ("mavros", "send_rtcm", "pixhawk", "rs16", "uart", "nav2"):
        assert forbidden not in source.lower()


def test_ntrip_real_launch_is_importable() -> None:
    spec = importlib.util.spec_from_file_location("ntrip_real", LAUNCH)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    description = module.generate_launch_description()
    assert len(description.entities) == 3


def test_real_observation_launch_stays_without_ntrip() -> None:
    observation = (
        Path(__file__).parents[2]
        / "salus_bringup/launch/real_observation.launch.py"
    ).read_text(encoding="utf-8").lower()
    assert "ntrip" not in observation
    assert "rtcm" in observation
