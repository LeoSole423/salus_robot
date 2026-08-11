"""Static checks for the isolated Fortress motion simulation assets."""

from pathlib import Path
import xml.etree.ElementTree as ET


SIMULATION_DIR = Path(__file__).resolve().parents[1]


def test_empty_world_has_motion_and_navsat_required_systems() -> None:
    world = ET.parse(SIMULATION_DIR / "worlds" / "empty.world").getroot()
    plugins = {
        plugin.attrib["name"] for plugin in world.findall(".//plugin")
    }
    assert plugins == {
        "ignition::gazebo::systems::Physics",
        "ignition::gazebo::systems::UserCommands",
        "ignition::gazebo::systems::SceneBroadcaster",
        "ignition::gazebo::systems::Sensors",
        "ignition::gazebo::systems::NavSat",
    }


def test_motion_bridge_preserves_control_and_feedback_endpoints() -> None:
    bridge = (SIMULATION_DIR / "config" / "motion_bridge.yaml").read_text(
        encoding="utf-8"
    )
    for endpoint in (
        "/cmd_vel_gazebo", "/cmd_vel_steer", "/odom_raw", "/odom", "/clock",
        "/gps/fix_raw", "/scan_3d_raw",
    ):
        assert endpoint in bridge
    assert "ROS_TO_GZ" in bridge
    assert bridge.count("GZ_TO_ROS") == 4


def test_motion_launch_keeps_simulation_as_an_isolated_partial_launch() -> None:
    launch_file = (SIMULATION_DIR / "launch" / "motion_sim.launch.py").read_text(
        encoding="utf-8"
    )
    assert "ros_gz_sim" in launch_file
    assert "robot_state_publisher" in launch_file
    assert "control_sim.launch.py" not in launch_file
    assert "use_sim:=true" in launch_file
