"""Structural checks for the canonical SALUS Xacro description."""

from pathlib import Path
import subprocess
import tempfile
import xml.etree.ElementTree as ET


DESCRIPTION_DIR = Path(__file__).resolve().parents[1]
XACRO = DESCRIPTION_DIR / "urdf" / "salus_robot.urdf.xacro"


def _expanded_robot() -> ET.Element:
    with tempfile.NamedTemporaryFile(suffix=".urdf") as output:
        subprocess.run(
            ["xacro", str(XACRO), "use_sim:=true", "-o", output.name],
            check=True,
            capture_output=True,
            text=True,
        )
        subprocess.run(
            ["check_urdf", output.name], check=True, capture_output=True, text=True
        )
        return ET.parse(output.name).getroot()


def test_canonical_description_has_expected_frames_and_geometry() -> None:
    robot = _expanded_robot()
    links = {element.attrib["name"] for element in robot.findall("link")}
    assert {
        "base_footprint",
        "base_link",
        "lidar_link",
        "imu_link",
        "imu_primary_link",
        "gps_link",
    }.issubset(links)

    joints = {element.attrib["name"]: element for element in robot.findall("joint")}
    assert set(joints).issuperset(
        {
            "base_footprint_joint",
            "rear_left_wheel_joint",
            "rear_right_wheel_joint",
            "front_left_steer_joint",
            "front_right_steer_joint",
        }
    )
    assert joints["front_left_steer_joint"].find("limit").attrib == {
        "lower": "-0.5235987756",
        "upper": "0.5235987756",
        "effort": "100",
        "velocity": "3.0",
    }

    for inertia in robot.findall(".//inertia"):
        assert all(float(value) > 0.0 for value in inertia.attrib.values() if value != "0")


def test_simulation_plugin_keeps_ackermann_dimensions() -> None:
    robot = _expanded_robot()
    plugin = robot.find(".//plugin[@name='ignition::gazebo::systems::AckermannSteering']")
    assert plugin is not None
    assert plugin.findtext("wheelbase") == "0.94"
    assert plugin.findtext("wheel_separation") == "0.75"
    assert plugin.findtext("wheel_radius") == "0.24"
    assert plugin.findtext("topic") == "/cmd_vel_steer"
    assert plugin.findtext("publish_odom_tf") == "false"
