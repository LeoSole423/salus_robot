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


def test_vector_keepout_long_range_world_preserves_systems_and_large_ground() -> None:
    world_path = SIMULATION_DIR / "worlds" / "vector_keepout_long_range.world"
    world = world_path.read_text(encoding="utf-8")
    assert '<world name="salus_empty">' in world
    assert "ignition::gazebo::systems::UserCommands" in world
    assert "ignition::gazebo::systems::NavSat" in world
    assert "<size>1200 1200</size>" in world


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
    assert bridge.count("GZ_TO_ROS") == 5
    assert 'ros_topic_name: "/scan_3d_raw"' in bridge
    assert 'gz_topic_name: "/lidar/points"' in bridge
    assert 'gz_type_name: "gz.msgs.PointCloudPacked"' in bridge


def test_lidar_sensor_and_world_keep_a_detectable_collision_target() -> None:
    description_root = SIMULATION_DIR.parent / "salus_description" / "urdf"
    gazebo_motion = (description_root / "components" / "gazebo_motion.xacro").read_text(
        encoding="utf-8"
    )
    world = (SIMULATION_DIR / "worlds" / "empty.world").read_text(encoding="utf-8")
    assert 'sensor name="lidar_3d_sensor" type="gpu_lidar"' in gazebo_motion
    assert "<topic>/lidar</topic>" in gazebo_motion
    assert 'model name="lidar_test_obstacle"' in world
    assert '<collision name="collision">' in world


def test_motion_launch_keeps_simulation_as_an_isolated_partial_launch() -> None:
    launch_file = (SIMULATION_DIR / "launch" / "motion_sim.launch.py").read_text(
        encoding="utf-8"
    )
    assert "ros_gz_sim" in launch_file
    assert "robot_state_publisher" in launch_file
    assert "control_sim.launch.py" not in launch_file
    assert "use_sim:=true" in launch_file


def test_physical_reference_is_read_only_and_geometry_uses_fortress_keys():
    description = SIMULATION_DIR.parent / "salus_description/urdf/components/gazebo_motion.xacro"
    root = ET.parse(description).getroot()
    plugins = {p.attrib["name"]: p for p in root.findall(".//plugin")}
    ackermann = plugins["ignition::gazebo::systems::AckermannSteering"]
    assert float(ackermann.findtext("wheel_base")) == 0.94
    assert float(ackermann.findtext("kingpin_width")) == 0.75
    assert ackermann.find("wheelbase") is None
    truth = plugins["ignition::gazebo::systems::OdometryPublisher"]
    assert truth.findtext("odom_topic") == "/simulation/ground_truth"
    assert truth.findtext("tf_topic") != "/tf"
    assert truth.findtext("odom_frame") == "map"
    bridge = (SIMULATION_DIR / "config/motion_bridge.yaml").read_text()
    assert 'ros_topic_name: "/simulation/ground_truth"' in bridge
    assert 'ros_topic_name: "/tf"' not in bridge
