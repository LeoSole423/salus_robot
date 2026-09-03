"""Structural and isolated runtime checks for the physical TF owner."""

from __future__ import annotations

from importlib.util import module_from_spec, spec_from_file_location
import math
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import threading
import time
import xml.etree.ElementTree as ET

import pytest
import rclpy
from rcl_interfaces.srv import GetParameters
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from tf2_msgs.msg import TFMessage


ROOT = Path(__file__).parents[1]
XACRO = ROOT / "urdf" / "salus_robot.urdf.xacro"
LAUNCH = ROOT / "launch" / "description_real.launch.py"


def _launch_module():
    spec = spec_from_file_location("description_real", LAUNCH)
    module = module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def _expanded_real_robot() -> ET.Element:
    with tempfile.NamedTemporaryFile(suffix=".urdf") as output:
        subprocess.run(
            ["xacro", str(XACRO), "use_sim:=false", "-o", output.name],
            check=True,
            capture_output=True,
            text=True,
        )
        subprocess.run(
            ["check_urdf", output.name], check=True, capture_output=True, text=True
        )
        return ET.parse(output.name).getroot()


def test_description_real_launch_has_exactly_one_node() -> None:
    description = _launch_module().generate_launch_description()
    assert len(description.entities) == 1

    contents = LAUNCH.read_text(encoding="utf-8")
    assert contents.count("Node(") == 1
    assert 'package="robot_state_publisher"' in contents
    assert 'executable="robot_state_publisher"' in contents
    assert '"use_sim_time": False' in contents
    assert "use_sim:=false" in contents
    assert '"robot_description"' in contents
    for forbidden in (
        "joint_state_publisher",
        "localization",
        "rs16",
        "perception",
        "nav2",
        "uart",
        "mavros",
        "ntrip",
        "gazebo",
        "static_transform_publisher",
    ):
        assert forbidden not in contents.lower()


def test_real_xacro_keeps_fixed_geometry_and_excludes_simulation_plugins() -> None:
    robot = _expanded_real_robot()
    assert robot.find(".//gazebo") is None
    joints = {element.attrib["name"]: element for element in robot.findall("joint")}

    base_joint = joints["base_footprint_joint"]
    assert base_joint.attrib["type"] == "fixed"
    assert base_joint.find("parent").attrib["link"] == "base_footprint"
    assert base_joint.find("child").attrib["link"] == "base_link"
    assert base_joint.find("origin") is None

    lidar_joint = joints["lidar_link_joint"]
    assert lidar_joint.attrib["type"] == "fixed"
    assert lidar_joint.find("parent").attrib["link"] == "base_link"
    assert lidar_joint.find("child").attrib["link"] == "lidar_link"
    assert lidar_joint.find("origin").attrib == {
        "xyz": "0.92 0 0.65",
        "rpy": "0 0.1745 0",
    }

    for joint_name, xyz in {
        "imu_primary_link_joint": "0.66 0 0.63",
        "imu_link_joint": "0.66 0 0.63",
        "gps_link_joint": "0.66 0.2 0.62",
    }.items():
        assert joints[joint_name].find("origin").attrib["xyz"] == xyz


def test_ros2_launch_show_args_is_available() -> None:
    result = subprocess.run(
        [
            "ros2",
            "launch",
            "salus_description",
            "description_real.launch.py",
            "--show-args",
        ],
        check=True,
        capture_output=True,
        text=True,
    )
    assert result.stderr == ""


class TfRuntimeProbe(Node):
    """Record the static and dynamic TF topics in the isolated test graph."""

    def __init__(self) -> None:
        super().__init__("description_real_runtime_probe")
        self.static_messages: list[TFMessage] = []
        self.dynamic_messages: list[TFMessage] = []
        static_qos = QoSProfile(depth=100)
        static_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        static_qos.reliability = ReliabilityPolicy.RELIABLE
        self.create_subscription(
            TFMessage, "/tf_static", self._on_static, static_qos
        )
        self.create_subscription(TFMessage, "/tf", self._on_dynamic, 100)

    def _on_static(self, message: TFMessage) -> None:
        self.static_messages.append(message)

    def _on_dynamic(self, message: TFMessage) -> None:
        self.dynamic_messages.append(message)

    def wait_for(self, predicate, timeout_s: float) -> bool:
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            if predicate():
                return True
            time.sleep(0.1)
        return False


def _start_launch(log_path: Path) -> subprocess.Popen:
    return subprocess.Popen(
        ["ros2", "launch", "salus_description", "description_real.launch.py"],
        stdout=log_path.open("w", encoding="utf-8"),  # noqa: SIM115
        stderr=subprocess.STDOUT,
        start_new_session=True,
    )


def _stop(process: subprocess.Popen) -> None:
    if process.poll() is not None:
        return
    os.killpg(os.getpgid(process.pid), signal.SIGINT)
    try:
        process.wait(timeout=20)
    except subprocess.TimeoutExpired:  # pragma: no cover - defensive cleanup
        os.killpg(os.getpgid(process.pid), signal.SIGKILL)
        process.wait(timeout=10)


def _all_transforms(messages: list[TFMessage]):
    return [transform for message in messages for transform in message.transforms]


def test_description_real_publishes_only_the_required_static_tf(tmp_path) -> None:
    os.environ["ROS_DOMAIN_ID"] = str(1 + (os.getpid() % 230))
    rclpy.init()
    probe = TfRuntimeProbe()
    executor = SingleThreadedExecutor()
    executor.add_node(probe)
    spinner = threading.Thread(target=executor.spin, daemon=True)
    spinner.start()
    process = None
    log_path = tmp_path / "description_real.log"
    try:
        process = _start_launch(log_path)
        assert probe.wait_for(
            lambda: "robot_state_publisher" in set(probe.get_node_names()),
            timeout_s=25.0,
        ), f"robot_state_publisher never joined the graph; log:\n{log_path.read_text()[-3000:]}"
        assert probe.wait_for(
            lambda: len(_all_transforms(probe.static_messages)) > 0,
            timeout_s=15.0,
        ), f"/tf_static was not published; log:\n{log_path.read_text()[-3000:]}"

        parameter_client = probe.create_client(
            GetParameters, "/robot_state_publisher/get_parameters"
        )
        assert parameter_client.wait_for_service(timeout_sec=10.0)
        parameter_request = GetParameters.Request()
        parameter_request.names = ["use_sim_time", "robot_description"]
        parameter_future = parameter_client.call_async(parameter_request)
        assert probe.wait_for(lambda: parameter_future.done(), timeout_s=10.0)
        parameter_values = parameter_future.result().values
        assert parameter_values[0].bool_value is False
        runtime_robot = ET.fromstring(parameter_values[1].string_value)
        assert runtime_robot.find(".//gazebo") is None

        node_names = set(probe.get_node_names())
        assert node_names == {
            "description_real_runtime_probe",
            "robot_state_publisher",
        }
        assert {
            info.node_name
            for info in probe.get_publishers_info_by_topic("/tf_static")
        } == {"robot_state_publisher"}

        static_transforms = _all_transforms(probe.static_messages)
        pairs = {
            (transform.header.frame_id, transform.child_frame_id)
            for transform in static_transforms
        }
        assert ("base_footprint", "base_link") in pairs
        assert ("base_link", "lidar_link") in pairs
        assert ("odom", "base_footprint") not in pairs
        assert ("map", "odom") not in pairs

        base = next(
            transform
            for transform in static_transforms
            if (transform.header.frame_id, transform.child_frame_id)
            == ("base_footprint", "base_link")
        )
        assert base.transform.translation.x == 0.0
        assert base.transform.translation.y == 0.0
        assert base.transform.translation.z == 0.0
        assert base.transform.rotation.x == 0.0
        assert base.transform.rotation.y == 0.0
        assert base.transform.rotation.z == 0.0
        assert base.transform.rotation.w == 1.0

        lidar = next(
            transform
            for transform in static_transforms
            if (transform.header.frame_id, transform.child_frame_id)
            == ("base_link", "lidar_link")
        )
        assert lidar.transform.translation.x == 0.92
        assert lidar.transform.translation.y == 0.0
        assert lidar.transform.translation.z == 0.65
        assert lidar.transform.rotation.x == 0.0
        assert lidar.transform.rotation.z == 0.0
        assert lidar.transform.rotation.y == pytest.approx(
            math.sin(0.1745 / 2.0), abs=1.0e-9
        )
        assert lidar.transform.rotation.w == pytest.approx(
            math.cos(0.1745 / 2.0), abs=1.0e-9
        )

        all_runtime_transforms = _all_transforms(
            probe.static_messages + probe.dynamic_messages
        )
        runtime_pairs = {
            (transform.header.frame_id, transform.child_frame_id)
            for transform in all_runtime_transforms
        }
        assert ("odom", "base_footprint") not in runtime_pairs
        assert ("map", "odom") not in runtime_pairs
    finally:
        if process is not None:
            _stop(process)
        executor.remove_node(probe)
        probe.destroy_node()
        executor.shutdown()
        rclpy.shutdown()
