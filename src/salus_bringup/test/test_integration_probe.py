import importlib.util
import math
from pathlib import Path
import sys

from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan


PROBE = Path(__file__).parents[3] / "tools" / "integration_probe.py"
sys.path.insert(0, str(PROBE.parent))
SPEC = importlib.util.spec_from_file_location("integration_probe", PROBE)
assert SPEC and SPEC.loader
probe = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = probe
SPEC.loader.exec_module(probe)


def _odom(stamp: int = 1) -> Odometry:
    message = Odometry()
    message.header.stamp.sec = stamp
    message.header.frame_id = "map"
    message.child_frame_id = "base_footprint"
    message.pose.pose.orientation.w = 1.0
    return message


def _scan(stamp: int = 1) -> LaserScan:
    message = LaserScan()
    message.header.stamp.sec = stamp
    message.header.frame_id = "base_footprint"
    message.angle_min = -1.0
    message.angle_max = 1.0
    message.angle_increment = 0.1
    message.range_min = 0.4
    message.range_max = 20.0
    message.ranges = [1.0, math.inf]
    return message


def test_validators_accept_contract_messages() -> None:
    probe.validate_odometry(_odom())
    probe.validate_scan(_scan())


def test_odometry_validator_rejects_nonfinite_bad_quaternion_and_frame() -> None:
    message = _odom()
    message.pose.pose.position.x = math.nan
    try:
        probe.validate_odometry(message)
    except probe.ValidationError as exc:
        assert "NaN" in str(exc)
    else:
        assert False, "nonfinite odometry was accepted"
    message = _odom()
    message.pose.pose.orientation.w = 0.0
    try:
        probe.validate_odometry(message)
    except probe.ValidationError as exc:
        assert "quaternion" in str(exc)
    else:
        assert False, "invalid quaternion was accepted"
    message = _odom()
    message.header.frame_id = "odom"
    try:
        probe.validate_odometry(message)
    except probe.ValidationError as exc:
        assert "frame" in str(exc)
    else:
        assert False, "wrong odometry frame was accepted"


def test_scan_validator_rejects_empty_and_inconsistent_messages() -> None:
    message = _scan()
    message.ranges = []
    try:
        probe.validate_scan(message)
    except probe.ValidationError as exc:
        assert "no ranges" in str(exc)
    else:
        assert False, "empty scan was accepted"
    message = _scan()
    message.intensities = [1.0]
    try:
        probe.validate_scan(message)
    except probe.ValidationError as exc:
        assert "intensities" in str(exc)
    else:
        assert False, "inconsistent scan was accepted"


def test_evidence_requires_increasing_timestamps() -> None:
    evidence = probe.TopicEvidence()
    evidence.record(_odom(3), probe.validate_odometry)
    evidence.record(_odom(3), probe.validate_odometry)
    assert not evidence.has_progress
    evidence.record(_odom(4), probe.validate_odometry)
    assert evidence.has_progress


def test_required_service_readiness_is_conjunctive() -> None:
    class Client:
        def __init__(self, ready):
            self.ready = ready

        def service_is_ready(self):
            return self.ready

    node = object.__new__(probe.IntegrationProbe)
    node.required_services = {"one": Client(True), "two": Client(True)}
    assert node.services_ready()
    node.required_services["two"].ready = False
    assert not node.services_ready()


def test_graph_readiness_requires_unique_authorities() -> None:
    node = object.__new__(probe.IntegrationProbe)
    node.get_node_names_and_namespaces = lambda: [("robot_state_publisher", "/")]
    counts = {"/cmd_vel_final": 1, "/scan_3d_raw": 1}
    node.count_publishers = lambda topic: counts.get(topic, 0)
    assert node.graph_ready()
    counts["/cmd_vel_final"] = 2
    assert not node.graph_ready()
