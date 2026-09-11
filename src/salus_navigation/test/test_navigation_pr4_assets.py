from pathlib import Path
from xml.etree import ElementTree

import yaml


PACKAGE = Path(__file__).parents[1]


def _profile(name: str) -> dict:
    return yaml.safe_load(
        (PACKAGE / "config" / name).read_text(encoding="utf-8")
    )


def test_rpp_search_bound_is_explicit_and_shared_by_real_and_sim_profiles():
    values = []
    for name in (
        "nav2_core_sim.yaml",
        "nav2_core_no_obstacles_sim.yaml",
        "nav2_core_real.yaml",
    ):
        values.append(
            _profile(name)["controller_server"]["ros__parameters"]
            ["FollowPath"]["max_robot_pose_search_dist"]
        )
    assert values == [4.0, 4.0, 4.0]


def test_multi_pose_retry_budget_is_unchanged_pending_transient_fault_evidence():
    tree = ElementTree.parse(PACKAGE / "config" / "navigation_through_poses.xml")
    recovery_nodes = tree.findall(".//RecoveryNode")
    assert [node.attrib["number_of_retries"] for node in recovery_nodes] == [
        "3", "1", "1"
    ]
