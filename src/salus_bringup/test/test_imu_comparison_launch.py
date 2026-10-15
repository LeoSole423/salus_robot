from pathlib import Path
import runpy

from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node


def test_comparison_composition_never_starts_control_or_selects_an_imu():
    path = Path(__file__).parents[1] / 'launch' / 'imu_comparison.launch.py'
    description = runpy.run_path(str(path))['generate_launch_description']()
    nodes = [entity for entity in description.entities if isinstance(entity, Node)]
    assert len(nodes) == 3
    assert {n.node_package for n in nodes} == {'salus_hardware', 'tf2_ros'}
    text = path.read_text()
    assert 'controller_server' not in text and 'imu_selector' not in text
    assert 'mount_configured' in text and 'IfCondition' in text
    assert 'base_footprint' in text and 'bmi088_link' in text
    declared = [e for e in description.entities if isinstance(e, DeclareLaunchArgument)]
    mount = next(e for e in declared if e.name == 'mount_configured')
    assert ''.join(v.text for v in mount.default_value) == 'false'
