"""Static safeguards for the dedicated physical runtime image."""

from pathlib import Path


REPOSITORY_ROOT = Path(__file__).parents[3]
REAL_DOCKERFILE = REPOSITORY_ROOT / "Dockerfile.real"
DEV_DOCKERFILE = REPOSITORY_ROOT / "Dockerfile"
BUILD_SCRIPT = REPOSITORY_ROOT / "tools/build_real_image.sh"
RUNTIME_GATE = REPOSITORY_ROOT / "tools/validate_real_runtime_image.sh"


def test_real_recipe_contains_physical_runtime_dependencies() -> None:
    contents = REAL_DOCKERFILE.read_text(encoding="utf-8")

    for package in (
        "ros-humble-mavros",
        "ros-humble-mavros-extras",
        "geographiclib-tools",
        "ros-humble-robot-localization",
        "ros-humble-robot-state-publisher",
        "ros-humble-nav2-msgs",
        "ros-humble-nav2-controller",
        "ros-humble-nav2-planner",
        "ros-humble-nav2-bt-navigator",
        "ros-humble-nav2-behaviors",
        "ros-humble-nav2-costmap-2d",
        "ros-humble-nav2-smac-planner",
        "ros-humble-nav2-regulated-pure-pursuit-controller",
        "ros-humble-nav2-collision-monitor",
        "ros-humble-rmw-cyclonedds-cpp",
    ):
        assert package in contents


def test_real_recipe_excludes_development_simulation_dependencies() -> None:
    contents = REAL_DOCKERFILE.read_text(encoding="utf-8")

    for package in (
        "ros-humble-ros-gz-sim",
        "ros-humble-ros-gz-bridge",
        "ros-humble-rviz2",
        "ros-humble-nav2-rviz-plugins",
        "ros-humble-rviz-",
        "ros_gz_sim",
        "ros_gz_bridge",
        "gazebo",
    ):
        assert package not in contents


def test_development_recipe_remains_the_simulation_recipe() -> None:
    contents = DEV_DOCKERFILE.read_text(encoding="utf-8")

    assert "ros-humble-ros-gz-sim" in contents
    assert "ros-humble-ros-gz-bridge" in contents
    assert "ros-humble-rviz2" in contents


def test_real_build_helper_selects_dedicated_image_and_host_ids() -> None:
    contents = BUILD_SCRIPT.read_text(encoding="utf-8")

    assert "--file Dockerfile.real" in contents
    assert "--tag salus-robot:humble-real" in contents
    assert 'USER_UID="$(id -u)"' in contents
    assert 'USER_GID="$(id -g)"' in contents


def test_real_runtime_gate_is_fail_closed_and_software_only() -> None:
    contents = RUNTIME_GATE.read_text(encoding="utf-8")

    assert contents.startswith("#!/usr/bin/env bash")
    assert "colcon build" in contents
    assert "ldd_output=\"$(ldd \"${target}\")\"" in contents
    assert 'grep -Fq "not found"' in contents
    assert "libpcap\\.so\\.0\\.8 => /" in contents
    assert "--network none" in contents
    assert "LD_LIBRARY_PATH" not in contents
    assert "ln -s" not in contents
