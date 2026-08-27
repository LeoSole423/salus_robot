from pathlib import Path


ROOT = Path(__file__).parents[1]


def test_control_node_name_does_not_collide_with_nav2_controller() -> None:
    launch = (ROOT / "launch/control_sim.launch.py").read_text(encoding="utf-8")
    source = (ROOT / "salus_control/controller_server_node.py").read_text(encoding="utf-8")
    assert 'name="salus_controller"' in launch
    assert 'super().__init__("salus_controller")' in source
    assert 'name="controller_server"' not in launch


def test_simulation_preserves_ros_steering_sign() -> None:
    launch = (ROOT / "launch/control_sim.launch.py").read_text(encoding="utf-8")
    source = (ROOT / "salus_control/controller_server_node.py").read_text(encoding="utf-8")
    assert '"sim_invert_actuation_steer_sign": False' in launch
    assert 'declare_parameter("sim_invert_actuation_steer_sign", False)' in source


def test_simulation_launches_non_authoritative_vehicle_command_shadow() -> None:
    launch = (ROOT / "launch/control_sim.launch.py").read_text(encoding="utf-8")
    setup = (ROOT / "setup.py").read_text(encoding="utf-8")
    source = (ROOT / "salus_control/legacy_vehicle_command_node.py").read_text(
        encoding="utf-8"
    )
    assert 'executable="legacy_vehicle_command_node"' in launch
    assert "legacy_vehicle_command_node =" in setup
    assert '"/vehicle/command_shadow"' in source


def test_simulation_launches_shadow_comparison_without_an_actuation_topic() -> None:
    launch = (ROOT / "launch/control_sim.launch.py").read_text(encoding="utf-8")
    setup = (ROOT / "setup.py").read_text(encoding="utf-8")
    source = (
        ROOT / "salus_control/vehicle_command_comparison_node.py"
    ).read_text(encoding="utf-8")
    assert 'executable="vehicle_command_comparison_node"' in launch
    assert "vehicle_command_comparison_node =" in setup
    assert '"/vehicle/command_shadow/diagnostics"' in source
    assert "create_publisher(VehicleCommand" not in source


def test_simulation_launches_non_authoritative_canonical_dry_run() -> None:
    launch = (ROOT / "launch/control_sim.launch.py").read_text(encoding="utf-8")
    setup = (ROOT / "setup.py").read_text(encoding="utf-8")
    source = (ROOT / "salus_control/canonical_command_dry_run_node.py").read_text(
        encoding="utf-8"
    )
    assert 'executable="canonical_command_dry_run_node"' in launch
    assert "canonical_command_dry_run_node =" in setup
    assert '"/vehicle/command_dry_run/diagnostics"' in source
    assert "create_publisher(VehicleCommand" not in source
