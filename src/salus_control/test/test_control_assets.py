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


def test_simulation_selects_one_command_input_and_defaults_to_legacy() -> None:
    launch = (ROOT / "launch/control_sim.launch.py").read_text(encoding="utf-8")
    source = (ROOT / "salus_control/controller_server_node.py").read_text(
        encoding="utf-8"
    )
    assert '"command_input_mode", default_value="legacy_cmd_vel"' in launch
    assert '"command_input_mode": command_input_mode' in launch
    assert 'if self._command_input_mode == "legacy_cmd_vel"' in source
    assert "canonical_vehicle_command is restricted to the sim_gazebo" in source


def test_legacy_cmd_vel_path_has_no_ignored_angular_rate_parameter() -> None:
    source = (ROOT / "salus_control/controller_server_node.py").read_text(
        encoding="utf-8"
    )
    domain = (ROOT / "salus_control/control_logic.py").read_text(encoding="utf-8")
    assert "max_abs_angular_z" not in source
    assert "max_abs_angular_z" not in domain


def test_real_uart_launch_has_one_explicit_legacy_authority() -> None:
    launch = (ROOT / "launch/control_real_uart.launch.py").read_text(encoding="utf-8")

    assert 'executable="controller_server_node"' in launch
    assert '"transport_backend": "uart"' in launch
    assert '"command_input_mode": "legacy_cmd_vel"' in launch
    assert '"serial_port": serial_port' in launch
    assert '"serial_baud": 115200' in launch
    assert '"serial_tx_hz": 50.0' in launch
    assert '"use_sim_time": False' in launch
    assert "legacy_vehicle_command_node" not in launch
    assert "vehicle_command_comparison_node" not in launch
    assert "canonical_command_dry_run_node" not in launch
    assert "/vehicle/command_shadow" not in launch


def test_read_only_profiles_do_not_include_the_uart_authority() -> None:
    workspace = ROOT.parent
    profiles = [
        workspace / "salus_bringup/launch/real_observation.launch.py",
        workspace / "salus_bringup/launch/real_localization_shadow.launch.py",
    ]

    for profile in profiles:
        source = profile.read_text(encoding="utf-8")
        assert "control_real_uart.launch.py" not in source
        assert 'executable="controller_server_node"' not in source
