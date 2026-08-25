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
