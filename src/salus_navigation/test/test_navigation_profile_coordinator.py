from salus_navigation.navigation_profile_coordinator import double_parameter
from rcl_interfaces.msg import ParameterType


def test_double_parameter_uses_explicit_ros_type() -> None:
    parameter = double_parameter("inflation_layer.inflation_radius", 1.4)
    assert parameter.name == "inflation_layer.inflation_radius"
    assert parameter.value.type == ParameterType.PARAMETER_DOUBLE
    assert parameter.value.double_value == 1.4
