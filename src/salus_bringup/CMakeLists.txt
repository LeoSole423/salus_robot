cmake_minimum_required(VERSION 3.8)
project(salus_bringup)

find_package(ament_cmake REQUIRED)
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_integration_sim_launch test/test_integration_sim_launch.py)
  ament_add_pytest_test(test_sim_operational_launch test/test_sim_operational_launch.py)
  ament_add_pytest_test(test_integration_probe test/test_integration_probe.py)
  ament_add_pytest_test(test_lifecycle_readiness_probe test/test_lifecycle_readiness_probe.py)
  ament_add_pytest_test(test_smoke_runtime test/test_smoke_runtime.py)
  ament_add_pytest_test(test_smoke_harness test/test_smoke_harness.py)
  ament_add_pytest_test(test_route_smoke_probe test/test_route_smoke_probe.py)
  ament_add_pytest_test(test_nightly_smokes test/test_nightly_smokes.py)
  ament_add_pytest_test(test_sim_helper test/test_sim_helper.py)
  ament_add_pytest_test(
    test_observe_hardware_contracts test/test_observe_hardware_contracts.py)
  ament_add_pytest_test(
    test_pixhawk_sensor_inputs_launch test/test_pixhawk_sensor_inputs_launch.py)
  ament_add_pytest_test(
    test_rtk_gnss_observation_launch test/test_rtk_gnss_observation_launch.py)
  ament_add_pytest_test(
    test_real_observation_launch test/test_real_observation_launch.py)
  ament_add_pytest_test(
    test_real_stack_integration test/test_real_stack_integration.py)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
