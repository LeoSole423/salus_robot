cmake_minimum_required(VERSION 3.8)
project(interfaces)

# This deliberately small package preserves the legacy ROS wire identities
# needed while ROS2_SALUS remains part of the deployment. See README.md.
find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/CmdVelFinal.msg"
  "msg/DriveTelemetry.msg"
  DEPENDENCIES builtin_interfaces geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_legacy_wire_contract
    test/test_legacy_wire_contract.py
  )
endif()

ament_package()
