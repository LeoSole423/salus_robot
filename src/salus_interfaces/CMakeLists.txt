cmake_minimum_required(VERSION 3.8)
project(salus_interfaces)

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/BatteryMissionGuard.msg"
  "msg/CmdVelFinal.msg"
  "msg/DriveTelemetry.msg"
  "msg/NavEvent.msg"
  "msg/NavTelemetry.msg"
  "msg/PathHealth.msg"
  "srv/BrakeNav.srv"
  "srv/CancelNavGoal.srv"
  "srv/GetNavState.srv"
  "srv/SetManualMode.srv"
  "srv/SetNavGoalLL.srv"
  "srv/SetZonesGeoJson.srv"
  "srv/GetZonesState.srv"
  "srv/SetSimBatteryPreset.srv"
  "srv/SetSimBatteryState.srv"
  DEPENDENCIES builtin_interfaces diagnostic_msgs geometry_msgs
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
