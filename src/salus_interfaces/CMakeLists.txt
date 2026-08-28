cmake_minimum_required(VERSION 3.8)
project(salus_interfaces)

find_package(ament_cmake REQUIRED)
find_package(ackermann_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/BatteryMissionGuard.msg"
  "msg/CapabilityState.msg"
  "msg/CmdVelFinal.msg"
  "msg/DriveTelemetry.msg"
  "msg/GnssRtkStatus.msg"
  "msg/MeasurementMetadata.msg"
  "msg/NavEvent.msg"
  "msg/NavSnapshotLayers.msg"
  "msg/NavTelemetry.msg"
  "msg/PathHealth.msg"
  "msg/RtcmFrame.msg"
  "msg/SteeringMeasurement.msg"
  "msg/SystemCapabilities.msg"
  "msg/TractionMeasurement.msg"
  "msg/VehicleCommand.msg"
  "srv/BrakeNav.srv"
  "srv/CameraPan.srv"
  "srv/CameraStatus.srv"
  "srv/CameraPtz.srv"
  "srv/CameraPreset.srv"
  "srv/CameraSavePreset.srv"
  "srv/CameraPtzState.srv"
  "srv/CancelNavGoal.srv"
  "srv/EvaluatePathHealth.srv"
  "srv/GetNavState.srv"
  "srv/GetNavSnapshot.srv"
  "srv/SetManualMode.srv"
  "srv/SetNavGoalLL.srv"
  "srv/SetNavigationProfile.srv"
  "srv/SetPatrolMissionLL.srv"
  "srv/CancelPatrolMission.srv"
  "srv/GetPatrolMissionState.srv"
  "srv/RequestReturnHome.srv"
  "srv/SetRouteMissionLL.srv"
  "srv/CancelRouteMission.srv"
  "srv/GetRouteMissionState.srv"
  "srv/SetZonesGeoJson.srv"
  "srv/GetZonesState.srv"
  "srv/SetSimBatteryPreset.srv"
  "srv/SetSimBatteryState.srv"
  DEPENDENCIES ackermann_msgs builtin_interfaces diagnostic_msgs geometry_msgs nav_msgs std_msgs
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
