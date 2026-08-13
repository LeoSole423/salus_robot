cmake_minimum_required(VERSION 3.8)
project(salus_navigation_bt)

find_package(ament_cmake REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(nav2_behavior_tree REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(salus_interfaces REQUIRED)

add_library(salus_path_health_condition_bt_node SHARED
  src/path_health_condition.cpp
)
target_compile_definitions(salus_path_health_condition_bt_node PRIVATE BT_PLUGIN_EXPORT)
ament_target_dependencies(salus_path_health_condition_bt_node
  behaviortree_cpp_v3 nav2_behavior_tree nav_msgs rclcpp salus_interfaces
)
install(TARGETS salus_path_health_condition_bt_node LIBRARY DESTINATION lib)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
