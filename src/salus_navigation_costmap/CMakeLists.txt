cmake_minimum_required(VERSION 3.8)
project(salus_navigation_costmap)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(salus_interfaces REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/vector_keepout_geometry.cpp
  src/vector_keepout_layer.cpp)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_include_directories(${PROJECT_NAME} PUBLIC include)
ament_target_dependencies(${PROJECT_NAME} geometry_msgs nav2_costmap_2d pluginlib rclcpp salus_interfaces tf2 tf2_geometry_msgs)
pluginlib_export_plugin_description_file(nav2_costmap_2d vector_keepout_layer.xml)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_vector_keepout_geometry test/test_vector_keepout_geometry.cpp src/vector_keepout_geometry.cpp)
  target_include_directories(test_vector_keepout_geometry PRIVATE include)
endif()

install(TARGETS ${PROJECT_NAME} DESTINATION lib)
install(DIRECTORY include/ DESTINATION include)
install(FILES vector_keepout_layer.xml DESTINATION share/${PROJECT_NAME})
ament_package()
