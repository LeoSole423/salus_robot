cmake_minimum_required(VERSION 3.18)
project(salus_perception)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Development.Module)

ament_python_install_package(${PROJECT_NAME})
ament_get_python_install_dir(PYTHON_INSTALL_DIR)
Python3_add_library(_radial_ground MODULE WITH_SOABI cpp/radial_ground.cpp)
target_compile_options(_radial_ground PRIVATE -O3 -ffp-contract=off)
install(TARGETS _radial_ground LIBRARY DESTINATION "${PYTHON_INSTALL_DIR}/${PROJECT_NAME}")
install(PROGRAMS scripts/cloud_normalizer scripts/scan_ground_filter scripts/scan_noise_filter
  scripts/scan_preview DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(perception_tests test TIMEOUT 240)
endif()

ament_package()
