cmake_minimum_required(VERSION 3.8)
project(salus_simulation)

find_package(ament_cmake REQUIRED)
install(DIRECTORY launch worlds config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
  ament_add_pytest_test(test_motion_assets test/test_motion_assets.py)
endif()

ament_package()
