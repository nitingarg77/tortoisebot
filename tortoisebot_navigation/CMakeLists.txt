cmake_minimum_required(VERSION 3.8)
project(tortoisebot_navigation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# floor_geometry lives here rather than in scripts/ so it can be imported and
# tested without rclpy. scripts/floor_scan.py imports it.
ament_python_install_package(${PROJECT_NAME})
install(
  DIRECTORY launch config maps
  DESTINATION share/${PROJECT_NAME}
)
install(
  PROGRAMS scripts/pose_watchdog.py scripts/floor_scan.py
  DESTINATION lib/${PROJECT_NAME}
)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_floor_geometry test/test_floor_geometry.py)
endif()

ament_package()
