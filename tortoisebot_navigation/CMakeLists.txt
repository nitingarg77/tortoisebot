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
           scripts/scan_fusion_node.py scripts/check_fusion.py
           scripts/tf_watch.py scripts/one_goal.py
           scripts/fit_camera_tilt.py
  DESTINATION lib/${PROJECT_NAME}
)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_floor_geometry test/test_floor_geometry.py)
  ament_add_pytest_test(test_places test/test_places.py)
  ament_add_pytest_test(test_scan_geometry test/test_scan_geometry.py)
  ament_add_pytest_test(test_runaway test/test_runaway.py)
  ament_add_pytest_test(test_scan_fusion test/test_scan_fusion.py)
endif()

ament_package()
