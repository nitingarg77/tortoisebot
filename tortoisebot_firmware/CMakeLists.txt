cmake_minimum_required(VERSION 3.8)
project(tortoisebot_firmware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  ament_lint_auto_find_test_dependencies()
endif()

ament_python_install_package(${PROJECT_NAME})
install(PROGRAMS
  scripts/differential.py
  # Drive calibration. These are how left_trim was measured and are the only
  # way to redo it on another chassis, or on this one after a motor change.
  scripts/calibrate_drive.py
  scripts/wheel_balance.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
