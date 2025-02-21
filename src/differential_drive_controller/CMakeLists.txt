cmake_minimum_required(VERSION 3.5)
project(differential_drive_controller)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

# Create the C++ node
add_executable(drive_controller src/differential_drive_controller.cpp)
ament_target_dependencies(drive_controller
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
)

# Install C++ node
install(TARGETS
  drive_controller
  DESTINATION lib/${PROJECT_NAME}
)

# Install Python scripts
install(PROGRAMS
  scripts/waypoints_nav.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()