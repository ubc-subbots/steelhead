cmake_minimum_required(VERSION 3.8)
project(steelhead_claw)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(gazebo_dev REQUIRED)
find_package(gazebo_ros REQUIRED)

# Build executables
add_executable(claw_controller src/claw_controller.cpp)
ament_target_dependencies(claw_controller 
  rclcpp 
  std_msgs 
  std_srvs
)

add_executable(claw_hardware_interface src/claw_hardware_interface.cpp)
ament_target_dependencies(claw_hardware_interface 
  rclcpp 
  std_msgs
)

# Build Gazebo plugin
add_library(gazebo_claw_plugin SHARED src/gazebo_claw_plugin.cpp)
ament_target_dependencies(gazebo_claw_plugin 
  gazebo_dev 
  gazebo_ros 
  rclcpp 
  std_msgs
)

# Install targets
install(TARGETS
  claw_controller
  claw_hardware_interface
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  gazebo_claw_plugin
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  urdf
  launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()