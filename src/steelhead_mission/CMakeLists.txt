cmake_minimum_required(VERSION 3.10)
project(steelhead_mission)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(steelhead_interfaces REQUIRED)

include_directories(include)

add_library(steelhead_mission_tasks SHARED
  src/tasks/drive_for.cpp
  src/tasks/dropper_task.cpp
  src/tasks/gate_task.cpp
  src/tasks/octagon_task.cpp
  src/tasks/pitch_flip_task.cpp
  src/tasks/register_nodes.cpp
  src/tasks/torpedo_task.cpp
)

target_include_directories(steelhead_mission_tasks PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(steelhead_mission_tasks PUBLIC
  behaviortree_cpp::behaviortree_cpp
  rclcpp::rclcpp
  sensor_msgs::sensor_msgs
  tf2::tf2
  steelhead_interfaces::steelhead_interfaces
)

add_executable(mission_runner
  src/mission_runner.cpp
)

target_include_directories(mission_runner PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(mission_runner PUBLIC
  ament_index_cpp::ament_index_cpp
  behaviortree_cpp::behaviortree_cpp
  rclcpp::rclcpp
  std_msgs::std_msgs
  steelhead_interfaces::steelhead_interfaces
  steelhead_mission_tasks
)

install(DIRECTORY include/
  DESTINATION include
)

install(TARGETS
  steelhead_mission_tasks
  mission_runner
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  missions
  DESTINATION share/${PROJECT_NAME}/
)

ament_export_include_directories(include)
ament_export_libraries(steelhead_mission_tasks)
ament_export_dependencies(
  behaviortree_cpp
  rclcpp
  sensor_msgs
  tf2
  steelhead_interfaces
)

ament_package()
