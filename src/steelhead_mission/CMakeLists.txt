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
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(steelhead_interfaces REQUIRED)
find_package(steelhead_tasks REQUIRED)

include_directories(include)

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
  visualization_msgs::visualization_msgs
  steelhead_interfaces::steelhead_interfaces
  steelhead_tasks::steelhead_tasks_bt
)

install(TARGETS
  mission_runner
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  missions
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
