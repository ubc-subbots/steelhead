cmake_minimum_required(VERSION 3.10)
project(steelhead_tasks)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(steelhead_interfaces REQUIRED)
find_package(behaviortree_cpp REQUIRED)

include_directories(include ${catkin_INCLUDE_DIRS})

add_library(steelhead_tasks SHARED
  src/automated_competition_gate.cpp
  src/automated_competition_torpedo.cpp
  src/automated_competition_dropper.cpp
  src/automated_competition_octagon.cpp
  src/hardcode_inputs.cpp
  src/pitch_flip.cpp
)

rclcpp_components_register_nodes(steelhead_tasks
  "steelhead_tasks::AutomatedCompetitionGate"
  "steelhead_tasks::AutomatedCompetitionTorpedo"
  "steelhead_tasks::AutomatedCompetitionDropper"
  "steelhead_tasks::AutomatedCompetitionOctagon"
  "steelhead_tasks::HardcodeInputs"
  "steelhead_tasks::PitchFlip"
)

target_compile_definitions(steelhead_tasks
  PRIVATE "steelhead_tasks_CPP_BUILDING_DLL"
)

target_link_libraries(steelhead_tasks PUBLIC rclcpp::rclcpp rclcpp_components::component sensor_msgs::sensor_msgs geometry_msgs::geometry_msgs tf2::tf2 steelhead_interfaces::steelhead_interfaces)

add_library(steelhead_tasks_bt SHARED
  src/bt/drive_for.cpp
  src/bt/gate_task.cpp
  src/bt/register_nodes.cpp
)

target_include_directories(steelhead_tasks_bt PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(steelhead_tasks_bt PUBLIC
  rclcpp::rclcpp
  sensor_msgs::sensor_msgs
  geometry_msgs::geometry_msgs
  steelhead_interfaces::steelhead_interfaces
  behaviortree_cpp::behaviortree_cpp
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  rclcpp
  sensor_msgs
  geometry_msgs
  steelhead_interfaces
  behaviortree_cpp
)

install(DIRECTORY include/
  DESTINATION include
)

install(TARGETS
  steelhead_tasks
  steelhead_tasks_bt
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
