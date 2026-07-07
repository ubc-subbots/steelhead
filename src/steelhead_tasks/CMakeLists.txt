cmake_minimum_required(VERSION 3.5)
project(steelhead_tasks)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
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

include_directories(include ${catkin_INCLUDE_DIRS})

add_library(steelhead_tasks SHARED
  src/hardcode_inputs.cpp
  src/pitch_flip.cpp
)

rclcpp_components_register_nodes(steelhead_tasks "steelhead_tasks::HardcodeInputs")
rclcpp_components_register_nodes(steelhead_tasks "steelhead_tasks::PitchFlip")

target_compile_definitions(steelhead_tasks
  PRIVATE "steelhead_tasks_CPP_BUILDING_DLL"
)

ament_target_dependencies(steelhead_tasks
  "rclcpp"
  "rclcpp_components"
  "sensor_msgs"
  "geometry_msgs"
  "tf2"
  "steelhead_interfaces"
)

install(TARGETS
  steelhead_tasks
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
