cmake_minimum_required(VERSION 3.5)
project(steelhead_example)

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
find_package(std_msgs REQUIRED)
find_package(steelhead_interfaces REQUIRED)

include_directories(include ${catkin_INCLUDE_DIRS})

add_library(steelhead_example SHARED
  src/component_one.cpp
  src/component_two.cpp
)

rclcpp_components_register_nodes(steelhead_example "steelhead_example::ComponentOne")
rclcpp_components_register_nodes(steelhead_example "steelhead_example::ComponentTwo")

target_compile_definitions(steelhead_example
  PRIVATE "steelhead_example_CPP_BUILDING_DLL"
)

ament_target_dependencies(steelhead_example
  "rclcpp"
  "rclcpp_components"
  "std_msgs"
  "steelhead_interfaces"
)

install(TARGETS
  steelhead_example
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(launch_testing_ament_cmake REQUIRED)
  add_launch_test(
    test/test_example_launch.py
  )
endif()

ament_package()
