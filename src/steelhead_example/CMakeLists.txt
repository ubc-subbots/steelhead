cmake_minimum_required(VERSION 3.5)
project(spiderfish_example)

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
find_package(spiderfish_interfaces REQUIRED)

include_directories(include ${catkin_INCLUDE_DIRS})

add_library(spiderfish_example SHARED
  src/component_one.cpp
  src/component_two.cpp
)

rclcpp_components_register_nodes(spiderfish_example "spiderfish_example::ComponentOne")
rclcpp_components_register_nodes(spiderfish_example "spiderfish_example::ComponentTwo")

target_compile_definitions(spiderfish_example
  PRIVATE "spiderfish_example_CPP_BUILDING_DLL"
)

ament_target_dependencies(spiderfish_example
  "rclcpp"
  "rclcpp_components"
  "std_msgs"
  "spiderfish_interfaces"
)

install(TARGETS
  spiderfish_example
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
