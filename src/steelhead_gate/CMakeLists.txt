cmake_minimum_required(VERSION 3.10)
project(steelhead_gate)

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
find_package(OpenCV 4 REQUIRED PATHS /usr/lib/x86_64-linux-gnu/cmake/opencv4 NO_DEFAULT_PATH)
find_package(cv_bridge REQUIRED)
find_package(steelhead_vision_utils REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(steelhead_interfaces REQUIRED)
find_package(tf2 REQUIRED)

include_directories(include ${OpenCV_INCLUDE_DIRS})

add_library(steelhead_gate SHARED
  src/gate_detector.cpp
)

rclcpp_components_register_nodes(steelhead_gate "steelhead_gate::GateDetector")

target_compile_definitions(steelhead_gate
  PRIVATE "steelhead_gate_CPP_BUILDING_DLL"
)
target_compile_options(steelhead_gate
  PRIVATE -Wno-vla
)






target_include_directories(steelhead_gate PRIVATE
  ${rclcpp_INCLUDE_DIRS}
  ${rclcpp_components_INCLUDE_DIRS}
  ${cv_bridge_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${steelhead_vision_utils_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS}
  ${image_transport_INCLUDE_DIRS}
  ${tf2_INCLUDE_DIRS}
  ${steelhead_interfaces_INCLUDE_DIRS}
)

target_link_libraries(steelhead_gate PRIVATE
  rclcpp::rclcpp
  rclcpp_components::component
  cv_bridge::cv_bridge
  ${OpenCV_LIBRARIES}
  steelhead_vision_utils::steelhead_vision_utils
  sensor_msgs::sensor_msgs
  image_transport::image_transport
  tf2::tf2
  steelhead_interfaces::steelhead_interfaces
)


install(TARGETS
  steelhead_gate
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
endif()

ament_package()
